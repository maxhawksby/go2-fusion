"""Offline contract, transport and cancellation regressions; no real model/robot."""

import json
import math
from unittest.mock import patch
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from go2_llm_agent.agent import AgentSession
from go2_llm_agent.commands import InvalidPlan, explicit_durations, is_stop, validate_plan, validate_request
from go2_llm_agent.heading import Heading, HeadingTracker
from go2_llm_agent.ollama_client import ModelError, OllamaClient


def response(action="walk", direction="backward", duration=0.1, unit="seconds"):
    return {"status": "execute", "reason": "Test request.", "steps": [
        {"action": action, "direction": direction, "quantity": duration, "unit": unit}]}


class ContractTests(unittest.TestCase):
    def test_signs_are_derived_from_direction(self):
        for action, direction, expected in [("walk", "forward", (0.2, 0)), ("walk", "backward", (-0.2, 0)),
                                             ("turn", "left", (0, 0.5)), ("turn", "right", (0, -0.5))]:
            with self.subTest(direction=direction):
                self.assertEqual(validate_plan(json.dumps(response(action, direction))).steps[0].velocities(), expected)

    def test_invalid_parameters_fail_closed(self):
        invalid = [response(duration=x) for x in [0, -1, 5.01, True, "1", float("inf"), float("nan")]]
        invalid += [response("walk", "left"), response("turn", "forward"), response("stop", "none", 1)]
        invalid += [{"status": "unsupported", "reason": "No.", "steps": response()["steps"]},
                    {"status": "execute", "reason": "Empty.", "steps": []},
                    {"status": "execute", "reason": "Too long.", "steps": response("spin", "left", 360, "degrees")["steps"] * 2},
                    {"status": "execute", "reason": "Too many.", "steps": response()["steps"] * 5}]
        for obj in invalid:
            with self.subTest(obj=obj):
                try:
                    result = validate_plan(json.dumps(obj))
                except InvalidPlan:
                    continue
                self.assertEqual(result.status, "unsupported")
                self.assertEqual(result.steps, ())

    def test_angle_defaults_and_walk_default_are_owned_by_code(self):
        for action, direction, text, seconds, angle in [
            ("walk", "forward", "walk forward", 1, None),
            ("turn", "left", "turn left", 0, 90),
            ("spin", "none", "spin around", 0, 360),
        ]:
            plan = validate_request(validate_plan(json.dumps(response(action, direction, None, "default"))), text)
            self.assertEqual(plan.status, "execute")
            self.assertEqual((plan.steps[0].duration_s, plan.steps[0].angle_deg), (seconds, angle))
        raw = response("walk", "forward", 3)["steps"] + response("turn", "left", None, "default")["steps"]
        plan = validate_request(validate_plan(json.dumps({"status":"execute", "reason":"Mixed.", "steps":raw})),
                                "walk forward for 3 seconds then turn left")
        self.assertEqual([(s.duration_s, s.angle_deg) for s in plan.steps], [(3, None), (0, 90)])

    def test_angles_and_source_quantities_cannot_be_substituted(self):
        for raw in [response("walk", "forward", 90, "degrees"), response("turn", "left", 90, "default")]:
            with self.assertRaises(InvalidPlan):
                validate_plan(json.dumps(raw))
        for amount in (0, 361):
            self.assertEqual(validate_plan(json.dumps(response("turn", "left", amount, "degrees"))).steps, ())
        plan = validate_plan(json.dumps(response("turn", "left", 90, "degrees")))
        self.assertEqual(validate_request(plan, "turn left 90 degrees"), plan)
        with self.assertRaises(InvalidPlan):
            validate_request(plan, "turn left for 3 seconds")
        with self.assertRaises(InvalidPlan):
            validate_request(plan, "turn left 180 degrees")
        timed = validate_plan(json.dumps(response("walk", "forward", 3)))
        self.assertEqual(validate_request(timed, "walk forward for three steps").steps, ())
        spin = validate_plan(json.dumps(response("spin", "right", 0.5)))
        normalized = validate_request(spin, "rotate clockwise for 0.5 seconds")
        self.assertEqual((normalized.steps[0].action, normalized.steps[0].duration_s), ("turn", 0.5))

    def test_untrusted_json_shapes(self):
        samples = ["not json", "[]", '{"status":"chat","status":"execute","reason":"x","steps":[]}',
                   '{"status":"chat","reason":"x","steps":[],"code":"print(1)"}']
        for content in samples:
            with self.subTest(content=content), self.assertRaises(InvalidPlan):
                validate_plan(content)

    def test_non_motion_and_stop(self):
        for status in ("chat", "clarify", "unsupported"):
            self.assertEqual(validate_plan(json.dumps({"status": status, "reason": "No movement.", "steps": []})).steps, ())
        self.assertTrue(is_stop(" Please STOP! "))
        self.assertFalse(is_stop("don't stop"))

    def test_missing_source_details_cannot_be_invented(self):
        plan = validate_plan(json.dumps(response("walk", "forward", 1)))
        for text in ("walk for one second", "move for one second"):
            self.assertEqual(validate_request(plan, text).status, "clarify")
        for text in ("walk forward for 100 seconds", "walk forward for two seconds"):
            with self.assertRaises(InvalidPlan):
                validate_request(plan, text)

    def test_source_evidence_supports_units_and_fractional_words(self):
        self.assertEqual(explicit_durations("1 second, half a second, 500ms, one and a half seconds"), [1, 0.5, 0.5, 1.5])
        self.assertEqual(explicit_durations("minus one second, -0.25s, .5sec, a quarter second"), [-1, -0.25, 0.5, 0.25])
        plan = validate_plan(json.dumps(response("turn", "right", 0.5)))
        self.assertEqual(validate_request(plan, "rotate clockwise for 500 milliseconds"), plan)

    def test_negations_conditions_and_direction_order_cannot_move(self):
        plan = validate_plan(json.dumps(response("walk", "forward", 1)))
        for text in ("Don't walk forward for one second", "Do not walk forward for one second",
                     "If the room is clear, walk forward for one second"):
            self.assertEqual(validate_request(plan, text).steps, ())
        turn = validate_plan(json.dumps(response("turn", "right", 1)))
        with self.assertRaises(InvalidPlan):
            validate_request(turn, "Turn counter-clockwise for one second")
        with self.assertRaises(InvalidPlan):
            validate_request(turn, "Turn right then left for one second")


class RecordingMotion:
    def __init__(self):
        self.samples = []
        self.moving = threading.Event()
        self.closed = False

    def publish_twist(self, x, z=0):
        if self.closed:
            raise RuntimeError("Publisher is closed.")
        self.samples.append((time.monotonic(), x, z))
        if x or z:
            self.moving.set()

    def publish_stop(self):
        self.publish_twist(0, 0)

    def close(self):
        self.closed = True

    def read_heading(self):
        return None


class HeadingTests(unittest.TestCase):
    def test_full_circle_unwraps_in_both_directions(self):
        for sign in (1, -1):
            tracker = HeadingTracker()
            for degrees in range(0, 361, 10):
                yaw = sign * math.radians(degrees)
                tracker.update((0, 0, math.sin(yaw / 2), math.cos(yaw / 2)), "odom")
            self.assertAlmostEqual(tracker.read().radians, sign * 2 * math.pi)

    def test_invalid_samples_do_not_refresh_and_frame_change_is_rejected(self):
        tracker = HeadingTracker()
        tracker.update((0, 0, 0, 1), "odom")
        valid = tracker.read()
        tracker.update((0, 0, 0, 0), "odom")
        tracker.update((0, 0, float("nan"), 1), "odom")
        self.assertEqual(tracker.read(), valid)
        tracker.update((0, 0, 0, 1), "map")
        with self.assertRaises(RuntimeError):
            tracker.read()


class StaticClient:
    def __init__(self, plan=None):
        self.calls = 0
        self.plan = plan or validate_plan(json.dumps(response()))

    def interpret(self, text):
        self.calls += 1
        return self.plan


class ExecutionTests(unittest.TestCase):
    def test_spin_waits_for_full_heading_change_and_then_stops(self):
        class MovingHeading(RecordingMotion):
            yaw = 0

            def read_heading(self):
                if self.moving.is_set():
                    self.yaw += 0.4 * (1 if self.samples[-1][2] > 0 else -1)
                return Heading(self.yaw, time.monotonic())

        for direction, sign in (("left", 1), ("right", -1)):
            motion = MovingHeading()
            plan = validate_plan(json.dumps(response("spin", direction, None, "default")))
            session = AgentSession(StaticClient(plan), motion, output=lambda _: None)
            session.submit("spin")
            session.wait()
            self.assertIsNone(session.last_error)
            self.assertGreaterEqual(sign * motion.yaw, math.radians(358))
            self.assertTrue(all(z * sign > 0 for _, _, z in motion.samples if z))
            self.assertEqual(motion.samples[-1][1:], (0, 0))
            session.close()

    def test_missing_heading_refuses_whole_mixed_plan_before_walking(self):
        raw = response(duration=0.1)["steps"] + response("turn", "left", None, "default")["steps"]
        plan = validate_plan(json.dumps({"status":"execute", "reason":"Mixed.", "steps":raw}))
        motion = RecordingMotion()
        session = AgentSession(StaticClient(plan), motion, output=lambda _: None)
        session.submit("walk backward then turn left")
        session.wait()
        self.assertIsNotNone(session.last_error)
        self.assertFalse(motion.moving.is_set())
        session.close()

    def test_small_angle_is_not_treated_as_already_complete(self):
        class SmallTurn(RecordingMotion):
            def read_heading(self):
                return Heading(math.radians(1) if self.moving.is_set() else 0, time.monotonic())

        motion = SmallTurn()
        session = AgentSession(StaticClient(validate_plan(json.dumps(response("turn", "left", 1, "degrees")))), motion, output=lambda _: None)
        session.submit("turn left one degree")
        session.wait()
        self.assertIsNone(session.last_error)
        self.assertTrue(motion.moving.is_set())
        self.assertEqual(motion.samples[-1][1:], (0, 0))
        session.close()

    def test_stale_heading_stops_and_skips_remaining_actions(self):
        class LostHeading(RecordingMotion):
            def read_heading(self):
                return Heading(0, time.monotonic() - (2 if self.moving.is_set() else 0))

        raw = response("turn", "left", 90, "degrees")["steps"] + response()["steps"]
        motion = LostHeading()
        session = AgentSession(StaticClient(validate_plan(json.dumps({"status":"execute", "reason":"Mixed.", "steps":raw}))), motion, output=lambda _: None)
        session.submit("turn left then walk backward")
        session.wait()
        self.assertIsNotNone(session.last_error)
        self.assertTrue(motion.moving.is_set())
        self.assertTrue(all(x == 0 for _, x, _ in motion.samples))
        self.assertEqual(motion.samples[-1][1:], (0, 0))
        session.close()

    def test_turn_timeout_and_cancellation_both_stop(self):
        class StationaryHeading(RecordingMotion):
            def read_heading(self):
                return Heading(0, time.monotonic())

        plan = validate_plan(json.dumps(response("turn", "left", 90, "degrees")))
        motion = StationaryHeading()
        with patch("go2_llm_agent.commands.Step.time_limit", return_value=0.1):
            session = AgentSession(StaticClient(plan), motion, output=lambda _: None)
            session.submit("turn left")
            session.wait()
        self.assertIsNotNone(session.last_error)
        self.assertEqual(motion.samples[-1][1:], (0, 0))
        session.close()
        motion = StationaryHeading()
        session = AgentSession(StaticClient(plan), motion, output=lambda _: None)
        session.submit("turn left")
        self.assertTrue(motion.moving.wait(1))
        session.stop()
        session.wait()
        self.assertIsNone(session.last_error)
        self.assertEqual(motion.samples[-1][1:], (0, 0))
        session.close()

    def test_timed_backward_ends_in_zero(self):
        motion = RecordingMotion()
        session = AgentSession(StaticClient(), motion, output=lambda _: None)
        session.submit("backward for 0.1 seconds")
        session.wait()
        self.assertIsNone(session.last_error)
        moving = [s for s in motion.samples if s[1] or s[2]]
        self.assertTrue(moving)
        self.assertTrue(all(s[1] == -0.2 and s[2] == 0 for s in moving))
        self.assertGreaterEqual(motion.samples[-1][0] - moving[0][0], 0.09)
        self.assertEqual(motion.samples[-1][1:], (0, 0))
        session.close()

    def test_stop_interrupts_and_never_calls_model(self):
        motion = RecordingMotion()
        client = StaticClient(validate_plan(json.dumps(response(duration=5))))
        session = AgentSession(client, motion, output=lambda _: None)
        session.submit("walk backward for five seconds")
        self.assertTrue(motion.moving.wait(1))
        stopped_at = time.monotonic()
        session.submit("stop")
        session.wait()
        self.assertLess(time.monotonic() - stopped_at, 0.5)
        self.assertEqual(client.calls, 1)
        self.assertTrue(all(x == z == 0 for t, x, z in motion.samples if t >= stopped_at))
        session.close()

    def test_cancel_during_inference_discards_late_plan(self):
        entered, release = threading.Event(), threading.Event()

        class SlowClient(StaticClient):
            def interpret(self, text):
                entered.set()
                release.wait(2)
                return self.plan

        motion = RecordingMotion()
        session = AgentSession(SlowClient(), motion, output=lambda _: None)
        session.submit("backward for a second")
        self.assertTrue(entered.wait(1))
        self.assertFalse(session.submit("another request"))
        session.stop()
        release.set()
        session.wait()
        self.assertFalse(motion.moving.is_set())
        session.close()

    def test_close_during_motion_leaves_no_late_publish(self):
        motion = RecordingMotion()
        session = AgentSession(StaticClient(validate_plan(json.dumps(response(duration=5)))), motion, output=lambda _: None)
        session.submit("walk")
        self.assertTrue(motion.moving.wait(1))
        session.close()
        session.wait()
        self.assertIsNone(session.last_error)
        self.assertEqual(motion.samples[-1][1:], (0, 0))

    def test_non_execute_and_model_error_never_move(self):
        class BrokenClient:
            def interpret(self, text):
                raise ModelError("Timed out.")

        for client in [BrokenClient(), StaticClient(validate_plan('{"status":"clarify","reason":"How long?","steps":[]}'))]:
            motion = RecordingMotion()
            session = AgentSession(client, motion, output=lambda _: None)
            session.submit("walk")
            session.wait()
            self.assertFalse(motion.samples)
            session.close()

    def test_plan_only_never_publishes(self):
        motion = RecordingMotion()
        session = AgentSession(StaticClient(), motion, output=lambda _: None, plan_only=True)
        session.submit("walk backward for a second")
        session.wait()
        session.submit("stop")
        session.close()
        self.assertFalse(motion.samples)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.payloads = []
        self.reply = {"done": True, "done_reason": "stop", "message": {"content": json.dumps(response())}}
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                outer.payloads.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
                body = outer.reply if isinstance(outer.reply, bytes) else json.dumps(outer.reply).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = OllamaClient(f"http://127.0.0.1:{self.server.server_port}", timeout=1)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_actual_http_request_preserves_contract(self):
        plan = self.client.interpret("walk backward for 0.1 seconds")
        self.assertEqual(plan.steps[0].direction, "backward")
        self.assertEqual(self.payloads[0]["format"]["type"], "object")
        self.assertFalse(self.payloads[0]["stream"])
        self.assertEqual(self.payloads[0]["options"]["num_ctx"], 2048)
        self.assertEqual(self.payloads[0]["messages"][-1]["content"], "walk backward for 0.1 seconds")

    def test_bad_server_responses(self):
        for reply in [b"no json", [], {"done": True, "message": {}},
                      {"done": True, "done_reason": "length", "message": {"content": json.dumps(response())}}]:
            self.reply = reply
            with self.subTest(reply=reply), self.assertRaises((ModelError, InvalidPlan)):
                self.client.interpret("walk")

    def test_memory_release_option_is_sent_to_server(self):
        client = OllamaClient(f"http://127.0.0.1:{self.server.server_port}", timeout=1, keep_alive="0")
        client.interpret("walk backward for 0.1 seconds")
        self.assertEqual(self.payloads[-1]["keep_alive"], 0)

    def test_network_failure(self):
        # Close an ephemeral listening socket rather than assuming a particular port is unused.
        import socket
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        with self.assertRaises(ModelError):
            OllamaClient(f"http://127.0.0.1:{port}", timeout=0.1).interpret("walk")


if __name__ == "__main__":
    unittest.main()
