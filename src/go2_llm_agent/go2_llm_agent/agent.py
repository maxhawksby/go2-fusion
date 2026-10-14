"""Typed requests -> Ollama -> validated actions; simulation is always default."""

import argparse
import json
import math
import os
import threading
import time

from go2_llm_agent.commands import is_stop
from go2_llm_agent.ollama_client import DEFAULT_MODEL, DEFAULT_URL, ModelError, OllamaClient

HEADING_MAX_AGE_S = 1.0
TURN_TOLERANCE_RAD = math.radians(2)


def fresh_heading(motion):
    heading = motion.read_heading()
    if heading is None or time.monotonic() - heading.received_at > HEADING_MAX_AGE_S:
        raise RuntimeError("Fresh /odom heading is required for angle turns; no timed substitute will run.")
    if not math.isfinite(heading.radians):
        raise RuntimeError("Invalid /odom heading.")
    return heading.radians


class AgentSession:
    """One request at a time; terminal input stays available for local stop."""

    def __init__(self, client, motion, output=print, plan_only=False):
        self.client, self.motion, self.output = client, motion, output
        self.plan_only = plan_only
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._worker = None
        self._closed = False
        self.last_error = None

    def submit(self, text):
        if is_stop(text):
            self.stop()
            return True
        if not text.strip():
            return False
        if len(text) > 1000:
            self.output("Command is too long (maximum 1000 characters).")
            return False
        with self._lock:
            if self._closed:
                return False
            if self._worker is not None and self._worker.is_alive():
                self.output("Busy with the current request. Type stop to cancel; wait for it to finish before another request.")
                return False
            self._cancel = threading.Event()
            self.last_error = None
            self._worker = threading.Thread(target=self._run, args=(text, self._cancel), daemon=True)
            self._worker.start()
        return True

    def _run(self, text, cancel):
        started = time.monotonic()
        try:
            self.output("Interpreting: " + text)
            plan = self.client.interpret(text)
            with self._lock:
                if cancel.is_set() or self._closed:
                    self.output("Cancelled; discarded the model response.")
                    return
                self.output("Plan: " + json.dumps(plan.as_dict()))
                self.output(f"Interpretation took {time.monotonic() - started:.2f}s.")
            if self.plan_only or plan.status != "execute":
                self.output(plan.reason)
                return
            # Check before the first action, including a timed walk preceding a turn.
            if any(s.angle_deg is not None for s in plan.steps):
                fresh_heading(self.motion)
            for step in plan.steps:
                with self._lock:
                    if cancel.is_set() or self._closed:
                        return
                    measure = f"{step.angle_deg:g} degrees" if step.angle_deg is not None else f"{step.duration_s:g}s"
                    self.output(f"Action: {step.action} {step.direction}, {measure}")
                try:
                    initial_heading = fresh_heading(self.motion) if step.angle_deg is not None else None
                    deadline = time.monotonic() + step.time_limit()
                    reached = False
                    while time.monotonic() < deadline:
                        with self._lock:
                            if cancel.is_set() or self._closed:
                                return
                            if time.monotonic() >= deadline:
                                break
                            if initial_heading is not None:
                                heading = fresh_heading(self.motion)
                                progress = (heading - initial_heading) * (1 if step.direction == "left" else -1)
                                if progress < -math.radians(10):
                                    raise RuntimeError("Heading moved opposite to the requested turn; stopped.")
                                target = math.radians(step.angle_deg)
                                if progress >= target - min(TURN_TOLERANCE_RAD, target / 2):
                                    reached = True
                                    break
                            # Check and publish under the same lock as stop: no late nonzero command.
                            self.motion.publish_twist(*step.velocities())
                        remaining = deadline - time.monotonic()
                        if remaining <= 0 or cancel.wait(min(0.05, remaining)):
                            break
                    if initial_heading is not None and not reached and not cancel.is_set():
                        raise RuntimeError("Turn did not reach its angle before the timeout; stopped remaining actions.")
                finally:
                    with self._lock:
                        if not self._closed:
                            self.motion.publish_stop()
                if cancel.is_set():
                    return
            self.output("Completed; stopped.")
        except Exception as exc:
            # All model/validation/backend failures fail closed; never execute partial plans.
            with self._lock:
                self.last_error = exc
            self.output("Request failed; no further actions: " + str(exc))

    def stop(self):
        with self._lock:
            self._cancel.set()
            if not self.plan_only:
                self.motion.publish_stop()
        self.output("Stopped the agent's movement; cancelled its pending actions.")

    def wait(self):
        with self._lock:
            worker = self._worker
        if worker is not None:
            worker.join()

    def close(self):
        with self._lock:
            self._closed = True
            self._cancel.set()
            if not self.plan_only:
                self.motion.publish_stop()
            self.motion.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ollama-url", default=os.getenv("OLLAMA_BASE_URL", DEFAULT_URL))
    parser.add_argument("--model", default=os.getenv("OLLAMA_MODEL", DEFAULT_MODEL))
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--keep-alive", choices=("0", "5m"), default=os.getenv("OLLAMA_KEEP_ALIVE", "5m"),
                        help="0 releases model memory after each reply; 5m keeps it loaded (default)")
    parser.add_argument("--mode", choices=("mock", "ros"), default="mock",
                        help="mock prints only (default); ros publishes on /cmd_vel_agent")
    parser.add_argument("--check", action="store_true", help="check server/model without starting ROS")
    parser.add_argument("--plan-only", action="store_true", help="interpret and validate without executing")
    parser.add_argument("--command", help="run one request and exit instead of prompting")
    args = parser.parse_args(argv)
    try:
        client = OllamaClient(args.ollama_url, args.model, args.timeout, keep_alive=args.keep_alive)
        if args.check:
            print(json.dumps(client.check(), indent=2))
            return 0
        # Plan-only is side-effect free even if --mode ros was also supplied.
        if args.mode == "mock" or args.plan_only:
            from go2_llm_agent.ros2_mock_motion import MockMotionController
            motion = MockMotionController()
        else:
            from go2_llm_agent.ros2_motion import RosMotionController
            motion = RosMotionController()
        motion.start()
    except (ValueError, ModelError) as exc:
        print("Setup failed: " + str(exc))
        return 1
    session = AgentSession(client, motion, plan_only=args.plan_only)
    print(f"Mode: {'plan-only' if args.plan_only else args.mode}; model: {args.model}")
    print("Walk defaults to 1s; turn to 90 degrees; spin to 360 degrees left. stop cancels locally; quit exits.")
    try:
        if args.command is not None:
            if not session.submit(args.command):
                return 1
            session.wait()
            return 1 if session.last_error else 0
        while True:
            text = input("go2> ")
            if text.strip().lower() in {"quit", "exit", "/bye"}:
                break
            session.submit(text)
    except (EOFError, KeyboardInterrupt):
        print("\nClosing agent.")
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
