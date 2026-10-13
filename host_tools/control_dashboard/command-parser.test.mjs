import assert from "node:assert/strict";
import test from "node:test";
import { parseCommand } from "./command-parser.js";

test("recognizes stop words before movement", () => {
  assert.equal(parseCommand("please stop now").value, "stop");
  assert.equal(parseCommand("freeze").value, "stop");
});

test("maps speech output without treating it as an action", () => {
  assert.deepEqual(parseCommand("say I am here"), {
    type: "tts", label: "Say “I am here”", value: "I am here", raw: "say I am here"
  });
});

test("maps basic movement commands", () => {
  assert.deepEqual(["forward", "back", "left", "right"], [
    parseCommand("walk forward").value,
    parseCommand("go backwards").value,
    parseCommand("turn left").value,
    parseCommand("turn right").value,
  ]);
});

test("maps known robot actions", () => {
  assert.equal(parseCommand("stand up").apiId, 1004);
  assert.equal(parseCommand("sit down").apiId, 1009);
  assert.equal(parseCommand("stretch").apiId, 1017);
  assert.equal(parseCommand("recovery stand").apiId, 1006);
  assert.equal(parseCommand("lay down").apiId, 1005);
  assert.equal(parseCommand("stand down").value, "stand_down");
  assert.equal(parseCommand("pounce").apiId, 1032);
  assert.equal(parseCommand("wiggle butt").type, "unknown");
  assert.equal(parseCommand("wallow").type, "unknown");
});

test("returns unknown for unsupported text", () => {
  assert.equal(parseCommand("bring me a sandwich").type, "unknown");
});
