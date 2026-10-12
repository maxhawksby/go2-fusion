import assert from "node:assert/strict";
import test from "node:test";
import { parseCommand } from "./command-parser.js";

test("recognizes stop words before movement", () => {
  assert.equal(parseCommand("please stop now").type, "stop");
  assert.equal(parseCommand("freeze").type, "stop");
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
});

test("returns unknown for unsupported text", () => {
  assert.equal(parseCommand("bring me a sandwich").type, "unknown");
});
