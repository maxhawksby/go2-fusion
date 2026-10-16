const COMMANDS = [
  { type: "action", value: "stop", label: "Stop", patterns: [/\bstop\b/i, /\bhalt\b/i, /\bfreeze\b/i] },
  { type: "motion", value: "forward", label: "Move forward", patterns: [/\b(move|walk|go) forward\b/i, /\bforward\b/i] },
  { type: "motion", value: "back", label: "Move backward", patterns: [/\b(move|walk|go) back(?:ward)?s?\b/i, /\bback(?:ward)?s?\b/i] },
  { type: "motion", value: "left", label: "Turn left", patterns: [/\bturn left\b/i] },
  { type: "motion", value: "right", label: "Turn right", patterns: [/\bturn right\b/i] },
  { type: "action", value: "recover", label: "Recovery stand", apiId: 1006, patterns: [/\brecover(?:y)?\b/i] },
  { type: "action", value: "stand_down", label: "Lay down", apiId: 1005, patterns: [/\b(?:lay|lie) down\b/i, /\bstand down\b/i] },
  { type: "action", value: "stand", label: "Stand up", apiId: 1004, patterns: [/\bstand(?: up)?\b/i] },
  { type: "action", value: "sit", label: "Sit down", apiId: 1009, patterns: [/\bsit(?: down)?\b/i] },
  { type: "action", value: "stretch", label: "Stretch", apiId: 1017, patterns: [/\bstretch\b/i] },
  { type: "action", value: "pounce", label: "Front pounce", apiId: 1032, patterns: [/\b(?:front )?pounce\b/i] },
  { type: "action", value: "finger_heart", label: "Show love", patterns: [/\bshow me (?:how much you )?love(?: me)?\b/i, /\bshow love\b/i, /\bfinger heart\b/i] },
  { type: "action", value: "dance2", label: "Dance 2", patterns: [/\bdance (?:two|2)\b/i] },
  { type: "action", value: "dance1", label: "Dance 1", patterns: [/\bdance\b/i] },
];

export function parseCommand(input) {
  const text = input.trim().replace(/^hey\s+ben\s+ben\b[\s,.:;-]*/i, "");
  if (!text) return { type: "unknown", label: "No command entered", raw: text };
  const tts = text.match(/^(?:say|speak)\s+(.+)$/i);
  if (tts) return { type: "tts", label: `Say “${tts[1]}”`, value: tts[1], raw: text };
  const command = COMMANDS.find((candidate) => candidate.patterns.some((pattern) => pattern.test(text)));
  return command ? { ...command, raw: text, patterns: undefined } : { type: "unknown", label: "Command not recognized", raw: text };
}
