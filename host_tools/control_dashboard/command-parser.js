const COMMANDS = [
  { type: "stop", label: "Stop", patterns: [/\bstop\b/i, /\bhalt\b/i, /\bfreeze\b/i] },
  { type: "motion", value: "forward", label: "Move forward", patterns: [/\b(move|walk|go) forward\b/i, /\bforward\b/i] },
  { type: "motion", value: "back", label: "Move backward", patterns: [/\b(move|walk|go) back(?:ward)?s?\b/i, /\bback(?:ward)?s?\b/i] },
  { type: "motion", value: "left", label: "Turn left", patterns: [/\bturn left\b/i] },
  { type: "motion", value: "right", label: "Turn right", patterns: [/\bturn right\b/i] },
  { type: "action", value: "recover", label: "Recovery stand", apiId: 1006, patterns: [/\brecover(?:y)?\b/i] },
  { type: "action", value: "stand", label: "Stand up", apiId: 1004, patterns: [/\bstand(?: up)?\b/i] },
  { type: "action", value: "sit", label: "Sit down", apiId: 1009, patterns: [/\bsit(?: down)?\b/i] },
  { type: "action", value: "stretch", label: "Stretch", apiId: 1017, patterns: [/\bstretch\b/i] },
  { type: "action", value: "love", label: "Show love", patterns: [/\bshow (?:me )?love\b/i] },
  { type: "action", value: "dance", label: "Dance", patterns: [/\bdance\b/i] },
];

export function parseCommand(input) {
  const text = input.trim();
  if (!text) return { type: "unknown", label: "No command entered", raw: text };
  const command = COMMANDS.find((candidate) => candidate.patterns.some((pattern) => pattern.test(text)));
  return command ? { ...command, raw: text, patterns: undefined } : { type: "unknown", label: "Command not recognized", raw: text };
}
