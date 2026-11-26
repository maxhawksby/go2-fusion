# Qwen3-VL in the auto mapper: ITE hallway ablation, 2026-11-16 to 2026-11-26

Does the auto mapper's VLM goal ranking, with Qwen3-VL 8B Instruct, help Oak map more? Four simulator arms, 100 start spots each, compared spot by spot.

## State

- **The VLM does not help yet.** With it steering, Oak mapped 7.1 points less. The loss comes from the camera and the wait for an answer, not from the model's picks.
- **Simulator only.** Nothing here ran on Oak.
- **Older simulator settings.** These runs used 0.1 s command lag, a 12 m lidar and true speed on `/odom`, from before the Oak-matched simulator (#107, #115, #145). The results may differ there.

## Setup

- **Code:** `LuisMend12/go2-fusion` master at `05aabe9` (2026-11-16). That fork's auto mapper also has spot memory and the longest-frontier hint, which are not on this repository's master. Every arm ran the same build; only the settings below differ.
- **World:** Noah's ITE hallway world (`ite_hallways.sdf` from `jeth/feat/automap-rework` at `bf8993a`).
- **Runs:** start spots 0-99 of seed 28, 300 s cap, stopped after 180 s without moving 0.25 m, headless, on the university GPU server. Each spot ran every arm, alternating which went first.
- **Model:** `qwen3-vl:8b-instruct` on Ollama 0.40.16, loaded before the runs (the first answer after loading takes about 65 s).
- **When:** fork-master and vlm-active ran on 2026-11-16, camera-only and vlm-shadow on 2026-11-26, on the same machine and build.

| Arm | Camera | Qwen3-VL | Who picks goals |
|---|---|---|---|
| fork-master | off | off | map score |
| camera-only | 5 pictures/s | off | map score |
| vlm-shadow | 5 pictures/s | asked, answer logged | map score |
| vlm-active | 5 pictures/s | asked | map score plus the model's score |

## Results

Runs that never sent a goal (the simulator failed to start) are left out, in both arms of a pair.

| Arm | Failed starts | Mean mapped | Median | Ended stuck | Collisions |
|---|---:|---:|---:|---:|---:|
| fork-master | 5 | 58.3 % | 61.9 % | 1 | 0 |
| camera-only | 4 | 54.0 % | 56.9 % | 1 | 0 |
| vlm-shadow | 4 | 51.1 % | 54.4 % | 2 | 0 |
| vlm-active | 1 | 51.4 % | 53.4 % | 2 | 0 |

No run in any arm finished the map inside 300 s.

**Matched start spots** (difference in map coverage; 95 % range by bootstrap; sign test over spots that differ):

| Step | Pairs | Difference | 95 % range | Better / worse | Sign test |
|---|---:|---:|---|---:|---|
| Camera on (camera-only vs fork-master) | 91 | -3.4 pts | -7.9 to +1.1 | 37 / 54 | p = 0.09 |
| Asking and waiting for Qwen (vlm-shadow vs camera-only) | 92 | -2.1 pts | -6.7 to +2.5 | 36 / 56 | p = 0.05 |
| Qwen's picks count (vlm-active vs vlm-shadow) | 95 | +0.2 pts | -3.7 to +4.3 | 48 / 47 | p = 1.0 |
| Whole VLM (vlm-active vs fork-master) | 94 | -7.1 pts | -10.8 to -3.4 | 30 / 63 | p = 0.001 |

**How often the model had a say:** in 100 runs, vlm-active ranked goals 138 times (agreed with the map 114 times, differed 24) and vlm-shadow 131 times (agreed 102, differed 29). No answer timed out. Most goal choices had no camera picture toward the candidates, so the map score decided anyway.

## What it means

- The whole loss is clear (the range stays below zero). Each of its two parts alone is not; together they account for it.
- Why the camera costs coverage is not measured. A guess: rendering pictures loads the simulator. On Oak the camera is always on, so this part may not carry over.
- The wait is real on Oak too: the explorer pauses up to `vlm_timeout` at each goal choice.
- The model's picks neither helped nor hurt here. With so few rankings per run, a real effect would be hard to see.

## Not tested

- The Oak-matched simulator, and Oak itself.
- Other worlds than the ITE hallways.
- Other models (Qwen3-VL 2B/4B/30B, Thinking variants).

Analysis script and raw results are on the university GPU server under `~/oak_sim/bench/1116_1334_ite100` (not in the repository).
