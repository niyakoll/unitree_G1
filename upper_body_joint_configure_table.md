**G1 UPPER-BODY JOINT REFERENCE TABLE**  


| Joint Name                  | Index | Target Value | Exact Visual Pose (from robot's view)                     | Recommended Use / Comment                          |
|-----------------------------|-------|--------------|------------------------------------------------------------|----------------------------------------------------|
| **RightShoulderPitch**      | 22    | -1.57        | Arm straight backward (behind body)                        | Superman flying, back stretch                      |
|                             |       | -1.0         | Arm 60° backward                                           | Relaxed back pose                                  |
|                             |       | -0.5         | Arm slightly behind torso                                  | Natural rest                                       |
|                             |       | 0.0          | Arm hanging straight down (perfect T-pose)                 | Default standing pose                              |
|                             |       | +0.5         | Arm forward ~30°                                           | Greeting start                                     |
|                             |       | +0.8         | Arm forward ~45°                                           | Gentle wave                                        |
|                             |       | +1.0         | Arm forward ~57° — **BEST wave height**                    | **MOST BEAUTIFUL WAVE**                            |
|                             |       | +1.2         | Arm almost horizontal forward                             | Victory, "I love you", heart pose                  |
|                             |       | +1.57        | Arm straight forward (zombie / point forward)              | Pointing, salute                                   |
|                             |       | +2.0         | Arm high upward                                            | "Hands up!", celebration                           |
| **RightShoulderRoll**       | 23    | -1.57        | Arm twisted inward extremely                               | Avoid — unnatural                                  |
|                             |       | -0.8         | Palm facing backward (arm crossed)                         | Hug, protective pose                               |
|                             |       | -0.4         | Slight inward twist                                        | Natural relaxed arm                                |
|                             |       | 0.0          | Neutral (palm facing inward toward body)                   | Default                                            |
|                             |       | +0.4         | Slight outward twist                                       | Open gesture                                       |
|                             |       | +0.8         | Palm facing forward — **perfect open-hand wave**           | **BEST FOR WAVING**                                |
|                             |       | +1.57        | Palm fully outward                                         | Ballet, dramatic pose                              |
| **RightShoulderYaw**        | 24    | -1.57        | Arm swung far left (across body)                           | Hug, stretch                                       |
|                             |       | -0.8         | Arm across chest                                           | Self-hug                                           |
|                             |       | 0.0          | Arm at side (neutral)                                      | Default                                            |
|                             |       | +0.8         | Arm swung out to right                                     | Open arms, welcoming                               |
|                             |       | +1.57        | Arm far out (ballet fifth position)                        | Grand gesture                                      |
| **RightElbow**              | 25    | 0.0          | Arm completely straight                                    | Pointing, relaxed                                  |
|                             |       | +0.8         | ~45° bend                                                  | Natural arm                                        |
|                             |       | +1.2         | ~70° bend — **PERFECT natural bend**                       | **MOST USED — beautiful & natural**                |
|                             |       | +1.57        | Fully bent (hand to shoulder)                              | Hug, salute, thinking pose                         |
| **RightWristRoll**          | 26    | -1.57        | Palm facing backward                                       | Avoid                                              |
|                             |       | -0.8         | Palm facing down-back                                      | Neutral                                            |
|                             |       | 0.0          | Palm facing down                                           | Default                                            |
|                             |       | +0.8         | Palm facing forward — **perfect open wave**                | **BEST FOR GREETING / WAVING**                     |
|                             |       | +1.57        | Palm straight forward                                      | Handshake, stop signal                             |
| **RightWristPitch**         | 27    | -1.0         | Wrist bent sharply down                                    | Avoid                                              |
|                             |       | 0.0          | Neutral                                                    | Default                                            |
|                             |       | +0.6         | Wrist bent up — **perfect open hand**                      | Wave, stop gesture                                 |
|                             |       | +1.0         | Wrist bent up high                                         | Dramatic open hand                                 |
| **RightWristYaw**           | 28    | -1.57        | Hand twisted fully left                                    | Avoid                                              |
|                             |       | 0.0          | Neutral                                                    | Default                                            |
|                             |       | +1.57        | Hand twisted fully right                                   | Avoid                                              |

### Left Arm = Mirror of Right Arm (Automatic!)

| Right Joint           | → Left Joint Index | Value Rule                     |
|-----------------------|--------------------|--------------------------------|
| RightShoulderPitch    | 15                 | Same value                     |
| RightShoulderRoll     | 16                 | **Negative** value             |
| RightShoulderYaw      | 17                 | **Negative** value             |
| RightElbow            | 18                 | Same value                     |
| RightWristRoll        | 19                 | **Negative** value             |
| RightWristPitch       | 20                 | Same value                     |
| RightWristYaw         | 21                 | **Negative** value             |

### Waist Joints (12–14)

| Joint            | Index | Value | Effect                                      |
|------------------|-------|-------|---------------------------------------------|
| WaistYaw         | 12    | ±0.5  | Gentle torso turn (very stable)             |
|                  |       | ±1.0  | Strong turn — use sparingly                 |
| WaistRoll        | 13    | ±0.3  | Slight lean left/right                      |
| WaistPitch       | 14    | ±0.3  | Slight forward/back bend                    |

**Pro tip**: Keep waist movements small (±0.3 max) unless you want dramatic motion.

### TOP 10 MOST BEAUTIFUL POSES (Copy-Paste Ready)

```python
POSES = {
    "wave":               {15: 1.0, 22: 1.0, 18: 1.2, 25: 1.2, 19: 0.8, 26: 0.8},
    "heart":              {15: 1.2, 22: 1.2, 18: 1.4, 25: 1.4, 16: -0.6, 23: 0.6},
    "victory":            {15: 2.0, 22: 2.0, 18: 0.1, 25: 0.1},
    "hug":                {15: 0.0, 22: 0.0, 18: 1.57, 25: 1.57, 16: -1.0, 23: 1.0},
    "tai_chi_circle":     {15: 0.8, 22: 0.8, 18: 1.0, 25: 1.0, 19: 1.0, 26: -1.0},
    "salute":             {22: 1.57, 25: 0.1},
    "thinker":            {22: 0.3, 25: 1.4},
    "ballet":             {15: 1.57, 22: 1.57, 18: 0.0, 25: 0.0},
    "double_wave":        {15: 1.0, 22: 1.0, 18: 1.2, 25: 1.2},
    "prayer":             {15: 0.0, 22: 0.0, 18: 1.57, 25: 1.57, 19: 0.8, 26: 0.8},
}
```
