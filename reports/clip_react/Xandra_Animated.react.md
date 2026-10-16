# REACT: Xandra_Animated.glb

Self-collision response on, joint limits off, floor off, gravity off; pelvis anchored to the clip;
drives 15.0 Hz critically damped. Deviation = reacted pose vs the original clip
(bone-head position mm, rotation deg).

## Idle (4.50 s)

Max deviation: 8 mm / 8° (drives alone, contacts off: 1 mm / 1°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| calf_l | 8 | 2 | 0 | 0 |
| calf_r | 7 | 1 | 0 | 0 |
| foot_l | 3 | 2 | 1 | 0 |
| foot_r | 2 | 2 | 1 | 0 |
| thigh_l | 0 | 8 | 0 | 0 |
| thigh_r | 0 | 8 | 0 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.40 | thigh_l | thigh_r | -8.1 | 0.00 |
| 0.47–0.47 | thigh_l | thigh_r | -11.9 | 0.47 |
| 0.53–0.53 | thigh_l | thigh_r | -12.0 | 0.53 |
| 0.70–0.73 | thigh_l | thigh_r | -12.0 | 0.70 |
| 1.03–1.03 | thigh_l | thigh_r | -12.0 | 1.03 |
| 1.13–1.23 | thigh_l | thigh_r | -12.0 | 1.13 |
| 1.33–1.33 | thigh_l | thigh_r | -12.0 | 1.33 |
| 1.40–1.67 | thigh_l | thigh_r | -11.9 | 1.67 |
| 1.73–2.60 | thigh_l | thigh_r | -11.9 | 1.73 |
| 2.67–2.67 | thigh_l | thigh_r | -12.0 | 2.67 |
| 2.83–2.83 | thigh_l | thigh_r | -12.0 | 2.83 |
| 3.50–3.50 | thigh_l | thigh_r | -12.0 | 3.50 |
| 3.57–3.57 | thigh_l | thigh_r | -12.0 | 3.57 |
| 3.63–4.50 | thigh_l | thigh_r | -11.9 | 3.87 |

## Jump (2.87 s)

Max deviation: 19 mm / 35° (drives alone, contacts off: 5 mm / 4°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| calf_l | 19 | 4 | 5 | 1 |
| calf_r | 19 | 4 | 5 | 1 |
| upperarm_l | 16 | 24 | 5 | 1 |
| lowerarm_l | 16 | 5 | 5 | 1 |
| hand_l | 12 | 35 | 5 | 3 |
| foot_r | 11 | 4 | 5 | 1 |
| upperarm_r | 10 | 16 | 5 | 1 |
| hand_r | 10 | 34 | 5 | 4 |
| lowerarm_r | 9 | 3 | 5 | 1 |
| head | 8 | 3 | 5 | 0 |
| foot_l | 8 | 4 | 5 | 3 |
| pelvis | 5 | 0 | 5 | 0 |
| torso_lower | 5 | 0 | 5 | 0 |
| torso_upper | 5 | 1 | 5 | 0 |
| thigh_l | 5 | 9 | 5 | 0 |
| thigh_r | 5 | 9 | 5 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.87 | head | upperarm_l | 7.8 | 0.00 |
| 0.00–0.07 | thigh_l | thigh_r | 0.6 | 0.00 |
| 0.00–0.07 | hand_l | thigh_l | 0.6 | 0.00 |
| 0.00–0.23 | head | upperarm_r | -6.4 | 0.00 |
| 0.13–0.13 | thigh_l | thigh_r | -11.8 | 0.13 |
| 0.20–0.23 | thigh_l | thigh_r | -12.0 | 0.20 |
| 0.27–0.27 | hand_l | thigh_l | -11.2 | 0.27 |
| 0.30–0.30 | head | upperarm_r | -12.0 | 0.30 |
| 0.33–0.93 | hand_l | thigh_l | -10.9 | 0.63 |
| 0.37–0.67 | head | upperarm_r | -11.8 | 0.43 |
| 0.53–0.63 | hand_r | thigh_r | -11.6 | 0.57 |
| 0.70–0.77 | hand_r | thigh_r | -10.8 | 0.77 |
| 0.73–1.17 | head | upperarm_r | -11.6 | 1.13 |
| 0.87–0.90 | hand_r | thigh_r | -11.2 | 0.87 |
| 0.93–1.27 | head | upperarm_l | -11.8 | 0.97 |
| 1.30–1.30 | thigh_l | thigh_r | -11.9 | 1.30 |
| 1.40–1.83 | thigh_l | thigh_r | -11.7 | 1.50 |
| 1.77–1.87 | head | upperarm_l | -12.0 | 1.77 |
| 1.93–1.97 | thigh_l | thigh_r | -11.8 | 1.93 |
| 1.93–2.83 | head | upperarm_l | -11.9 | 1.97 |
| 1.97–2.83 | head | upperarm_r | -11.9 | 2.07 |
| 2.03–2.33 | lowerarm_l | torso_upper | -11.9 | 2.13 |
| 2.07–2.13 | hand_l | thigh_l | -11.6 | 2.10 |
| 2.10–2.13 | hand_r | thigh_r | -12.0 | 2.10 |
| 2.17–2.20 | thigh_l | thigh_r | -11.9 | 2.20 |
| 2.23–2.23 | hand_l | thigh_l | -11.9 | 2.23 |
| 2.23–2.23 | hand_r | thigh_r | -12.0 | 2.23 |
| 2.27–2.83 | thigh_l | thigh_r | -11.8 | 2.27 |
| 2.30–2.30 | hand_r | thigh_r | -11.8 | 2.30 |

## Run_Fwd (0.77 s)

Max deviation: 118 mm / 124° (drives alone, contacts off: 4 mm / 3°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| foot_r | 118 | 74 | 4 | 3 |
| calf_l | 98 | 20 | 3 | 1 |
| calf_r | 92 | 40 | 3 | 0 |
| foot_l | 79 | 124 | 3 | 2 |
| hand_r | 56 | 36 | 2 | 2 |
| hand_l | 52 | 71 | 2 | 2 |
| lowerarm_r | 38 | 30 | 2 | 0 |
| lowerarm_l | 34 | 48 | 2 | 0 |
| upperarm_l | 19 | 22 | 2 | 0 |
| upperarm_r | 18 | 14 | 2 | 0 |
| head | 8 | 4 | 2 | 0 |
| torso_upper | 6 | 11 | 2 | 0 |
| thigh_l | 2 | 68 | 2 | 0 |
| thigh_r | 2 | 47 | 2 | 0 |
| pelvis | 2 | 0 | 2 | 0 |
| torso_lower | 2 | 7 | 2 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.10 | lowerarm_r | torso_upper | 17.0 | 0.00 |
| 0.00–0.77 | head | upperarm_l | 13.5 | 0.00 |
| 0.00–0.20 | lowerarm_l | torso_upper | 13.0 | 0.00 |
| 0.00–0.13 | lowerarm_r | torso_lower | 11.7 | 0.00 |
| 0.00–0.77 | thigh_l | thigh_r | 9.9 | 0.00 |
| 0.00–0.07 | hand_r | torso_upper | 0.9 | 0.00 |
| 0.00–0.00 | head | upperarm_r | -9.4 | 0.00 |
| 0.00–0.00 | hand_r | torso_lower | -11.9 | 0.00 |
| 0.17–0.20 | hand_l | hand_r | -9.7 | 0.20 |
| 0.17–0.17 | lowerarm_r | torso_upper | -11.1 | 0.17 |
| 0.23–0.27 | calf_r | foot_l | -7.5 | 0.23 |
| 0.23–0.70 | lowerarm_r | torso_upper | -11.1 | 0.23 |
| 0.23–0.27 | calf_l | calf_r | -11.5 | 0.23 |
| 0.23–0.23 | head | upperarm_r | -11.8 | 0.23 |
| 0.27–0.30 | lowerarm_l | torso_upper | -11.5 | 0.27 |
| 0.30–0.30 | lowerarm_l | torso_lower | -11.9 | 0.30 |
| 0.30–0.30 | head | upperarm_r | -12.0 | 0.30 |
| 0.33–0.37 | hand_l | torso_lower | -11.5 | 0.37 |
| 0.37–0.47 | hand_l | torso_upper | -11.8 | 0.37 |
| 0.43–0.57 | lowerarm_l | torso_upper | -11.6 | 0.57 |
| 0.50–0.60 | head | upperarm_r | -11.9 | 0.57 |
| 0.57–0.60 | hand_l | hand_r | -10.5 | 0.57 |
| 0.63–0.63 | calf_l | foot_r | -9.7 | 0.63 |
| 0.63–0.73 | lowerarm_r | torso_lower | -11.2 | 0.70 |
| 0.63–0.63 | lowerarm_l | torso_upper | -11.3 | 0.63 |
| 0.67–0.77 | head | upperarm_r | -11.7 | 0.73 |
| 0.70–0.77 | lowerarm_l | torso_upper | -11.2 | 0.73 |
| 0.77–0.77 | lowerarm_r | torso_upper | -11.8 | 0.77 |

## Squat (2.93 s)

Max deviation: 2 mm / 13° (drives alone, contacts off: 2 mm / 5°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| torso_upper | 2 | 0 | 2 | 0 |
| head | 2 | 0 | 2 | 0 |
| upperarm_l | 2 | 1 | 2 | 1 |
| lowerarm_l | 2 | 2 | 2 | 2 |
| upperarm_r | 2 | 1 | 2 | 1 |
| lowerarm_r | 2 | 1 | 2 | 1 |
| thigh_l | 2 | 0 | 2 | 0 |
| thigh_r | 2 | 0 | 2 | 0 |
| pelvis | 2 | 0 | 2 | 0 |
| torso_lower | 2 | 0 | 2 | 0 |
| hand_l | 2 | 5 | 2 | 5 |
| hand_r | 2 | 13 | 2 | 5 |
| calf_l | 2 | 0 | 2 | 0 |
| foot_l | 2 | 0 | 2 | 0 |
| calf_r | 2 | 0 | 2 | 0 |
| foot_r | 2 | 0 | 2 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.00 | head | upperarm_l | -11.0 | 0.00 |
| 0.13–0.17 | hand_r | thigh_r | -12.0 | 0.13 |
| 0.23–0.30 | hand_r | thigh_r | -11.9 | 0.23 |
| 0.37–0.37 | hand_r | thigh_r | -12.0 | 0.37 |
| 1.23–1.73 | head | upperarm_l | -12.0 | 1.23 |

## ToBridge (4.30 s)

Max deviation: 9 mm / 28° (drives alone, contacts off: 9 mm / 28°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| hand_l | 9 | 28 | 9 | 28 |
| hand_r | 9 | 28 | 9 | 28 |
| upperarm_l | 4 | 4 | 4 | 4 |
| upperarm_r | 4 | 4 | 4 | 4 |
| torso_upper | 4 | 0 | 4 | 0 |
| lowerarm_l | 4 | 4 | 4 | 4 |
| lowerarm_r | 4 | 4 | 4 | 4 |
| head | 4 | 1 | 4 | 1 |
| calf_l | 4 | 0 | 4 | 0 |
| calf_r | 4 | 0 | 4 | 0 |
| foot_l | 3 | 0 | 3 | 0 |
| foot_r | 3 | 0 | 3 | 0 |
| torso_lower | 2 | 0 | 2 | 0 |
| thigh_l | 2 | 0 | 2 | 0 |
| thigh_r | 2 | 1 | 2 | 0 |
| pelvis | 2 | 0 | 2 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.30 | thigh_l | thigh_r | -11.2 | 0.00 |
| 2.47–2.47 | head | upperarm_l | -12.0 | 2.47 |
| 2.63–2.77 | head | upperarm_l | -12.0 | 2.63 |
| 2.70–2.73 | head | upperarm_r | -12.0 | 2.70 |
| 2.87–2.87 | head | upperarm_l | -12.0 | 2.87 |
| 2.93–3.03 | head | upperarm_l | -12.0 | 2.93 |
| 2.97–3.10 | head | upperarm_r | -12.0 | 2.97 |
| 3.10–3.30 | head | upperarm_l | -12.0 | 3.10 |
| 3.17–3.30 | head | upperarm_r | -12.0 | 3.17 |
| 3.90–4.07 | head | upperarm_r | -12.0 | 3.90 |
| 3.93–4.10 | head | upperarm_l | -12.0 | 3.93 |

## ToPlank (3.97 s)

Max deviation: 41 mm / 60° (drives alone, contacts off: 8 mm / 12°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| lowerarm_l | 41 | 59 | 6 | 3 |
| lowerarm_r | 40 | 60 | 6 | 3 |
| hand_l | 35 | 24 | 6 | 12 |
| hand_r | 34 | 25 | 6 | 12 |
| foot_l | 22 | 11 | 8 | 1 |
| calf_l | 20 | 30 | 5 | 1 |
| upperarm_l | 19 | 12 | 6 | 1 |
| upperarm_r | 17 | 12 | 6 | 1 |
| torso_upper | 12 | 7 | 6 | 0 |
| foot_r | 10 | 4 | 5 | 1 |
| head | 10 | 7 | 7 | 0 |
| calf_r | 9 | 32 | 4 | 1 |
| thigh_l | 3 | 5 | 3 | 1 |
| thigh_r | 3 | 3 | 3 | 0 |
| pelvis | 3 | 0 | 3 | 0 |
| torso_lower | 3 | 0 | 3 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.17 | hand_r | thigh_r | 3.2 | 0.00 |
| 0.00–0.17 | hand_l | thigh_l | 2.5 | 0.00 |
| 0.00–0.23 | thigh_l | thigh_r | -6.1 | 0.00 |
| 0.50–1.00 | head | upperarm_r | -11.5 | 0.63 |
| 0.50–0.60 | head | upperarm_l | -11.9 | 0.53 |
| 0.63–1.77 | calf_l | lowerarm_l | -10.9 | 0.67 |
| 0.67–0.80 | calf_r | lowerarm_r | -11.2 | 0.70 |
| 0.67–1.93 | head | upperarm_l | -11.6 | 1.03 |
| 0.87–1.37 | calf_r | lowerarm_r | -11.6 | 1.37 |
| 1.10–1.13 | head | upperarm_r | -11.7 | 1.10 |
| 1.20–1.33 | head | upperarm_r | -12.0 | 1.20 |
| 1.40–1.90 | head | upperarm_r | -11.9 | 1.40 |
| 1.60–1.83 | thigh_l | torso_upper | -11.8 | 1.67 |
| 1.83–1.83 | calf_l | lowerarm_l | -11.8 | 1.83 |
| 2.37–2.50 | head | upperarm_r | -11.9 | 2.37 |
| 2.37–2.53 | head | upperarm_l | -12.0 | 2.37 |
| 2.70–3.13 | head | upperarm_l | -12.0 | 2.70 |
| 2.80–3.07 | head | upperarm_r | -12.0 | 2.80 |
| 3.60–3.83 | head | upperarm_l | -12.0 | 3.60 |
| 3.63–3.83 | head | upperarm_r | -12.0 | 3.63 |

## Walk_Back (1.67 s)

Max deviation: 72 mm / 109° (drives alone, contacts off: 2 mm / 3°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| calf_l | 72 | 15 | 1 | 0 |
| calf_r | 67 | 15 | 1 | 0 |
| hand_r | 46 | 109 | 1 | 3 |
| foot_r | 45 | 22 | 2 | 1 |
| foot_l | 39 | 21 | 1 | 1 |
| upperarm_r | 27 | 22 | 1 | 1 |
| hand_l | 20 | 96 | 1 | 2 |
| upperarm_l | 15 | 19 | 1 | 1 |
| lowerarm_r | 12 | 30 | 1 | 1 |
| lowerarm_l | 10 | 8 | 1 | 1 |
| head | 6 | 4 | 1 | 0 |
| torso_lower | 1 | 3 | 1 | 0 |
| thigh_r | 1 | 59 | 1 | 0 |
| thigh_l | 1 | 41 | 1 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.00 | thigh_l | thigh_r | 2.2 | 0.00 |
| 0.00–0.77 | head | upperarm_l | -2.2 | 0.00 |
| 0.00–0.53 | head | upperarm_r | -8.7 | 0.00 |
| 0.07–0.10 | thigh_l | thigh_r | -11.8 | 0.07 |
| 0.30–0.37 | hand_l | thigh_l | -11.2 | 0.37 |
| 0.30–0.83 | thigh_l | thigh_r | -11.3 | 0.80 |
| 0.40–0.40 | lowerarm_r | torso_upper | -12.0 | 0.40 |
| 0.43–0.57 | hand_l | thigh_l | -11.6 | 0.43 |
| 0.63–0.63 | head | upperarm_r | -11.7 | 0.63 |
| 0.67–0.77 | hand_l | thigh_l | -10.5 | 0.70 |
| 0.70–0.77 | hand_r | thigh_r | -11.1 | 0.77 |
| 0.70–1.63 | head | upperarm_r | -11.6 | 0.77 |
| 0.87–0.87 | head | upperarm_l | -12.0 | 0.87 |
| 0.90–0.97 | thigh_l | thigh_r | -11.4 | 0.90 |
| 1.00–1.60 | head | upperarm_l | -11.9 | 1.57 |
| 1.03–1.03 | thigh_l | thigh_r | -12.0 | 1.03 |
| 1.10–1.63 | thigh_l | thigh_r | -10.8 | 1.53 |
| 1.17–1.27 | hand_r | thigh_r | -10.9 | 1.20 |
| 1.30–1.33 | lowerarm_r | thigh_r | -11.2 | 1.33 |
| 1.30–1.30 | lowerarm_r | torso_lower | -11.7 | 1.30 |
| 1.33–1.33 | hand_r | thigh_r | -10.3 | 1.33 |
| 1.37–1.37 | lowerarm_r | torso_lower | -11.9 | 1.37 |
| 1.40–1.53 | hand_r | thigh_r | -10.2 | 1.47 |
| 1.47–1.47 | hand_l | thigh_l | -11.9 | 1.47 |

## Walk_Fwd (1.07 s)

Max deviation: 90 mm / 130° (drives alone, contacts off: 4 mm / 11°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| foot_l | 90 | 80 | 2 | 4 |
| calf_l | 76 | 51 | 3 | 0 |
| foot_r | 64 | 31 | 3 | 8 |
| calf_r | 56 | 104 | 4 | 1 |
| hand_l | 49 | 97 | 2 | 11 |
| hand_r | 35 | 130 | 2 | 10 |
| upperarm_l | 16 | 15 | 1 | 2 |
| lowerarm_l | 12 | 29 | 1 | 3 |
| lowerarm_r | 8 | 31 | 1 | 3 |
| head | 6 | 3 | 1 | 0 |
| upperarm_r | 2 | 3 | 1 | 2 |
| thigh_l | 1 | 66 | 1 | 0 |
| thigh_r | 1 | 69 | 1 | 0 |
| torso_lower | 1 | 4 | 1 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–0.03 | hand_r | thigh_r | 69.2 | 0.00 |
| 0.00–0.00 | hand_l | thigh_l | 30.1 | 0.00 |
| 0.00–0.13 | thigh_l | thigh_r | 9.0 | 0.00 |
| 0.00–0.00 | lowerarm_r | thigh_r | 8.3 | 0.00 |
| 0.00–0.53 | head | upperarm_l | 1.3 | 0.00 |
| 0.00–0.00 | calf_l | calf_r | -5.2 | 0.00 |
| 0.00–0.00 | lowerarm_l | thigh_l | -10.0 | 0.00 |
| 0.00–0.00 | lowerarm_r | torso_lower | -11.7 | 0.00 |
| 0.10–0.10 | lowerarm_r | thigh_r | -11.9 | 0.10 |
| 0.13–0.13 | hand_r | thigh_r | -10.1 | 0.13 |
| 0.27–0.40 | thigh_l | thigh_r | -11.5 | 0.40 |
| 0.47–0.70 | thigh_l | thigh_r | -9.2 | 0.53 |
| 0.47–0.47 | hand_r | thigh_r | -11.9 | 0.47 |
| 0.50–0.50 | lowerarm_l | torso_lower | -11.9 | 0.50 |
| 0.53–0.57 | foot_l | foot_r | -9.7 | 0.57 |
| 0.53–0.57 | hand_r | thigh_r | -11.2 | 0.57 |
| 0.53–0.57 | hand_l | thigh_l | -11.3 | 0.57 |
| 0.53–0.57 | calf_l | calf_r | -11.5 | 0.57 |
| 0.60–1.07 | head | upperarm_l | -11.8 | 0.60 |
| 0.77–1.07 | thigh_l | thigh_r | -10.5 | 1.00 |
| 0.97–0.97 | lowerarm_r | torso_lower | -11.9 | 0.97 |
| 1.00–1.07 | hand_r | thigh_r | -11.3 | 1.00 |
| 1.07–1.07 | hand_l | thigh_l | -11.2 | 1.07 |
| 1.07–1.07 | calf_l | calf_r | -11.3 | 1.07 |
| 1.07–1.07 | lowerarm_r | thigh_r | -11.5 | 1.07 |
| 1.07–1.07 | foot_l | foot_r | -11.8 | 1.07 |
