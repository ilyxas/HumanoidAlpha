# REACT: animation.glb

Self-collision response on, joint limits off, floor off, gravity off; pelvis anchored to the clip;
drives 15.0 Hz critically damped. Deviation = reacted pose vs the original clip
(bone-head position mm, rotation deg).

## XMS_Bundle (10.00 s)

Max deviation: 66 mm / 49° (drives alone, contacts off: 2 mm / 3°).

| body | max mm | max deg | contacts off: mm | deg |
|---|---|---|---|---|
| hand_l | 66 | 49 | 2 | 3 |
| hand_r | 63 | 33 | 2 | 3 |
| calf_l | 62 | 9 | 1 | 0 |
| calf_r | 35 | 10 | 1 | 0 |
| foot_r | 23 | 43 | 1 | 0 |
| foot_l | 16 | 38 | 1 | 1 |
| lowerarm_r | 13 | 17 | 1 | 2 |
| upperarm_r | 9 | 3 | 1 | 0 |
| lowerarm_l | 9 | 18 | 1 | 2 |
| upperarm_l | 5 | 2 | 1 | 0 |
| thigh_l | 1 | 14 | 1 | 0 |
| thigh_r | 1 | 8 | 1 | 0 |

Pushing contacts (force ≥ 0.5 N; depth < 0 = held apart inside the 6 mm margin):

| from–to, s | a | b | deepest, mm | at, s |
|---|---|---|---|---|
| 0.00–3.67 | foot_l | foot_r | 50.0 | 0.00 |
| 0.00–0.00 | calf_l | calf_r | 23.6 | 0.00 |
| 0.00–0.00 | calf_l | foot_r | 16.0 | 0.00 |
| 0.00–0.03 | thigh_l | thigh_r | 13.1 | 0.00 |
| 0.00–1.30 | hand_l | torso_lower | 10.7 | 0.00 |
| 0.00–0.00 | calf_r | foot_l | 2.4 | 0.00 |
| 0.00–0.00 | hand_r | torso_lower | 1.6 | 0.00 |
| 0.00–0.00 | hand_r | thigh_r | 1.1 | 0.00 |
| 0.00–0.00 | calf_r | thigh_l | -6.8 | 0.00 |
| 0.00–0.00 | hand_l | thigh_l | -7.4 | 0.00 |
| 0.00–0.00 | calf_l | thigh_r | -11.0 | 0.00 |
| 0.07–0.30 | hand_r | thigh_r | -10.1 | 0.30 |
| 0.10–0.10 | thigh_l | thigh_r | -5.5 | 0.10 |
| 0.17–0.17 | thigh_l | thigh_r | -2.6 | 0.17 |
| 0.17–0.17 | calf_l | calf_r | -11.9 | 0.17 |
| 0.23–0.33 | thigh_l | thigh_r | -3.1 | 0.23 |
| 0.23–0.23 | calf_l | calf_r | -12.0 | 0.23 |
| 0.30–0.30 | hand_r | torso_lower | -11.9 | 0.30 |
| 0.37–0.37 | hand_r | thigh_r | -11.4 | 0.37 |
| 0.40–0.40 | thigh_l | thigh_r | -6.3 | 0.40 |
| 0.47–0.47 | thigh_l | thigh_r | -3.8 | 0.47 |
| 0.47–0.53 | hand_r | thigh_r | -10.6 | 0.47 |
| 0.53–0.63 | thigh_l | thigh_r | -3.7 | 0.53 |
| 0.60–0.67 | hand_r | thigh_r | -10.4 | 0.60 |
| 0.70–0.70 | thigh_l | thigh_r | -6.5 | 0.70 |
| 0.77–0.77 | thigh_l | thigh_r | -3.8 | 0.77 |
| 0.77–0.83 | hand_r | thigh_r | -10.9 | 0.83 |
| 0.80–0.80 | calf_l | calf_r | -11.6 | 0.80 |
| 0.83–0.93 | thigh_l | thigh_r | -3.7 | 0.83 |
| 0.90–1.17 | calf_l | calf_r | -10.5 | 0.90 |
| 0.90–1.00 | hand_r | thigh_r | -11.5 | 0.93 |
| 1.00–1.00 | thigh_l | thigh_r | -6.4 | 1.00 |
| 1.07–1.07 | thigh_l | thigh_r | -3.3 | 1.07 |
| 1.07–1.07 | hand_r | thigh_r | -11.8 | 1.07 |
| 1.13–1.23 | thigh_l | thigh_r | -3.3 | 1.13 |
| 1.13–1.13 | hand_r | thigh_r | -12.0 | 1.13 |
| 1.17–1.37 | hand_r | torso_lower | -11.5 | 1.17 |
| 1.23–1.27 | calf_l | calf_r | -11.5 | 1.27 |
| 1.30–1.30 | thigh_l | thigh_r | -6.5 | 1.30 |
| 1.37–1.37 | thigh_l | thigh_r | -3.7 | 1.37 |
| 1.37–1.87 | hand_l | torso_lower | -11.5 | 1.40 |
| 1.40–1.53 | calf_l | calf_r | -11.1 | 1.47 |
| 1.43–1.53 | thigh_l | thigh_r | -3.6 | 1.43 |
| 1.43–1.93 | hand_r | torso_lower | -11.4 | 1.93 |
| 1.60–1.60 | thigh_l | thigh_r | -6.5 | 1.60 |
| 1.67–1.67 | thigh_l | thigh_r | -3.5 | 1.67 |
| 1.70–1.77 | calf_l | calf_r | -11.6 | 1.70 |
| 1.73–1.83 | thigh_l | thigh_r | -3.6 | 1.73 |
| 1.87–1.87 | calf_l | calf_r | -11.7 | 1.87 |
| 1.90–1.90 | thigh_l | thigh_r | -6.6 | 1.90 |
| 1.93–2.00 | hand_l | torso_lower | -11.3 | 1.93 |
| 1.93–1.93 | calf_l | calf_r | -11.8 | 1.93 |
| 1.97–1.97 | thigh_l | thigh_r | -3.4 | 1.97 |
| 1.97–2.00 | hand_r | pelvis | -11.6 | 1.97 |
| 2.00–2.00 | calf_l | calf_r | -11.9 | 2.00 |
| 2.03–2.13 | thigh_l | thigh_r | -3.0 | 2.10 |
| 2.07–2.10 | calf_l | calf_r | -11.6 | 2.10 |
| 2.17–2.30 | calf_l | calf_r | -9.6 | 2.27 |
| 2.20–2.20 | thigh_l | thigh_r | -5.9 | 2.20 |
| 2.27–2.27 | thigh_l | thigh_r | -2.9 | 2.27 |
| 2.33–2.50 | thigh_l | thigh_r | -2.6 | 2.33 |
| 2.37–2.43 | hand_r | thigh_r | -11.0 | 2.40 |
| 2.40–2.77 | calf_l | calf_r | -9.3 | 2.63 |
| 2.40–2.50 | hand_l | thigh_l | -10.8 | 2.43 |
| 2.50–2.50 | hand_r | thigh_r | -11.9 | 2.50 |
| 2.57–2.57 | thigh_l | thigh_r | -2.6 | 2.57 |
| 2.63–2.73 | thigh_l | thigh_r | -2.0 | 2.70 |
| 2.73–2.93 | hand_r | thigh_r | -9.1 | 2.87 |
| 2.73–2.73 | hand_l | thigh_l | -11.7 | 2.73 |
| 2.80–2.93 | thigh_l | thigh_r | -4.4 | 2.90 |
| 2.80–2.87 | hand_l | thigh_l | -9.9 | 2.83 |
| 3.00–3.00 | thigh_l | thigh_r | -5.3 | 3.00 |
| 3.00–3.03 | calf_l | calf_r | -11.4 | 3.03 |
| 3.07–3.07 | thigh_l | thigh_r | -1.6 | 3.07 |
| 3.10–3.23 | calf_l | calf_r | -9.4 | 3.20 |
| 3.10–3.10 | hand_r | thigh_r | -11.4 | 3.10 |
| 3.13–3.20 | thigh_l | thigh_r | -3.8 | 3.13 |
| 3.13–3.33 | hand_l | thigh_l | -9.6 | 3.30 |
| 3.17–3.17 | hand_r | thigh_r | -10.0 | 3.17 |
| 3.23–3.27 | hand_r | thigh_r | -10.6 | 3.23 |
| 3.27–3.27 | thigh_l | thigh_r | -3.1 | 3.27 |
| 3.30–3.30 | calf_l | calf_r | -11.3 | 3.30 |
| 3.33–3.43 | thigh_l | thigh_r | -1.1 | 3.40 |
| 3.33–3.33 | hand_r | thigh_r | -10.5 | 3.33 |
| 3.37–3.37 | calf_l | calf_r | -11.8 | 3.37 |
| 3.40–3.40 | hand_l | thigh_l | -10.2 | 3.40 |
| 3.47–3.47 | hand_l | thigh_l | -11.5 | 3.47 |
| 3.47–3.47 | calf_l | calf_r | -11.8 | 3.47 |
| 3.50–3.50 | thigh_l | thigh_r | -5.5 | 3.50 |
| 3.53–3.63 | calf_l | calf_r | -10.0 | 3.57 |
| 3.57–3.57 | thigh_l | thigh_r | -3.0 | 3.57 |
| 3.63–3.77 | thigh_l | thigh_r | -4.9 | 3.63 |
| 3.83–4.00 | hand_r | thigh_r | -11.6 | 3.97 |
| 4.07–4.13 | hand_r | thigh_r | -11.9 | 4.07 |
| 4.20–4.23 | hand_r | thigh_r | -11.9 | 4.20 |
| 4.30–4.40 | hand_r | thigh_r | -11.8 | 4.33 |
| 4.37–4.37 | hand_l | thigh_l | -12.0 | 4.37 |
| 4.43–4.43 | hand_l | thigh_l | -11.9 | 4.43 |
| 4.50–4.60 | hand_l | thigh_l | -11.6 | 4.50 |
| 4.67–4.70 | hand_l | thigh_l | -11.8 | 4.67 |
| 4.80–4.80 | hand_l | thigh_l | -11.8 | 4.80 |
| 4.83–4.93 | hand_r | thigh_r | -11.5 | 4.87 |
| 5.03–5.03 | hand_r | thigh_r | -11.8 | 5.03 |
| 5.10–5.13 | hand_r | thigh_r | -11.9 | 5.10 |
| 5.20–5.33 | hand_l | thigh_l | -11.8 | 5.20 |
| 5.40–5.53 | hand_l | thigh_l | -11.5 | 5.53 |
| 5.50–5.53 | hand_r | thigh_r | -11.7 | 5.53 |
| 5.60–5.70 | hand_l | thigh_l | -11.7 | 5.63 |
| 5.60–5.60 | hand_r | thigh_r | -11.9 | 5.60 |
| 5.67–5.70 | hand_r | thigh_r | -11.4 | 5.70 |
| 5.77–5.77 | hand_r | thigh_r | -11.3 | 5.77 |
| 5.77–6.13 | hand_l | thigh_l | -11.4 | 6.13 |
| 5.87–6.07 | hand_r | thigh_r | -11.3 | 5.93 |
| 6.13–6.30 | hand_r | thigh_r | -11.0 | 6.27 |
| 6.20–6.23 | hand_l | thigh_l | -11.6 | 6.20 |
| 6.30–10.00 | hand_l | thigh_l | -11.3 | 6.57 |
| 6.40–10.00 | hand_r | thigh_r | -9.7 | 7.43 |
