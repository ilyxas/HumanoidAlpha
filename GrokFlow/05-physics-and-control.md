# 05. Физика и управление

Источник чисел — [physics/model/humanoid_alpha.xml](../physics/model/humanoid_alpha.xml), если не сказано иное. Метаданные [physics/model_meta.json](../physics/model_meta.json) с этим XML согласованы по размерностям. [physics/assumptions.json](../physics/assumptions.json) — слой пометок ASSUMPTION, **не** подменяет инерции модели.

## Размерности (XML + model_meta)

| Величина | Значение |
|---|---|
| timestep | 0.002 с (500 Гц) |
| gravity | `0 0 -9.81` |
| integrator | `implicitfast` |
| cone | `elliptic` |
| nbody | 17 (world + 16) |
| njnt | 20 |
| nq | 47 |
| nv | 39 |
| nu | 33 |
| dof_check | 1 free + 7 ball + 12 hinge → nq 47, nv 39, nu 33, все `*_ok: true` |
| total mass | сумма 16 `<inertial mass>` = **89.043737750521** кг; `model_meta.json` пишет `89.04373775048496` (тот же порядок, хвост float расходится) |

`assumptions.json`: `body_mass_kg` **60.0**, сумма `masses_kg` ≈ 60. Это доли до нормализации (`mass_normalize_factor` ≈ 1.089). В рабочем XML стоят **явные** `<inertial>`, compiler `inertiafromgeom="auto"`. Голова: масса 4.278… кг, сфера radius 0.09, density 1401.209880. Старый файл [humanoid_alpha_pre_anatomical.xml](../physics/model/humanoid_alpha_pre_anatomical.xml) — капсулы и `inertiafromgeom="true"`, рантайм его не открывает.

Свободный корень: joint `root` на теле `pelvis`, qpos 7, qvel 6. Старт pelvis в XML: `pos="-0.00000000 -0.01795729 0.92590946"`.

Тела: pelvis, torso_lower, torso_upper, head, upperarm/lowerarm/hand L+R, thigh/calf/foot L+R.

## Суставы

Ball, `limited="true"`, range `0 <конус рад>`. Конус изотропный. [anatomical_ranges.json](../physics/anatomical_ranges.json) прямо говорит: анизотропия **не** смоделирована, замена ball на 3 hinge отвергнута (сломала бы nq и `joint_angles`). Все ball помечены `ENGINEERING`, `needs_external_validation: true`.

| Joint | Конус | Приоры осей в JSON (не лимиты MuJoCo по осям) |
|---|---|---|
| lumbar | 40° (0.6981317007977318) | flex −20..40, lat ±15, twist ±10 |
| thoracic | 25° | flex −15..25, lat ±12, twist ±15 |
| neck | 60° | flex −40..50, lat ±35, twist ±60 — конус задаёт twist |
| shoulder_l / shoulder_r | 150° | flex −60..180, abd −30..120, rot ±90; заметка: анатомический flex 180, конус ужат до 150 |
| hip_l / hip_r | 120° | hip_r `mirror_of` hip_l |

Hinge (диапазоны в градусах из `anatomical_ranges.json`, радианы в XML совпадают с `range_rad` этого файла):

| Joint | Градусы | Метка файла |
|---|---|---|
| knee_l, knee_r | −5 .. 140 | ось R = FACT, ось L = ASSUMPTION_MIRROR; лимиты ENGINEERING |
| elbow_l, elbow_r | 0 .. 150 | ось ASSUMPTION_UNKNOWN_VALIDATION |
| ankle_*_dp | −20 .. 50 | ASSUMPTION |
| ankle_*_ie | −20 .. 20 | ASSUMPTION |
| wrist_*_flex | −70 .. 70 | ASSUMPTION |
| wrist_*_dev | −25 .. 35 | ASSUMPTION |

Оси hinge продублированы в `assumptions.json` → `axes` и в атрибуте `axis` XML. Joint damping 0.08, armature 0.002 (default и повтор на суставах). Мягкие лимиты сустава: `solreflimit="0.002 1"`, `solimplimit="0.95 0.99 0.001"`, margin сустава 0.02.

Ключицы и ball/toes: `engineering_welds` — вварены в torso_upper и в стопы. Отдельных joint в XML нет.

Пальцы в коллизии — часть hull кисти, не суставы ([docs/COLLISIONS.md](../docs/COLLISIONS.md) и docstring build_collision_surfaces).

## 33 мотора

Порядок = порядок `<actuator>` = `model_meta.actuator_order` = каналы `./humanoid act`. Все `<motor ctrllimited="true">`. Gear ball: оси x/y/z по три мотора. Hinge: gear 1.

| Каналы | Имена | ctrlrange Н·м |
|---|---|---|
| 0–2 | lumbar_x/y/z_motor | ±35 |
| 3–5 | thoracic_x/y/z_motor | ±30 |
| 6–8 | neck_x/y/z_motor | ±20 |
| 9–11 | hip_l_x/y/z_motor | ±120 |
| 12–14 | hip_r_x/y/z_motor | ±120 |
| 15–17 | shoulder_l_x/y/z_motor | ±60 |
| 18–20 | shoulder_r_x/y/z_motor | ±60 |
| 21 | knee_l_motor | ±100 |
| 22 | knee_r_motor | ±100 |
| 23–24 | elbow_l/r_motor | ±40 |
| 25–26 | ankle_l_dp/ie_motor | ±40 |
| 27–28 | ankle_r_dp/ie_motor | ±40 |
| 29–30 | wrist_l_flex/dev_motor | ±10 |
| 31–32 | wrist_r_flex/dev_motor | ±10 |

Два разных контракта момента:

- `HumanoidEnv.apply_torques`: **clip** в ctrlrange, длительность режется в [0, 1] с (`MAX_APPLY_DURATION`). Это API агента, не CLI.
- `./humanoid` / `ActuatorAPI.validate`: длина ровно 33, конечные числа, каждый канал внутри ctrlrange, иначе отказ **всего** вектора. В RAW `data.ctrl` пишется как есть, без второго clip.
- Слайдер Torque в консоли: одно значение, отказ вне ctrlrange. В режиме assisted слайдер пишет `u_cmd[i]`, ctrl пересчитывает `compute`.

`ws_bridge` сценарии сами подставляют числа (колено −40/10, плечо 30·sin / 15·cos) в те же имена моторов. Это не CLI.

## Коллизии

Рабочие geom конечностей и торса: mesh `*_surface`, `margin="0.006"`. Пол: plane `floor` size 5 5 0.1, friction `0.8 0.1 0.1`, condim 3, **без** margin 6 мм в теге floor (default geom solref/solimp наследуются). Default geom: `solref="0.004 1"`, `solimp="0.95 0.99 0.001"`.

Явные `<pair>` (margin 0.006, friction пять чисел `0.8 0.8 0.1 0.1 0.1`) связывают distal guard с родителем: рука↔торс, предплечье↔плечо, кисть↔предплечье, бедро↔таз, бедро↔torso_lower, голень↔бедро, стопа↔голень, зеркально L/R.

`<exclude body1="torso_lower" body2="thigh_l"/>` и то же для thigh_r. Других exclude в XML нет.

[collision_guard.py](../physics/collision_guard.py) только для Position (`op joint`):

- шаг сэмпла ≤ 0.005 рад или ≤ 0.002 м;
- пол: допуск глубины `TOLERANCE = 0.002` (комментарий: стопа в rest торчит на 1.61 мм ниже z=0);
- самоконтакт: порог `-SELF_CLEARANCE` где `SELF_CLEARANCE = 0.006`;
- отказ: `Movement blocked by contact: <имена geom>`. Время симуляции не идёт. Уже существующее проникновение можно уменьшать, но не углублять.

Консоль: корень ±10 м отдельно от ориентации. Ball — вектор поворота в кватернион, угол не выше конуса модели.

## Assisted stand (код, не отчёт)

Класс `AssistedStandController`. Формула из docstring и `compute`:

`τ = clip(u_cmd + fade * (τ_pd + τ_com), ctrlrange)`

- PD: hinge `kp*(q_des-q) - kd*qd`; ball — ошибка кватерниона на ось gear мотора.
- Классы коэффициентов по подстроке имени актуатора: ankle, knee, hip, lumbar, thoracic, neck, shoulder, elbow, иначе wrist. Числа зашиты в `DEFAULT_GAINS` (не в JSON). Комментарий класса: «CEM + modest vertical COM».
- COM: `mj_jacSubtreeCom` тела pelvis. Опора = середина `xpos` foot_l и foot_r. Сила xy: `KP_COM=4474.954…`, `KD_COM=133.760…`. Вертикаль: `KP_Z=203.954…`, `KD_Z=12.803…`, vz = `qvel[2]`. В момент добавляется только каналам, в чьём имени есть ankle/knee/hip/lumbar.
- `reset_to_stand`: `mj_resetData`, qpos = qpos0, qvel 0, ctrl 0, `mj_forward`. `sole_lift` по умолчанию **False** (комментарий: тюнинг рассчитан на ~1.6 мм проникновения подошвы).
- Fade: tilt > 1.0 рад или pelvis z < 0.35 м или нет контакта floor∩{foot_l_geom, foot_r_geom} дольше 80 шагов (~0.16 с) → fade −0.05 (пол 0). Иначе fade +0.1 (потолок 1).
- Пороги «стоит» в том же файле, их же использует validate: z ≥ 0.70, tilt ≤ 0.55. Это не лимиты суставов.

Имена, без которых контроллер не резолвится: body `pelvis`, `foot_l`, `foot_r`; geom `floor`, `foot_l_geom`, `foot_r_geom`.

## Маппинг на кости (кратко)

[mapping_full.py](../physics/bridge_full/mapping_full.py): в rest все xquat тел = I (так написано в модуле). Желаемый мир кости: `R_mj @ R_rest_world` из калибровки.

Ведущие: pelvis→pelvis, torso_lower→spine_01, torso_upper→spine_05, head→head, конечности 1:1.

Производные: spine_02 ← torso_lower; spine_03/04 ← torso_upper; neck_01/02 ← slerp(torso_upper, head, 0.50 и 0.77); **clavicle_l/r ← torso_upper** (не плечо).

Не в apply_order как привод: список `UNMAPPED_VISUAL` = ball_l, ball_r, ik_foot_root, ik_hand_root.

Заметка внутри `calibration_rest.json` (`clavicles_welded`: «clavicle_* not driven») **расходится** с кодом, который их ведёт от torso_upper. Код новее этой фразы JSON.

Консоль и observation: `q_three = C * q_blender`, C = Rx(−90°). main.js POC: полное сопряжение `C * q * C^{-1}`. Документ CONTROL_ARCHITECTURE описывает поправку именно для console-viewer.
