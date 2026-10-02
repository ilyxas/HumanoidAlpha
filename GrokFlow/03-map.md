# 03. Карта репозитория

Корень `main` @ `18631fa`. Ниже — зачем файл, кто зовёт, от чего зависит. Вендор `viewer/vendor/three/**` — библиотека r170, продукт её не форкает по смыслу; править только если ломается importmap.

## Корень

| Путь | Роль | Кто вызывает | От чего зависит |
|---|---|---|---|
| [START.command](../START.command) | Меню, venv, pip, `exec` launch.py | пользователь, [start-console.command](../start-console.command), [start-actuator-runtime](../start-actuator-runtime) | Python 3.11–3.13, [requirements.txt](../requirements.txt), [scripts/launch.py](../scripts/launch.py) |
| [stop-project.command](../stop-project.command) | STOP_FORCE в сокет лаунчера, иначе SIGKILL своих процессов с cwd=корень; снимает мёртвый actuator socket | START `stop`, пользователь | [scripts/launch.py](../scripts/launch.py) `stop_managed` |
| [humanoid](../humanoid) | CLI `act` (33 числа) / `zero`. Симуляцию не импортирует | человек, тесты API | Unix socket рантайма |
| [humanoid-assist](../humanoid-assist) | симлинк на `scripts/assist_cli.py` | человек в режиме assisted | тот же socket, ключ JSON `assist` |
| [start-console.command](../start-console.command) | `exec START.command debug` | старые инструкции | START.command |
| [start-actuator-runtime](../start-actuator-runtime) | `exec START.command experiment` | старые инструкции | START.command |
| [package.json](../package.json) / [package-lock.json](../package-lock.json) | puppeteer-core и three для **опционального** npm | capture `*.mjs` после `npm ci` | не нужны браузерному рантайму |
| [requirements.txt](../requirements.txt) | пины venv | START.command | pip |
| [README.md](../README.md) | короткий вход | человек | местами отстаёт от START (нет режима 3) |
| [INSTALL_RU.md](../INSTALL_RU.md) | инструкция тестировщика zip | [scripts/package_release.py](../scripts/package_release.py) копирует рядом с архивом | Python 3.13 в тексте |
| [LICENSE](../LICENSE) | MIT 2026 | — | — |
| [.gitignore](../.gitignore) | игнор `.venv`, `node_modules`, `.runtime/`, `viewer/.bak_*/` | git | `reports/*.jsonl` **не** игнор: журнал команд лежит в git |

## physics/

Запускать `control_console.py` и `ws_bridge.py` как скрипты: `sys.path[0]` становится `physics/`, отсюда голые импорты `actuator_runtime`, `agent_api`, `bridge_full`.

| Путь | Роль | Кто вызывает | Зависит от |
|---|---|---|---|
| [physics/control_console.py](../physics/control_console.py) | Единственный писатель MuJoCo в контуре A | launch.py, тесты control/collisions/actuator | env, diag, mapping_full, collision_guard, assisted_stand, actuator_runtime, XML |
| [physics/ws_bridge.py](../physics/ws_bridge.py) | Сценарный мост контура B | run_demo.sh, README_MAC | env, diag, mapping_full, XML |
| [physics/actuator_runtime.py](../physics/actuator_runtime.py) | Unix API, валидация 33 каналов, лог jsonl | control_console при `--actuator-api` | модель через controller; сокет |
| [physics/assisted_stand.py](../physics/assisted_stand.py) | PD позы + COM Jacobian, клип ctrl | Controller.tick / set_assisted / API `assist` | имена тел pelvis, foot_l/r, геомы foot_*_geom, floor |
| [physics/collision_guard.py](../physics/collision_guard.py) | Проверка swept-пути в Position, без mj_step | Controller.handle `joint` | живая модель |
| [physics/agent_api/env.py](../physics/agent_api/env.py) | `HumanoidEnv`: reset/observe/apply_torques/step/render | оба моста, validate/tune скрипты | `model/humanoid_alpha.xml`, model_meta.json, assumptions.json |
| [physics/agent_api/__init__.py](../physics/agent_api/__init__.py) | экспорт env | импортёры пакета | env |
| [physics/diagnostic_api/diag.py](../physics/diagnostic_api/diag.py) | qpos/qvel, энергии, xpos/xquat, COM pelvis | мосты | env |
| [physics/bridge_full/mapping_full.py](../physics/bridge_full/mapping_full.py) | тело MuJoCo → мировые кватернионы костей | мосты, build_collision_surfaces, тесты GLB | calibration_rest.json |
| [physics/bridge_full/calibration_rest.json](../physics/bridge_full/calibration_rest.json) | `R_rest_world` костей, метод выравнивания | load_calibration | не рантайм-пишется |
| [physics/model/humanoid_alpha.xml](../physics/model/humanoid_alpha.xml) | **рабочая** MJCF | HumanoidEnv по умолчанию | `../../assets/collision/*.obj` |
| [physics/model/humanoid_alpha_pre_anatomical.xml](../physics/model/humanoid_alpha_pre_anatomical.xml) | старая геометрия капсул/боксов, `inertiafromgeom="true"` | в Python-рантайме **не загружается** (упоминание в MANIFEST) | без OBJ |
| [physics/model/actuator_order.json](../physics/model/actuator_order.json) | тот же порядок 33 моторов + class/axis | человеком / инвентарём; env порядок берёт из **загруженной модели**, не из этого файла напрямую | должен совпадать с XML |
| [physics/model_meta.json](../physics/model_meta.json) | nq/nv/nu, имена, joint→bone, total_mass | env читает actuator_order **только как fallback до первого init** | факт модели на момент экспорта метаданных |
| [physics/anatomical_ranges.json](../physics/anatomical_ranges.json) | конусы ball и диапазоны hinge, пометки ENGINEERING | **не импортируется** рантаймом; числа продублированы в XML | исторический CSV-путь не открывается |
| [physics/assumptions.json](../physics/assumptions.json) | массы-допущения 60 кг, оси, лимиты, заметка про коллизии | env читает `torque_limits_Nm`, но лимиты шага берутся из `actuator_ctrlrange` модели | не равен инерциям XML |
| [physics/__init__.py](../physics/__init__.py) | однострочный маркер пакета | — | — |

`apply_full_blender.py` упомянут в docstring `extract_full_pose` как потребитель схемы. **Файла в дереве нет.**

## viewer/ (свой код, не vendor)

| Путь | Роль | Кто |
|---|---|---|
| [viewer/console.html](../viewer/console.html) | Разметка Debug, вкладка Assisted | редирект launch в режиме debug; грузит console.js и console-viewer.js |
| [viewer/console.js](../viewer/console.js) | Слайдеры, режимы, ошибки контакта, ссылка на `reports/control_inventory.json` | страница консоли |
| [viewer/console-viewer.js](../viewer/console-viewer.js) | WebGL консоли, WS 8766 или `?wsPort=`, формула `QC*q` | страница консоли |
| [viewer/console.css](../viewer/console.css) | стили консоли | console.html |
| [viewer/observation.html](../viewer/observation.html) | Экран Experiment/Assisted | редирект launch |
| [viewer/observation.js](../viewer/observation.js) | Камеры, пол, GLB, WS, поза; профилировщик включён всегда (`createProfiler(true)`) | observation.html |
| [viewer/observation.css](../viewer/observation.css) | стили | observation.html |
| [viewer/observation-perf.js](../viewer/observation-perf.js) | таймеры CPU, лог `[observation-perf]` раз в ~5 с если enabled | observation.js |
| [viewer/index.html](../viewer/index.html) + [viewer/main.js](../viewer/main.js) | POC: manual sliders + Physics WS **:8765**, формула с сопряжением | run_demo.sh, README_MAC |

## scripts/

| Путь | Роль |
|---|---|
| [scripts/launch.py](../scripts/launch.py) | супервизор контура A |
| [scripts/package_release.py](../scripts/package_release.py) | zip в `../HumanoidAlpha_Delivery` |
| [scripts/assist_cli.py](../scripts/assist_cli.py) | CLI assist |
| [scripts/run_demo.sh](../scripts/run_demo.sh) | контур B |
| [scripts/build_collision_surfaces.py](../scripts/build_collision_surfaces.py) | пересборка OBJ из Xandra.glb |
| [scripts/validate_assisted_stand.py](../scripts/validate_assisted_stand.py) | headless PASS: удержание ≥ 5 с, пишет `reports/assisted_stand/` |
| [scripts/tune_assisted_stand.py](../scripts/tune_assisted_stand.py) | CEM вокруг текущих коэффициентов, горизонт 15 с |
| [scripts/01_audit_and_export.py](../scripts/01_audit_and_export.py) | Blender (`bpy`): аудит и экспорт GLB. Blend в репо нет |
| [scripts/02_inspect_glb.py](../scripts/02_inspect_glb.py) | разбор GLB без MuJoCo |
| `scripts/03_capture_manual.mjs` … `07_capture_fall_early.mjs`, `shot_obs.mjs` | puppeteer на порт **8787** |

## tests/

`unittest` видит только `test_*.py`. JS-файл отдельно.

| Файл | Что проверяет |
|---|---|
| [tests/test_control.py](../tests/test_control.py) | инвентарь, суставы, режимы, лимиты момента, пауза/reset |
| [tests/test_collisions.py](../tests/test_collisions.py) | сохранность динамики, блок swept-пути, пол, контакт |
| [tests/test_actuator_api.py](../tests/test_actuator_api.py) | вектор 33, CLI, интеграция сокета |
| [tests/test_glb_frames.py](../tests/test_glb_frames.py) | якоря костей vs GLB |
| [tests/test_launch.py](../tests/test_launch.py) | debug и experiment: редирект, ownership, stop. **Режима assisted нет** |
| [tests/test_observation_perf.mjs](../tests/test_observation_perf.mjs) | тайминги профилировщика (`node …`) |

Тестов `assisted_stand` в `tests/` **нет** (поиск `assisted` по каталогу tests пустой). Проверка слоя — скрипт validate, не unittest.

## docs/ и reports/

`docs/` — накопленные заметки разных дат (упаковка 2026-09-29 и более поздние консоль/коллизии/observation/assisted). Это не автоматически текущий контракт. Расхождения: [06-open-questions.md](06-open-questions.md).

`reports/` — свидетельства прогонов. Рантайм **пишет** `reports/control_inventory.json` при каждом launch и **дописывает** `reports/actuator_commands.jsonl` в experiment/assisted. Остальное — архив. Крупные бинарники не читались (см. SOURCES).

В дереве **нет** (документы на них ссылаются): `shots/`, `docs/CONTROL_VALIDATION.md` (DEBUG.md пишет `reports/CONTROL_VALIDATION.md`), `reports/observation/`, `reports/blender_export_stdout.txt`, `HumanoidAlpha_v0.2_SHA256.txt`.

## Бинарники, которые не читались целиком

Роль по XML/скриптам/дереву git. Размеры — blob size на `18631fa`.

| Путь | Байт | Роль |
|---|---|---|
| [assets/Xandra.glb](../assets/Xandra.glb) | 49607732 | скин + арматура, грузят все три viewer-скрипта и `build_collision_surfaces.py` |
| [assets/Xandra_nude.glb](../assets/Xandra_nude.glb) | 49340004 | лежит в git; **ссылок в py/js/md/xml/sh/mjs не найдено** |
| `assets/collision/*.obj` | 27 файлов, см. SOURCES | входы convex hull (`maxhullvert="64"`) |
| [reports/actuator_commands.jsonl](../reports/actuator_commands.jsonl) | 479448 | журнал API, не исходник |
| `reports/assisted_stand/*.png` | plot 90170, stand_t0 29712, stand_t5 41101, stand_t10 48340 | картинки validate; не читались |
| [reports/assisted_stand/timeseries.csv](../reports/assisted_stand/timeseries.csv) | 123581 | ряд validate; не читался целиком |
| [reports/assisted_stand/metrics.json](../reports/assisted_stand/metrics.json) | 6654 | не читался; отчёт `reports/ASSISTED_STAND_REPORT.md` утверждает PASS 10.05 с |
| [viewer/vendor/three/three.module.js](../viewer/vendor/three/three.module.js) | 1314681 | библиотека; прочитана только строка REVISION |

OBJ: surface — оболочка звена; `*_distal` — mesh `*_joint_guard` (в XML `contype="0" conaffinity="0" group="3"`) и одновременно geom в явных `<pair>` с родителем. Голова — сфера `head_geom`, не OBJ.
