# SOURCES

Снимок `main` `18631fa64be8a0213356893b80c8c29af5d89a37` (tree `d6babfbfb8a4ad5b025a2ac8de550f8d822eec4d`), 2026-10-02.

«Прочитан» = текст файла открыт в этой сессии. «Заголовок» = первые строки через `git show`. «Дерево» = путь и размер blob, тело не читалось. Вендор Three (кроме строки REVISION) не конспектировался: это библиотека r170.

## Корень

| Файл | Итог |
|---|---|
| README.md | Прочитан. Вход START.command, режимы 1/2/0, порт 8788, unittest, package_release. Режима assisted нет. |
| INSTALL_RU.md | Прочитан. Python 3.13, experiment, адрес 8788, критерий OK тестов и `./humanoid zero`. |
| START.command | Прочитан. Меню 1 debug / 2 experiment / 3 assisted / 0 stop; Python 3.11–3.13; pip если нет mujoco/numpy/websockets. |
| humanoid | Прочитан. `act` из 33 конечных чисел или `zero`; Unix JSON `{"values":...}`; stdout только OK. |
| humanoid-assist | Дерево: symlink 21 байт на `scripts/assist_cli.py`. |
| start-actuator-runtime | Прочитан. `exec START.command experiment`. |
| start-console.command | Прочитан. `exec START.command debug`. |
| stop-project.command | Прочитан. STOP_FORCE, иначе SIGKILL своих процессов, снятие refused actuator socket. |
| package.json | Прочитан. three 0.170.0, puppeteer-core ^23.11.1, `npm test` — заглушка. |
| package-lock.json | Фрагмент. lock three 0.170.0, puppeteer-core 23.11.1, Node >=18 только у @puppeteer/browsers. |
| requirements.txt | Прочитан. mujoco 3.14.0, numpy 2.5.3, websockets 17.1 + четыре transitive пина. |
| LICENSE | Начало. MIT, copyright 2026 ilyxas. |
| .gitignore | Фрагмент хвоста. Игнор `.venv`, `node_modules`, `.runtime/`. |

## docs/

| Файл | Итог |
|---|---|
| docs/ACTUATOR_API.md | Прочитан. Контракт 33 каналов, сокет, jsonl, «не clamp»; старт через start-actuator-runtime и console.html — расходится с текущим experiment→observation. |
| docs/ASSISTED_STAND.md | Прочитан. Как включить в debug и `./START.command assisted`; формула u_cmd+PD/COM. |
| docs/CHANGES_FROM_LIVE.md | Прочитан. Портативность 2026-09-29, что не копировали, какие пути относительные. |
| docs/COLLISIONS.md | Прочитан. Convex hull, margin 6 мм, guard, 27 OBJ, лимиты приближения. |
| docs/CONTROL_ARCHITECTURE.md | Прочитан. Очередь, порты 8788/8766, формула кватерниона консоли; фразы про ключицы и «нет баланса» устарели. |
| docs/DEBUG.md | Прочитан. UX консоли, инвентарь 20/33/47/39; ссылка на отсутствующий CONTROL_VALIDATION.md. |
| docs/HumanoidAlpha_v0.2_MANIFEST.md | Прочитан. Старый состав portable, порты 8787/8765, GLB ~81MB. |
| docs/OBSERVATION_VIEW.md | Прочитан. Камеры, пол, perf по флагу `?profile=1` — флаг в коде сейчас зашит в true. |
| docs/README_MAC.md | Прочитан. Mac-инструкция контура B; macOS не тестировался при упаковке. |
| docs/RESTORE_TEST_REPORT.md | Прочитан. Linux 2026-09-29, GLB 84113904 байт, bones=23 на том прогоне. |

## physics/

| Файл | Итог |
|---|---|
| physics/__init__.py | Прочитан. Одна строка-маркер. |
| physics/ws_bridge.py | Прочитан. Сценарии rest/knee/shoulder/fall/hold/posed, порт 8765, broadcast, dt assert 0.002. |
| physics/actuator_runtime.py | Прочитан. Валидация 33, сокет 0600 без unlink, assist ops, лог jsonl, apply только из очереди. |
| physics/assisted_stand.py | Прочитан. PD, COM, fade, пороги 0.70/0.55, коэффициенты в коде. |
| physics/collision_guard.py | Прочитан. Sweep, допуски 2 мм / 6 мм, без шага времени. |
| physics/control_console.py | Прочитан целиком. Controller, очередь 128, 30 Гц, флаги CLI. |
| physics/agent_api/__init__.py | Прочитан. Экспорт HumanoidEnv. |
| physics/agent_api/env.py | Прочитан. reset/observe/apply_torques clip, DEFAULT_MODEL, render через PIL или npy. |
| physics/diagnostic_api/__init__.py | Прочитан. |
| physics/diagnostic_api/diag.py | Прочитан. qpos, энергии, xpos/xquat, subtree COM pelvis. |
| physics/bridge_full/__init__.py | Прочитан. Реэкспорт маппера. |
| physics/bridge_full/mapping_full.py | Прочитан BODY_TO_BONE, производные кости, extract_full_pose. Тело quat-утилит между строками 122–247 не пересказывалось построчно. |
| physics/bridge_full/calibration_rest.json | Начало (~40 строк) + ключи method/notes. Полный набор матриц R_rest не выписывался. |
| physics/model/humanoid_alpha.xml | Прочитан целиком. Суставы, инерции, пары контакта, 33 мотора. |
| physics/model/humanoid_alpha_pre_anatomical.xml | Начало + типы geom (капсулы/боксы). Не рантайм. |
| physics/model/actuator_order.json | Начало. Тот же порядок, поля class/axis. |
| physics/model_meta.json | Прочитан. Размерности, имена, joint_to_visual_bone, total_mass, исторический абсолютный путь. |
| physics/anatomical_ranges.json | Механизм, все ball_cones, все hinge_ranges, хвост с joints_dof_csv. |
| physics/assumptions.json | Прочитан. Масса 60 кг, лимиты, оси, welds, заметка collision_geometry. |

## viewer/ (свой код)

| Файл | Итог |
|---|---|
| viewer/index.html | Прочитан. POC UI, слайдеры knee/shoulder, importmap. |
| viewer/main.js | Начало (формула QC*q*QC⁻¹) и кусок WS :8765 + загрузка GLB. Середина ручного режима не конспектировалась. |
| viewer/console.html | Прочитан. Вкладки Position/Torque/Assisted. |
| viewer/console.js | Начало (~40 строк): send, inventory, сброс слайдера после error. Дальше не построчно. |
| viewer/console-viewer.js | Начало (формула QC*q) и grep порта 8766. |
| viewer/observation.html | Прочитан. Кнопки камер, лишний `</div>`, canvas `#can`. |
| viewer/observation.js | Прочитан ключевой путь: пол, пресеты, `createProfiler(true)`, zupQuatToYup, WS, GLB. |
| viewer/observation-perf.js | Прочитан. Лог раз в 5 с, если enabled. |
| viewer/observation.css | Не конспектировался. |
| viewer/console.css | Не конспектировался. |
| viewer/vendor/three/three.module.js | Только `REVISION = '170'`. Остальное дерево vendor — размеры blob, не чтение. |

## scripts/ и tests/

| Файл | Итог |
|---|---|
| scripts/launch.py | Прочитан. Редирект, порты, флаги режимов, stop. |
| scripts/package_release.py | Прочитан. Фильтр zip, 27 collision, SHA-256. |
| scripts/assist_cli.py | Прочитан. on/off/reset/status. |
| scripts/run_demo.sh | Прочитан. HTTP 8787 + ws_bridge. |
| scripts/build_collision_surfaces.py | Docstring и начало разбора GLB. |
| scripts/validate_assisted_stand.py | Docstring и пороги PASS 5 с / z 0.70 / tilt 0.55. |
| scripts/tune_assisted_stand.py | Docstring: CEM, горизонт 15 с. |
| scripts/01_audit_and_export.py | Начало: bpy, экспорт GLB. |
| scripts/02_inspect_glb.py | Сигнатура и grep clavicle. |
| scripts/03_capture_manual.mjs | Начало: puppeteer, Chrome, URL 8787. |
| scripts/04–07_*.mjs, shot_obs.mjs | Не читались построчно; по дереву это capture-скрипты. |
| tests/test_*.py | Имена методов через поиск `def test_`. Тела не пересказывались, кроме test_launch (прочитан цикл debug/experiment). |
| tests/test_observation_perf.mjs | Не читался. Запуск отдельно от unittest — из OBSERVATION_VIEW.md. |

## reports/ (не менялись; бинарники не читались)

| Файл | Итог |
|---|---|
| reports/ASSISTED_STAND_REPORT.md | Заголовок ~25 строк: дата 2026-10-02, PASS, best upright 10.05 с. |
| reports/ASSISTED_STAND_INVESTIGATION.md | Заголовок: «investigate only», масса ≈89.044 кг, nq/nv/njnt/nu. Тело дальше не читалось. |
| reports/assisted_stand/metrics.json | Дерево, 6654 байт. Не читался. |
| reports/assisted_stand/tune_*.json | Дерево. Не читались. |
| reports/assisted_stand/timeseries.csv | Дерево, 123581 байт. Не читался. |
| reports/assisted_stand/*.png | Дерево: plot.png 90170, stand_t0.png 29712, stand_t5.png 41101, stand_t10.png 48340. Картинки не открывались. |
| reports/actuator_commands.jsonl | Дерево, 479448 байт. Журнал, не читался. |
| reports/control_inventory.json | Дерево, 29339 байт. Генерируется лаунчером; в сессии не читался. |
| reports/control_joint_sweep.json | Дерево, 7357. Не читался. |
| reports/glb_frame_validation.json | Дерево, 3149. Не читался. |
| reports/collision/model_before.xml | Дерево, 14719. Не читался. Упомянут как заморозка в COLLISIONS.md. |
| reports/collision/controlled_motor_contact.json | Дерево, 121. Не читался. |
| reports/collision/visual_anchor_validation.json | Дерево, 19790. Не читался. |

## assets/ (не читались)

| Файл | Байт | Роль по ссылкам |
|---|---|---|
| assets/Xandra.glb | 49607732 | GLB сцены и вход build_collision_surfaces |
| assets/Xandra_nude.glb | 49340004 | ссылок в исходниках не найдено |
| assets/collision/calf_l_distal.obj | 9759 | joint guard |
| assets/collision/calf_l_surface.obj | 14301 | оболочка |
| assets/collision/calf_r_distal.obj | 9961 | joint guard |
| assets/collision/calf_r_surface.obj | 14503 | оболочка |
| assets/collision/foot_l_distal.obj | 34662 | joint guard |
| assets/collision/foot_l_surface.obj | 39085 | оболочка стопы |
| assets/collision/foot_r_distal.obj | 35719 | joint guard |
| assets/collision/foot_r_surface.obj | 40366 | оболочка стопы |
| assets/collision/hand_l_distal.obj | 51125 | joint guard |
| assets/collision/hand_l_surface.obj | 58286 | оболочка |
| assets/collision/hand_r_distal.obj | 50206 | joint guard |
| assets/collision/hand_r_surface.obj | 57304 | оболочка |
| assets/collision/lowerarm_l_distal.obj | 8785 | joint guard |
| assets/collision/lowerarm_l_surface.obj | 11771 | оболочка |
| assets/collision/lowerarm_r_distal.obj | 9340 | joint guard |
| assets/collision/lowerarm_r_surface.obj | 12518 | оболочка |
| assets/collision/pelvis_surface.obj | 166032 | оболочка таза (distal-пары нет) |
| assets/collision/thigh_l_distal.obj | 8300 | joint guard |
| assets/collision/thigh_l_surface.obj | 13098 | оболочка |
| assets/collision/thigh_r_distal.obj | 7865 | joint guard |
| assets/collision/thigh_r_surface.obj | 12817 | оболочка |
| assets/collision/torso_lower_surface.obj | 7345 | оболочка |
| assets/collision/torso_upper_surface.obj | 32663 | оболочка |
| assets/collision/upperarm_l_distal.obj | 7764 | joint guard |
| assets/collision/upperarm_l_surface.obj | 11247 | оболочка |
| assets/collision/upperarm_r_distal.obj | 7433 | joint guard |
| assets/collision/upperarm_r_surface.obj | 10893 | оболочка |

27 OBJ. Голова в этот список не входит: сфера в XML.
