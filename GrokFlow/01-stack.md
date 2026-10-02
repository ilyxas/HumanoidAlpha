# 01. Стек

Факты из [package.json](../package.json), [package-lock.json](../package-lock.json), [requirements.txt](../requirements.txt), импортов и vendored Three.

## Python — физика, запуск, CLI актуаторов

Прямые импорты кода (не комментарии):

| Библиотека | Где | Версия в requirements.txt |
|---|---|---|
| `mujoco` | `physics/*`, часть `scripts/`, тесты | `==3.14.0` |
| `numpy` | те же | `==2.5.3` |
| `websockets` | `physics/ws_bridge.py`, `physics/control_console.py`, `scripts/launch.py` (`websockets.sync.client`) | `==17.1` |

Стандартная библиотека без сторонних пакетов: [humanoid](../humanoid), [scripts/assist_cli.py](../scripts/assist_cli.py), [physics/actuator_runtime.py](../physics/actuator_runtime.py) (сокет + JSON; MuJoCo трогает только вызывающий цикл).

Закреплено в requirements как «transitive / commonly co-installed with mujoco», **прямых import в репозитории не найдено**:

- `absl-py==2.5.0`
- `etils==1.14.0`
- `glfw==2.10.2`
- `PyOpenGL==3.1.10`

`PIL` / Pillow: опциональный импорт в `HumanoidEnv.render_observation` ([physics/agent_api/env.py](../physics/agent_api/env.py)). В `requirements.txt` нет. Нет Pillow — пишется `.npy`, не PNG.

`bpy` / `mathutils`: только [scripts/01_audit_and_export.py](../scripts/01_audit_and_export.py). Это Blender, не рантайм. `.blend` в дереве `main` нет (исключён ещё при упаковке, см. [docs/CHANGES_FROM_LIVE.md](../docs/CHANGES_FROM_LIVE.md)).

Комментарий в requirements: пины сняты с `/workspace/blender/body_lab/knee_physics/.venv`, Python 3.13.5, Linux. Это историческая пометка файла, не текущий путь рантайма.

## JavaScript — только просмотр и съёмка

Рантайм браузера **не требует** `node_modules`. Three vendored:

- [viewer/vendor/three/three.module.js](../viewer/vendor/three/three.module.js) — `REVISION = '170'`
- importmap в `viewer/index.html`, `viewer/console.html`, `viewer/observation.html` указывает на `./vendor/three/...`

[package.json](../package.json):

- `three`: точный `0.170.0` (lock: `0.170.0`)
- `puppeteer-core`: `^23.11.1`, в lock разрешён **23.11.1**
- `engines` для Node в package.json **нет**. Lock у `@puppeteer/browsers` пишет `"node": ">=18"`. Документ [docs/README_MAC.md](../docs/README_MAC.md) говорит «Node.js 18+ только если нужны puppeteer capture scripts».
- script `npm test` — заглушка: печатает ошибку и `exit 1`. Юнит-тесты продукта — Python `unittest`, плюс один `node tests/test_observation_perf.mjs`.

Скрипты `scripts/03_…mjs`, `04_`, `05_`, `06_`, `07_`, `shot_obs.mjs` импортируют `puppeteer-core` и ходят на **8787** (старый HTTP), Chrome path в прочитанном `03_capture_manual.mjs`: `/usr/bin/google-chrome-stable`.

## Кто что делает

| Слой | Процесс |
|---|---|
| Меню, venv, pip | shell [START.command](../START.command) |
| HTTP статики, редирект `/`, надзор, открытие браузера | Python [scripts/launch.py](../scripts/launch.py) |
| MuJoCo, очередь команд, WebSocket 8766, Unix-сокет актуаторов | Python [physics/control_console.py](../physics/control_console.py) |
| Сценарии fall/knee/… и WebSocket 8765 | Python [physics/ws_bridge.py](../physics/ws_bridge.py) |
| Команда `act`/`zero` | Python stdlib [humanoid](../humanoid) → Unix socket |
| Команда assist | Python stdlib [scripts/assist_cli.py](../scripts/assist_cli.py) |
| Кожа, камера, скелет | браузер, vendored three r170 |
| Headless-скриншоты POC | Node + puppeteer-core, опционально |

Blender-ассеты: визуал — `assets/Xandra.glb` (и рядом неподключённый `assets/Xandra_nude.glb`). Коллизии — OBJ в `assets/collision/`, собираются [scripts/build_collision_surfaces.py](../scripts/build_collision_surfaces.py) из кожи GLB, без Blender в этом скрипте (numpy + MuJoCo).
