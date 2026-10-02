# 00. Обзор

Humanoid Alpha — локальный браузерный рантайм скиннированной модели **Xandra** поверх физики **MuJoCo**. Физика — источник истины позы. Three.js только рисует GLB. Blender в цикле запуска не участвует (`docs/README_MAC.md`, `docs/CHANGES_FROM_LIVE.md`). Лицензия: MIT, copyright 2026 ilyxas ([LICENSE](../LICENSE)).

Имя пакета в [package.json](../package.json): `humanoid_alpha_v0.2`, version `1.0.0`. Это не semver продукта в README: README говорит «Humanoid Alpha» без номера версии в заголовке.

## Одна точка входа

[START.command](../START.command) — единственный штатный запуск (так написано в [README.md](../README.md)).

Интерактивное меню в скрипте (не в README):

| Ввод | Аргумент | Что делает |
|---|---|---|
| 1 | `debug` | Консоль разработчика |
| 2 | `experiment` | Экран наблюдения + RAW CLI `./humanoid` |
| 3 | `assisted` | То же, плюс авто-включение assisted stand |
| 0 | `stop` | `./stop-project.command` |

Прямой вызов: `./START.command debug|experiment|assisted|stop`.

README описывает только пункты 1 / 2 / 0 и **не упоминает режим 3 / `assisted`**. Расхождение зафиксировано в [06-open-questions.md](06-open-questions.md).

Адрес обоих (и третьего) режимов, который печатает [scripts/launch.py](../scripts/launch.py): **http://127.0.0.1:8788/**. Корень HTTP редиректит на viewer с `?wsPort=`.

## Режимы, как они есть в коде

[scripts/launch.py](../scripts/launch.py) выбирает страницу:

- `debug` → `/viewer/console.html?wsPort=<ws>`
- `experiment` и `assisted` → `/viewer/observation.html?wsPort=<ws>`

Порты по умолчанию: HTTP **8788**, WebSocket **8766**.

Одновременно один управляемый экземпляр на корень проекта: lock `.runtime/launcher.lock` и Unix-сокет `/tmp/humanoid-launcher-<uid>-<sha256(root)[:16]>.sock`. Второй запуск падает с текстом «Project already running».

Терминал запуска должен жить. Ctrl+C в нём ставит stop и гасит физику и HTTP.

## Python

[START.command](../START.command) принимает интерпретатор **3.11–3.13** (проверка `sys.version_info`). Если `.venv` нет или версия вне диапазона, ищет `python3.13`, `python3.12`, `python3.11`, `python3` (или `$PYTHON`) и делает `python -m venv .venv`. Если `mujoco`, `numpy`, `websockets` не импортируются — `pip install -r requirements.txt`.

Текст ошибки при отсутствии подходящего Python: `Install Python 3.13, then run START.command again.` — хотя диапазон шире. [INSTALL_RU.md](../INSTALL_RU.md) требует именно Python 3.13, unzip и браузер с WebGL.

`.venv` в git не входит ([.gitignore](../.gitignore)).

## Что открывается пользователю

- **Debug:** [viewer/console.html](../viewer/console.html) — вкладки Position / Torque / Assisted, инвентарь суставов, диагностика. Первый клиент WebSocket владеет симуляцией.
- **Experiment:** [viewer/observation.html](../viewer/observation.html) — сцена, камеры FRONT/LEFT/BACK/RIGHT/TOP/RESET, LOCK, часы, Screenshot. Команды в сокет не шлёт. Моторы: `./humanoid act <33 числа>` или `./humanoid zero`.
- **Assisted:** та же observation-страница; физика стартует с `--actuator-api --assisted-stand`. CLI слоя: симлинк [humanoid-assist](../humanoid-assist) → [scripts/assist_cli.py](../scripts/assist_cli.py) (`on|off|reset|status`).

Старые обёртки сохранены и только переадресуют:

- [start-console.command](../start-console.command) → `START.command debug`
- [start-actuator-runtime](../start-actuator-runtime) → `START.command experiment`

## Старый демо-контур (не START.command)

[scripts/run_demo.sh](../scripts/run_demo.sh) поднимает `python -m http.server` на **8787** и [physics/ws_bridge.py](../physics/ws_bridge.py) на **8765**. Страница: http://127.0.0.1:8787/viewer/index.html ([viewer/main.js](../viewer/main.js) жёстко стучится на порт 8765). Это отдельный сценарий POC (rest/knee/shoulder/fall/hold/posed), не меню START.
