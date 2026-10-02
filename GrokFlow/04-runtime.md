# 04. Рантайм, тесты, поставка

Команды ниже — как они записаны в репозитории. Эта база их **не запускала** (в среде снимка нет целевого venv проекта и не поднимался MuJoCo).

## Что должно быть живо одновременно

Режим START (debug / experiment / assisted), один корень проекта:

1. Процесс `scripts/launch.py` — HTTP **127.0.0.1:8788**, Unix-сокет остановки, lock `.runtime/launcher.lock`.
2. Дочерний `physics/control_console.py` — WebSocket **127.0.0.1:8766**.
3. Для experiment и assisted: сокет `/tmp/humanoid-actuator-<uid>.sock` (или `HUMANOID_ACTUATOR_SOCKET`), режим 0600. Пока сокет не слушается, `./humanoid` печатает `ERROR runtime unavailable; command outcome unknown`.
4. Браузер на той же машине. Лаунчер вызывает `webbrowser.open`, если не передан `--no-open`.
5. Терминал лаунчера не закрывать: выход гасит HTTP и физику (`terminate`, по таймауту `kill`; `STOP_FORCE` бьёт сразу `kill`).

Не обязательно, но появляется на диске:

- `reports/control_inventory.json` перезаписывается **до** старта долгоживущего процесса (отдельный короткий прогон `--inventory`).
- `reports/` создаётся, если нет.
- В experiment/assisted лог `reports/actuator_commands.jsonl` открывается на append.

Чего быть не должно рядом:

- второго launch.py на тот же корень (lock);
- чужого слушателя на 8788 или 8766 — лаунчер **не убивает** чужой процесс, выходит с «Port … is busy»;
- второго рантайма на тот же actuator socket: `bind` не делает unlink живого сокета.

Контур B (`run_demo.sh`) — отдельные PID HTTP 8787 и ws_bridge 8765. С контуром A порты не совпадают, модели одна и та же на диске. Два процесса MuJoCo = две независимые симуляции.

## Останов

| Способ | Эффект |
|---|---|
| Ctrl+C в терминале START | SIGINT → stop event → terminate физики, shutdown HTTP и control socket |
| `./START.command stop` или `./stop-project.command` | клиент шлёт `STOP_FORCE\n`, ждёт `STOPPED\n` (таймаут сокета клиента 10 с, сервер ждёт 8 с) |
| Если managed-лаунчера нет | обход `ps`, SIGKILL процессов того же uid, чей cwd = корень и чья команда похожа на START / start-actuator-runtime / start-console.command / scripts/launch.py / physics/control_console.py / physics/ws_bridge.py / `python -m http.server 8788` |
| После принудительного kill | если actuator socket есть, принадлежит uid, и `connect` даёт `ConnectionRefusedError` — файл сокета удаляется. Живой сокет не трогается |

Документ [docs/ACTUATOR_API.md](../docs/ACTUATOR_API.md): после kill **не** удалять сокет вслепую на старте. Код `ActuatorAPI.__aenter__` действительно не делает unlink чужого сокета. `stop-project.command` удаляет только refused socket.

## Переключение режима

README: одновременно один режим; чтобы сменить — остановить и выбрать другой. Код это обеспечивает lock'ом, а не горячей сменой флагов внутри живого процесса. `--assisted-stand` без `--actuator-api` argparse отвергает. В меню debug assist доступен кнопкой без этого флага: процесс консоли всегда создаёт `AssistedStandController`.

## Тесты

Из README и DEBUG:

```sh
.venv/bin/python -m unittest discover -s tests -v
```

DEBUG добавляет `PYTHONDONTWRITEBYTECODE=1`. Каталог `tests/` не является пакетом с `__init__.py` (файла нет). Тесты сами вставляют `physics/` или `scripts/` в `sys.path`.

Отдельно, не входит в discover:

```sh
node tests/test_observation_perf.mjs
```

Нужен Node. `npm test` из package.json **не** запускает эти тесты.

`tests/test_launch.py` специально изолирован (временный корень, симлинки physics/viewer/assets, свободные порты) и по докстрингу файла может идти рядом с живым experiment. Он не покрывает `assisted`.

Скрипты, которые пишут отчёты, а не unittest:

```sh
.venv/bin/python scripts/validate_assisted_stand.py
.venv/bin/python scripts/tune_assisted_stand.py
```

Успех validate в его docstring: непрерывное «upright» (pelvis z ≥ 0.70 м, tilt ≤ 0.55 рад) **≥ 5 с**. Цель ретюна в тексте — ≥ 10 с. Перезапуск в этой сессии не делался; готовый `reports/ASSISTED_STAND_REPORT.md` (заголовок прочитан) пишет PASS и 10.05 с со ссылкой на `reports/assisted_stand/metrics.json`. Сам metrics.json здесь не разбирался.

Пересборка оболочек (меняет OBJ, не часть обычного запуска):

```sh
.venv/bin/python scripts/build_collision_surfaces.py
```

## Поставка `scripts/package_release.py`

Запуск: `.venv/bin/python scripts/package_release.py` (так в README).

Куда: каталог **рядом с проектом**, `HumanoidAlpha_Delivery/` (`ROOT.parent / 'HumanoidAlpha_Delivery'`).

Пишет:

- `HumanoidAlpha.zip` (сначала `.tmp`, `testzip`, затем replace)
- копию `INSTALL_RU.md`
- `SHA256SUMS.txt` — хеш zip
- `README.txt` — три строки по-русски и абсолютный путь исходной папки

Внутри zip префикс `HumanoidAlpha/`. Пропускается:

- каталоги `.venv`, `.runtime`, `__pycache__`, `.git`, `node_modules`, `shots`
- не-файлы и **симлинки** (`humanoid-assist` в архив не попадёт; `scripts/assist_cli.py` — обычный файл, попадёт)
- `.DS_Store`, суффиксы `.pyc`, `.log`, `.jsonl` (журнал команд не уезжает)
- всё под `reports/`, кроме двух путей: `reports/control_inventory.json` и `reports/collision/model_before.xml`

Обязательная проверка содержимого: `START.command`, `INSTALL_RU.md`, `scripts/launch.py`, `physics/collision_guard.py`, `viewer/observation.html`, `viewer/observation.js`, `viewer/console.html`, и ровно **27** имён под `HumanoidAlpha/assets/collision/`.

Скрипт не вызывает unittest.
