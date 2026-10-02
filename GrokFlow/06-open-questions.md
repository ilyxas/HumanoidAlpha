# 06. Открытые вопросы и хрупкие места

Ничего из этого в коде не «исправлялось». Это список, что проверить **до** правки.

## Документы отстают от START.command

1. [README.md](../README.md) меню: только Debug / Experiment / Stop. [START.command](../START.command) имеет третий пункт `assisted`.
2. [INSTALL_RU.md](../INSTALL_RU.md) требует Python 3.13. Скрипт пускает 3.11–3.13, но текст ошибки говорит «Install Python 3.13».
3. [docs/ACTUATOR_API.md](../docs/ACTUATOR_API.md) велит открыть `viewer/console.html` после `./start-actuator-runtime`. Сейчас этот файл делает `exec START.command experiment`, а лаунчер редиректит на **observation.html**. Консоль по прямому URL жива, но в режиме actuator-api она read-only. Документ также говорит «no balancing»; при `--assisted-stand` баланс есть.
4. [docs/CONTROL_ARCHITECTURE.md](../docs/CONTROL_ARCHITECTURE.md): «Clavicles use the existing 40% slerp toward upper arms» и «No balance controller». В [mapping_full.py](../physics/bridge_full/mapping_full.py) ключицы = `torso_upper`. Баланс — [assisted_stand.py](../physics/assisted_stand.py). Шея по-прежнему slerp 0.50 / 0.77, не 40%.
5. [docs/OBSERVATION_VIEW.md](../docs/OBSERVATION_VIEW.md): профилирование выключено, пока нет `?profile=1`. [viewer/observation.js](../viewer/observation.js) строка с `createProfiler(params.get('profile') === '1')` закомментирована; вызывается `createProfiler(true)`. Лог `[observation-perf]` идёт всегда, с оверхедом.
6. [docs/HumanoidAlpha_v0.2_MANIFEST.md](../docs/HumanoidAlpha_v0.2_MANIFEST.md) и [docs/RESTORE_TEST_REPORT.md](../docs/RESTORE_TEST_REPORT.md): GLB «~81MB» / Content-Length **84113904**. Blob `assets/Xandra.glb` на этом коммите — **49607732** байт. Отчёт restore от 2026-09-29, другой снимок. Порты в MANIFEST и README_MAC — **8787/8765**, это контур B, не START.
7. [docs/DEBUG.md](../docs/DEBUG.md) ссылается на `reports/CONTROL_VALIDATION.md`. Файла в дереве нет. Нет и `shots/`, `reports/observation/`, `reports/blender_export_stdout.txt`, упомянутых в старых доках.
8. [calibration_rest.json](../physics/bridge_full/calibration_rest.json) note: ключицы «not driven». Код их пишет.
9. [reports/ASSISTED_STAND_INVESTIGATION.md](../reports/ASSISTED_STAND_INVESTIGATION.md) (прочитан только заголовок): scope «no controller implementation». В том же коммите контроллер есть. Отчёт нельзя читать как описание текущего кода без сверки даты и тела файла целиком (тело не читалось).
10. Docstring `extract_full_pose` ссылается на `bridge_full/apply_full_blender.py`. Файла нет.

## Две физические «правды» о массе

- XML / model_meta: ~**89.04** кг (явные inertial).
- assumptions.json: **60** кг и другие `masses_kg`. Поле `radii_m` само помечено в `collision_geometry.legacy_radii_note` как старые примитивы. Не подставлять 60 кг в расчёты момента, не переписывая XML.

`total_mass_kg` в meta и сумма inertial отличаются в 12-м знаке. Для порядка величины совпадают; побитово — нет.

## Поведение, о которое легко споткнуться

- **Observation HTML:** в [viewer/observation.html](../viewer/observation.html) лишний `</div>` и canvas `#can`, который `observation.js` не читает. Браузер, скорее всего, простит; валидатор — нет.
- **Пол не белый фон.** README говорит «белый пол». Материал пола в observation.js — `#f3f2ed`, фон сцены `#66717e`. Физический пол в XML — серо-синий rgba, не визуальный пол Three.
- **Профилировщик всегда on** — см. выше. Любая правка observation.js должна решить, баг это или намеренный временный флаг.
- **Симлинк поставки.** `package_release.py` не кладёт симлинки в zip. `./humanoid-assist` в архиве не появится. CLI останется как `scripts/assist_cli.py`.
- **Тесты не знают assisted.** `tests/` не содержит слова assisted. `test_launch.py` гоняет только debug и experiment. Регрессию assist ловит только ручной прогон `scripts/validate_assisted_stand.py` (здесь не запускался).
- **Два сокета и два порта.** Правка «дефолтного порта» в main.js (8765) не меняет консоль (8766) и наоборот.
- **stop не ищет http.server 8787.** Демо `run_demo.sh` этим скриптом может не убиваться, если cmdline не совпал с `ws_bridge.py`.
- **Чужой порт не убивается.** Занятый 8788/8766 — ошибка, не takeover.
- **Stale actuator socket.** Старт API не удаляет существующий sock. Нужен stop или ручное удаление после проверки, что никто не слушает ([docs/ACTUATOR_API.md](../docs/ACTUATOR_API.md) это описывает верно).
- **Очередь 128 и 16 KiB.** Лишний клиент или большой JSON — `error`, не расширение буфера.
- **Nonfinite → reset+pause.** Аварийный путь в `tick`, не отдельный сторож.
- **Position vs dynamic.** Команда не того режима — ошибка, состояние не наполовину применено (guard и присваивание qpos после validate). Не двигать оба пути одним патчем без обоих тестов.
- **Clip только у assist и у `apply_torques`.** RAW CLI вне диапазона отвергается целиком. Смешивать «подрежем» и «откажем» нельзя незаметно.
- **`sole_lift=False`.** Поднятие подошвы на 1.6 мм есть в коде, но выключено, потому что коэффициенты сняты с qpos0. Включать без нового validate — ломать тюнинг, о котором говорит сам комментарий.
- **Имена geom.** Assisted ищет `foot_l_geom` / `foot_r_geom` / `floor`. Переименование в XML молча ломает контактный счётчик (mj_name2id вернёт -1; что будет с контактом — не проверено в этой сессии).
- **Xandra_nude.glb** (~47.1 МиБ) ничем не подключён. Не считать его активным скином.
- **Исторические абсолютные пути** `/workspace/blender/...` в model_meta, anatomical_ranges, CHANGES_FROM_LIVE, старых reports. Рантайм их не открывает. Не «чинить» их на новую машину, не проверив, что ни один скрипт аудита их не ждёт. `01_audit_and_export.py` всё ещё про blend, которого нет.
- **PIL нет в requirements.** `render_observation` без Pillow пишет npy. Неизвестно, зовёт ли текущий UI этот метод. Прямых вызовов из control_console/ws_bridge не видно.
- **macOS.** README_MAC и RESTORE: упаковка проверялась на Linux, macOS — NOT TESTED. START.command написан как POSIX sh и использует `fcntl` в launch.py (есть и на macOS). Фактический прогон на Apple Silicon в этом дереве не задокументирован как пройденный.
- **npm test врёт.** Падает заглушкой. Не использовать как gate.
- **Журнал команд в git.** `reports/actuator_commands.jsonl` ~479 КиБ уже в истории. Новые прогоны допишут файл в рабочую копию. В zip он не входит (суффикс `.jsonl`).

## Что неизвестно (не выдумывать)

- Содержимое GLB (число костей, меши) на **этом** blob 49 607 732 байт: файл не разбирался. RESTORE_TEST_REPORT для старого GLB писал bones=23; mapping_full.apply_order перечисляет 23 имени, но это не сверка с текущим бинарником.
- PASS/FAIL metrics.json побайтно: не читался. Цифра 10.05 с — утверждение `reports/ASSISTED_STAND_REPORT.md`, не повторный прогон.
- Покрытие `test_actuator_api` интеграционным сокетом на этой машине: документ ACTUATOR_API сам пишет, что в одном sandbox bind localhost не прошёл. Здесь тесты не запускались.
- Есть ли в `viewer/vendor` расхождение с npm-пакетом three 0.170.0 побайтно: не сравнивалось, REVISION в файле равен `170`.
- Назначение canvas `#can` и блока `#stats` в observation.html: в JS есть `statsEl`, стиль неизвестен без разбора CSS целиком (CSS открывался только как факт существования файла, содержимое observation.css не конспектировалось).
