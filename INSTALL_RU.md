# Установка и запуск

Нужны Python 3.13, unzip и браузер с WebGL.

## 1. Распаковать

```sh
unzip HumanoidAlpha.zip
cd HumanoidAlpha
chmod +x START.command stop-project.command humanoid
```

## 2. Запустить

```sh
./START.command experiment
```

Дождись установки зависимостей. Оставь терминал открытым.
Откроется браузер. Адрес: **http://127.0.0.1:8788/**

В окне: модель, белый пол, часы, кнопки FRONT / LEFT / BACK / RIGHT / TOP / RESET,
LOCK / UNLOCK и Screenshot. RESET меняет только камеру.

## 3. Проверить во втором терминале

Перейди в ту же распакованную папку `HumanoidAlpha`:

```sh
.venv/bin/python -m unittest discover -s tests -v
./humanoid zero
```

Проверка пройдена, если тесты завершаются `OK`, команда возвращает `OK`,
модель видна, часы обновляются и кнопки камеры работают.

## 4. Остановить

```sh
./START.command stop
```
