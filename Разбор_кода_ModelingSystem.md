# Разбор кода ModelingSystem-1.1

Разбор исходного проекта `C:\АО ИТТ\ModelingSystem-1.1` (форк AirSimDatasetCreator, v1.1). Ссылки на файлы и строки.

---

## Состав проекта

| Файл | Назначение |
|---|---|
| `test.py` | Точка входа: подключение, запуск записи, выполнение одной траектории |
| `dataset_manager.py` | Режимы online/offline + Flask API (порт 5000) |
| `api_server.py` | Сервер заданий: offline-задания и online-пошаговая сессия (порт 8080) |
| `src/AirSimDatasetCreator/AirSimDroneConnector.py` | Обёртка подключения и команд дрона |
| `src/AirSimDatasetCreator/SensorRecorder.py` | Потоки записи IMU/GPS/камер/GT |
| `src/AirSimDatasetCreator/DatasetWriterEuRoCLike.py` | Формирование EuRoC-структуры и запись строк |
| `src/AirSimDatasetCreator/TrajectoryExecuter.py` | Исполнитель команд JSON-траектории |
| `src/AirSimDatasetCreator/IP.py` | Константы `DESKTOP_IP`, `PORT` |
| `src/AirSimDatasetCreator/instruction.txt` | Инструкция (pip, api_server, send-command) |
| `MyTest.py` | Мини-тест получения кадра с камеры `front` |
| `trajectories/*.json` | 8 готовых траекторий |
| `settings.json` | Настройки AirSim (в корне проекта) |

---

## 1. `test.py` — классический запуск

Поток: открыть `trajectories/*.json` → для каждой траектории:

1. `AirSimDroneConnector(ip, port)` + `enable()` (взять API-контроль, `armDisarm(True)`) — test.py:21–22;
2. **четыре отдельных клиента** для камер/IMU/GPS/writer — test.py:24–34 (в MS cam_client — обёртка, остальные — голые `MultirotorClient`);
3. `EuRoCDatasetWriter` → `create_structure()` + `init_csv_files()` — test.py:36–39;
4. `SensorRecorder` с `cam0_name='cam0', cam1_name='cam1'` — test.py:41–44;
5. `start_imu_recording(200)` + `start_gps_recording(10)` — test.py:49–50;
6. `TrajectoryExecutor.execute(commands)` — test.py:53;
7. `finally`: стоп потоков, `drone.disable()` — test.py:55–57.

Замечания: камеры здесь пишутся только если в JSON есть команды `start`/`stop` (см. п. 4); GT отдельно не пишется вовсе.

---

## 2. `AirSimDroneConnector.py` — обёртка подключения

- `__post_init__`: создаёт `MultirotorClient` и `confirmConnection()` (строки 16–18).
- `enable()/disable()`: API-контроль и arm/disarm (20–34).
- `takeoff/land/stop`: `.join()` на асинхронных командах (26–42).
- `move_by_velocity(vx, vy, vz, duration)`: `moveByVelocityAsync` (43–48).
- **Преобразование СК**: `get_position()` возвращает `(x, -z, y)` (строки 49–53), `get_orientation()` — углы из переставленного кватерниона `(x, -z, y)` (строки 55–75).
- `get_ground_truth()`: `simGetGroundTruthKinematics()`, позиция/скорость `(x,-z,y)`, **кватернион переставлен** `(qw, qx, -qz, qy)` (77–92) — см. документ «Исправление_ошибок_ModelingSystem.md», п. 1.

Класс дублирует функциональность `airsim.MultirotorClient`, но часть методов возвращает уже преобразованные в СК F данные — это создаёт риск смешения сырых NED и F-данных в вызывающем коде.

---

## 3. `SensorRecorder.py` — запись данных

Инициализация (16–36): три клиента + writer; события остановки для IMU/GPS; флаг `camera_recording_active`, `camera_hz=20`; `start_timestamp_ns`.

**IMU** `record_imu` (79–95): `getImuData()`, метка `imu.time_stamp`, перестановка угловых скоростей/ускорений `(x, -z, y)`, запись в writer без времени `time_s`.

**GPS** `record_gps` (97–120): `getGpsData()`, отбрасывание `is_valid=False`, метка `gps.time_stamp`, `(x,-z,y)` для скорости.

**Камеры** `record_stereo_images` (184–214): один `simGetImages` на cam0+cam1 (Scene, без сжатия), проверка размеров, сохранение PNG + строка в `data.csv`.

**Оценка** `record_ground_estimate` (216–252): `getMultirotorState()` → `kinematics_estimated`; углы от переставленного кватерниона; пишет в `state_ground_estimate0` через `write_gt_row`.

**Истина** `record_ground_truth` (254–283): `get_ground_truth()` (истина) + **отдельный** вызов `getMultirotorState()` только ради метки; пишет в `state_ground_truth0` через `write_true_gt_row`. Обе функции содержат `print("DEBUG: ...")`.

**Потоки**:
- `start_imu_recording(200)` / `stop_imu_recording` — свой поток (122–152);
- `start_gps_recording(10)` / `stop_gps_recording` — свой поток (154–182);
- `start_camera_recording(20)` / `stop_camera_recording` — **daemon-поток** `_camera_recording_loop` (46–77), внутри цикла: `record_stereo_images()` + `record_ground_estimate()` + `record_ground_truth()` каждые 1/20 с.

Циклы IMU/GPS построены «`start = perf_counter()`; запись; `sleep(dt − elapsed)`» без `timeBeginPeriod`.

**Ключевые недостатки:** GT и оценка пишутся с частотой камер (~20 Гц), а не независимо; метка «истины» берётся из другого RPC; перестановка кватерниона некорректна; камера — daemon-поток, который останавливается лишь командами `start`/`stop`.

---

## 4. `TrajectoryExecuter.py` — команды траектории

`execute(commands)` (25–127): перебор команд; каждая — ветка `if/elif`.

- `takeoff` / `land` — `.join()` асинхронных команд;
- `hover` — `hoverAsync().join()` + `sleep(2.0)`;
- `velocity` — `moveByVelocityAsync(vx, vz, -vy, dur)` + `_motion_with_record` (перестановка в F-СК);
- `velocity_body` — `moveByVelocityBodyFrameAsync(vx, vz, -vy, dur)` (перестановка NED-типа применена к **телесной** СК — некорректно);
- `move_to` — `moveToPositionAsync(x, z, -y, v)` + `_motion_with_record_flow`;
- `yaw_rate` / `yaw_to` — вращение;
- `path` — точки переставляются `(x, z, -y)`;
- `move` — целевая точка от `get_position()` (уже F!) снова переставляется в NED — двойное преобразование;
- `start` — взлёт, `set_start_time(state.timestamp)`, запуск всех трёх потоков записи;
- `stop` — остановка всех потоков записи, посадка.

**Проблемы:** 
- без `start` камеры/GT не пишутся, а `test.py` использует траектории без `start`;
- преобразования СК размазаны и частью некорректны (двойное в `move`);
- `_motion_with_record` / `_motion_with_record_flow` / `record_during_motion` — по сути мёртвый код: `record_during_motion` только спит, `record_loop` в `_motion_with_record_flow` не записывает ничего (строки 138–211).

---

## 5. `DatasetWriterEuRoCLike.py` — формат датасета

Структура (12–47): `cam0|cam1/data.csv` + `data.data` + `data/`; `imu0/`, `gps0/`, `state_ground_estimate0/`, `state_ground_truth0/`.

- `_format_timestamp` (55–71): абсолютный `timestamp_ns` → строка `ЧЧ:ММ:СС.нс` от `start_timestamp_ns`; отрицательные обрезаются до 0; если старт не задан — возвращает абсолютное число.
- `init_csv_files` (81–193): создаёт `.csv` (разделитель `,`) и `.data` (разделитель `\t`) с заголовками. Заголовок `#timestamp [ns]` вводит в заблуждение: реально пишется **относительное** время.
- Методы записи `write_imu_row`, `write_gps_row`, `write_gt_row`, `write_true_gt_row`, `write_camera_image`: каждый раз открывают файл на `"a"` и пишут **две** копии (`.csv` + `.data`). Абсолютный ns в строках отсутствует (только в имени PNG).
- `write_command_log` (245–253): рядом с датасетом сохраняет список команд.

**Замечания:** дублирование `.csv`/`.data`, открытие файла на каждую запись, потеря абсолютного времени в данных, заголовки не соответствуют содержимому.

---

## 6. `dataset_manager.py` — режимы online/offline

`DatasetManager.initialize` (73–127): настройки, имя датасета `mav_online_<сессия>` или `mav_<траектория>_<сессия>`, подключение, `EuRoCDatasetWriter`, `SensorRecorder` с именами **`front_left`/`front_right`** (строка 108–110 — не совпадает с `cam0`/`cam1` в settings.json!), `TrajectoryExecutor`.

`start` (142–159): `enable()` + `takeoff()`, запуск IMU/GPS, ветка `_run_online` / `_run_offline`, в `finally` — `finish()`.

**Online** `_run_online` (189–271): поднимает Flask на 5000, цикл: очередь API-команд → `_execute_api_command` (161–187, движение в **сырых NED** без перестановок — рассинхрон с executor'ом); опрос клавиатуры (стрелки, Q/E, пробел, Esc); запись GT + кадров раз в 1/20 с.

**Offline** `_run_offline` (273–289): читает JSON траектории, `executor.execute(commands)` на каждую.

**API-эндпоинты** (318–408): `/move`, `/status`, `/takeoff`, `/land` — добавляют команды в очередь `api_command_queue`.

**Замечания:** имя камеры `front_left`/`front_right` не совпадает с конфигурацией; в online движение подаётся без перестановок СК, а в offline — с ними; GT записывается только в online-цикле, в offline его нет.

---

## 7. `api_server.py` — сервер заданий

- `SimulationConfig` (19–34): параметры по умолчанию.
- `StepSession` (41–136): пошаговая **online**-сессия — очередь команд, фоновый поток `_process_queue` выполняет каждую через `TrajectoryExecutor.execute([cmd])`; `finish()` останавливает запись, сажает дрон, отдаёт путь к датасету.
- `Job` (139–246): **offline**-задание — читает `trajectories`, подключает дрон, создаёт writer/recorder, выполняет все траектории с прогрессом `progress_callback`, в конце останавливает запись.
- Эндпоинты: `POST /start` (265–312) — `trajectory_file` → задание `mode='offline'`; одиночная команда/массив → сессия `mode='online'`; `/finish`, `/status`, `/status/<job_id>`, `/dataset/<job_id>` (ZIP), `/stop/<job_id>`.

**Замечания:** проверка пути `trajectory_file` есть, но выгрузка ZIP — без аутентификации; recorder в Job создаётся с `cam0_name='cam0', cam1_name='cam1'`, в сессии — тоже, а в `dataset_manager.py` — иначе (несоответствие между двумя серверными реализациями).

---

## 8. Итог по архитектуре

Поток данных: **JSON-траектория → `TrajectoryExecutor` → `AirSimDroneConnector`/клиенты → `SensorRecorder` → `DatasetWriterEuRoCLike` → датасет (`.csv`+`.data`+PNG)**.

Сильные стороны: два режима записи (online/offline), сервер заданий с прогрессом и выгрузкой, инструкции, набор готовых траекторий.

Слабые места: некорректные преобразования СК/кватернионов (см. отдельный документ), GT привязана к камерному циклу, потеря абсолютного времени, дублирование файлов и открытие их на каждую запись, рассинхрон имён камер между реализациями, мёртвый код в `TrajectoryExecuter`.
