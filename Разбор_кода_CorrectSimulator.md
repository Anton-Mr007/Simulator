# Разбор кода CorrectSimulator

Разбор целевого проекта `C:\АО ИТТ\CorrectSimulator` (v0.5). Ссылки на файлы и строки.

---

## Состав проекта

| Файл | Назначение |
|---|---|
| `test.py` | Точка входа: подключение, запуск записи, выполнение траектории |
| `settings.json` | Настройки AirSim (cam0/cam1/cam2), дубль в `WindowsNoEditor` |
| `verify_dataset.py` | Проверка целостности датасета (частоты, углы, курс) |
| `src/AirSimDatasetCreator/AirSimDroneConnector.py` | Обёртка подключения (только базовые команды) |
| `src/AirSimDatasetCreator/SensorRecorder.py` | 3 потока записи (IMU/GPS/GT) + камеры |
| `src/AirSimDatasetCreator/DatasetWriterEuRoCLike.py` | Формирование датасета и запись строк |
| `src/AirSimDatasetCreator/TrajectoryExecuter.py` | Исполнитель команд NED-траектории |
| `src/AirSimDatasetCreator/IP.py` | Константы `DESKTOP_IP`, `PORT` |
| `trajectories/*.json` | 8 траекторий (square_1000m и др.) |

---

## 1. `test.py` — точка входа

Поток (test.py:14–68):

1. Чтение `trajectories/square_1000m.json` (или другого файла по коду).
2. Для каждой траектории:
   - `AirSimDroneConnector(ip, port)` + `enable()` (API-контроль, arm) — test.py:21–22;
   - **пять клиентов**: cam/imu/gps/gt/writer — test.py:24–34;
   - `EuRoCDatasetWriter(root_dir='dataset', dataset_name=f"mav_{_dct['id']}")` + `create_structure()` + `init_csv_files()` + `write_command_log()` — test.py:36–39;
   - `SensorRecorder(cam, imu, gps, gt, writer, cam0/cam1/cam2)` — test.py:41–44;
   - `recorder.start_recording(hz=200)` — **запись включается сразу**, не зависит от команд траектории — test.py:48;
   - вывод стартовой/финальной позиции и отклонения от старта — test.py:50–62;
   - `executer.execute(commands)` — test.py:53;
3. `finally`: `recorder.stop_recording()` + `drone.disable()` — test.py:67–68.

**Ключевая особенность:** запись не привязана к командам траектории (в отличие от MS, где нужны `start`/`stop`); остановка гарантирована блоком `finally`.

---

## 2. `AirSimDroneConnector.py` — минимальная обёртка

Всего 45 строк: `__post_init__` (подключение + `confirmConnection`), `enable()/disable()`, `takeoff()/land()`, `get_position_ned()` — возвращает **сырые NED** `(x, y, z)`.

Сознательно убрано всё преобразование СК из этого класса: здесь только NED, перевод в выходную СК F выполняется единственным местом — в `SensorRecorder`. Это устраняет рассинхрон между «сырыми» и «уже переставленными» данными, который был в MS.

---

## 3. `SensorRecorder.py` — ядро записи

**Документация класса** (8–22) фиксирует единое правило: NED → F = `(x, -z, y)` для векторов, `q_F = q_R ⊗ q_NED` для кватерниона; углы Эйлера — от сырого NED-кватерниона.

**Преобразования** (43–58):
- `_to_drone_frame(v)` → `(v.x, -v.z, v.y)`;
- `_quat_to_drone_frame(q)` → `s*(w-x), s*(x+w), s*(y-z), s*(z+y)`, `s=1/√2` — доворот на 90° вокруг X;
- `_quat_to_euler(w,x,y,z)` → стандартные формулы roll/pitch/yaw от **сырого** кватерниона.

**Запись потоков:**
- `record_imu` (63–69): `getImuData()`, метка `imu.time_stamp` сохраняется в `_last_imu_ns`, векторы через `_to_drone_frame`, строка + `time_s`.
- `record_gps` (71–83): `getGpsData()`, отбрасывание `is_valid=False`, метка `gps.time_stamp`.
- `record_ground_truth` (85–101): `simGetGroundTruthKinematics()`; метка = `_last_imu_ns` (fallback на `getMultirotorState().timestamp`); позиция/скорость через `_to_drone_frame`, кватернион через `_quat_to_drone_frame` → `state_ground_truth0/data.dat`; углы от сырого кватерниона → `angle.dat`.
- `record_camera_images` (103–127): **один** `simGetImages` на 3 камеры (Scene, без сжатия, 752×480); проверка пустых кадров; PNG + строка `data.dat` с меткой `response.time_stamp`.

**Старт/стоп** `start_recording` (129–164):
- `timeBeginPeriod(1)` + `sys.setswitchinterval(0.0005)`;
- `t0 = time.time()` — отсчёт `time_s`;
- внутренняя `loop(fn, rate)`: «`next_time += dt` по `perf_counter`, `sleep(0.0005)`» — 3 потока: IMU 200, GPS 10, GT 200;
- `stop_recording`: event, join потоков, `timeEndPeriod(1)`, `writer.flush()`.

**Особенности:** GT — свой поток на 200 Гц (не привязан к камерам); кадры пишутся синхронно из потока выполнения траектории (см. п. 4); в `record_imu` метка сохраняется для GT, т.к. у `simGetGroundTruthKinematics` своей метки нет (в airsim 1.8.1 нет `kinematics_ground_truth`).

---

## 4. `TrajectoryExecuter.py` — команды траектории

`CAMERA_HZ = 20` (21). `execute(commands)` (53–184) — ветки по типу команды:

- `takeoff` / `land` — асинхронные вызовы с `timeout_sec` из JSON + пауза 3/5 с;
- `hover` — `hoverAsync()` + `_sleep_with_recording(duration)`;
- `velocity` — **доворот курса** `rotateToYawAsync(atan2(vy, vx)).join()` (86–88) + `moveByVelocityAsync(vx, vy, vz, dur)` + запись камер;
- `velocity_body` — `moveByVelocityBodyFrameAsync(vx, vy, vz, dur)` + запись камер;
- `velocity_z` — `moveByVelocityZAsync(vx, vy, z, dur)` + запись камер (новая команда);
- `move_to` / `move` — `moveToPositionAsync(x, y, z, v)` + **опрос расстояния** до цели с дедлайном 60 с и порогом 1 м (127–131, 144–148);
- `yaw_rate` — `rotateByYawRateAsync` + запись камер;
- `yaw_to` — `rotateToYawAsync` + пауза 3 с;
- `path` — `moveOnPathAsync(path, v)` + оценка времени полёта по длине пути и запись камер.

**`_sleep_with_recording`** (41–51): цикл на время `duration`, каждые `1/CAMERA_HZ` вызывает `recorder.record_camera_images()`. Именно здесь пишутся кадры во время манёвра.

**Замечания:** команды принимаются **в NED без перестановок**; камеры пишутся только в командах с `_sleep_with_recording` (в `takeoff`/`land`/`move_to` кадров нет); команды `start`/`stop` не поддержаны — запись управляется из `test.py`.

---

## 5. `DatasetWriterEuRoCLike.py` — формат датасета

Структура (24–36): `dataset/cam0|cam1|cam2/data/*.png` + `cam*/data.dat`; `imu.dat`, `gps.dat`, `angle.dat`, `state_ground_truth0/data.dat`.

**Формат строк** — табулированные `.dat`, первая колонка `time_s` (float), вторая — абсолютный `timestamp_ns`:

- `imu.dat`: `time_s, ns, wx, wy, wz, ax, ay, az`
- `gps.dat`: `time_s, ns, lat, lon, alt, vx, vy, vz`
- `state_ground_truth0/data.dat`: `time_s, ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz`
- `angle.dat`: `time_s, ns, roll, pitch, yaw`
- `cam*/data.dat`: `time_s, ns, filename`

**Механизм записи** (42–66): файлы открываются один раз (`_open`), держатся открытыми, каждая запись под `threading.Lock` (`_write_line`); `flush()` — при остановке. Это исключает потерю данных при сбое и ускоряет запись по сравнению с MS.

`write_camera_image` (73–89): `cv2.cvtColor(RGB→BGR)` → `cv2.imwrite` PNG; имя файла = `timestamp_ns`. `write_command_log` (107–111): сохраняет команды траектории.

---

## 6. `verify_dataset.py` — проверка целостности

Читает `imu.dat`, `gps.dat`, `angle.dat` (пропуская строки с `#`), numpy-анализ:

- количество строк и диапазон `time_s` (37–48);
- средний `dt` IMU и число пропусков > 6 мс (50–51);
- **проверка гравитации**: на горизонтальных участках (roll/pitch < 5° из `angle.dat`) среднее `ay` должно быть ≈ +9.81 м/с² при оси Y вверх (53–60);
- **проверка курса**: при движении |V| > 1 м/с курс из GPS-скоростей (`atan2(east, north)`) сравнивается с yaw из `angle.dat`, ожидается расхождение < 10° (62–76).

Это автономный инструмент контроля качества датасета, отсутствующий в MS.

---

## 7. `settings.json` — конфигурация камер

- `SimMode: Multirotor`, `ApiServerPort: 41451`;
- cam0/cam1 — стерео вперёд `(X=0.35, Y=∓0.1, Z=-0.05)`, Pitch/Roll/Yaw = 0;
- cam2 — вниз `(X=0, Y=0, Z=0.3, Pitch=-90)`, FOV 90°, 752×480;
- у всех камер: `MotionBlurAmount=0`, автоэкспозиция отключена (`AutoExposureMax/MinBrightness=1`);
- IMU с шумовыми параметрами (GyroNoiseDensity и др.).

**Важно:** симулятор читает свой `settings.json` из `WindowsNoEditor` (применяется при старте); файл в корне проекта — рабочая копия, оба должны совпадать.

---

## 8. Поток данных и итог

Поток: **JSON-траектория (NED) → `TrajectoryExecutor` → `AirSimDroneConnector` → `SensorRecorder` → `DatasetWriterEuRoCLike` → датасет (.dat + PNG)**.

Архитектурные решения, исправляющие проблемы MS:
1. Единственная точка перевода NED→F в `SensorRecorder`; команды траекторий — в сыром NED.
2. GT 200 Гц отдельным потоком, только истина из `simGetGroundTruthKinematics()`.
3. Углы от сырого NED-кватерниона в отдельном `angle.dat`.
4. `time_s` + абсолютный `timestamp_ns` в одном `.dat`; открытые файлы с lock.
5. Точный тайминг циклов (`timeBeginPeriod(1)`, `perf_counter`).
6. Доворот курса при `velocity`; контроль достижения точки в `move_to`/`move`.
7. Третья камера cam2 вниз; единые имена камер; settings в репо.

Осознанные ограничения: фактически ~4.2 fps камер (синхронный `simGetImages`), GT-метка от последнего IMU (приближение), нет online-режима и серверной части (см. отчёт о сравнении).
