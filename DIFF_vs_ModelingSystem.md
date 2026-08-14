# Отличия CorrectSimulator от ModelingSystem-1.1

Сравнение: **CorrectSimulator** (этот проект) и **C:\АО ИТТ\ModelingSystem-1.1** (исходный проект AirSimDatasetCreator, форк AbsltRaDu).

---

## 1. Краткая сводка

| Аспект | ModelingSystem-1.1 | CorrectSimulator |
|---|---|---|
| Камеры | 2 (cam0, cam1 — стерео вперёд) | 3 (cam0, cam1 — стерео вперёд, cam2 — вниз, Pitch=-90) |
| Ground truth | 2 файла: `state_ground_estimate0` (оценка ПК) + `state_ground_truth0` (истина) | только `state_ground_truth0` (истина) |
| Частота GT | ~20 Гц (привязана к камерному циклу) | 200 Гц (отдельный поток) |
| Углы Эйлера | колонки в строках GT | отдельный файл `angle.dat`, от сырого NED-кватерниона |
| Формат времени | 1-я колонка `ЧЧ:ММ:СС.нс` (относит. от `start`), 2-я — абсолютный ns | 1-я `time_s` (сек от начала записи, float), 2-я — абсолютный ns |
| Файлы данных | дублируются `.csv` + `.data` (таб), файл открывается на каждую запись | только `.dat` (таб), файлы держатся открытыми + lock |
| СК в командах траекторий | F-СК (X=север, Y=вверх, Z=восток) с перестановками | прямо NED (X=север, Y=восток, Z=вниз), без перестановок |
| Кватернион в СК F | перестановка компонент `(qw, qx, -qz, qy)` — некорректно | `q_F = q_R ⊗ q_NED`, `q_R = (1/√2, 1/√2, 0, 0)` — корректный доворот |
| Углы Эйлера | от переставленного кватерниона (физически неверно) | от сырого NED-кватерниона (физически верно) |
| Запись камер | непрерывный daemon-поток между командами `start`/`stop` | только внутри `_sleep_with_recording` у активных команд (hover/velocity/…) |
| Команды `start`/`stop` | есть (взлёт + старт записи / стоп записи + посадка) | нет (запись стартует в `test.py` до команд) |
| Доп. команды | — | `velocity_z`; `velocity` доворачивает курс по направлению движения |
| Контроль достижения точки | нет (просто `future.join()`) | есть: ожидание до 60 с, пока дистанция < 1 м |
| Синхронизация времени | `timeBeginPeriod` нет, обычные циклы со `sleep` | `timeBeginPeriod(1)`, `sys.setswitchinterval(0.0005)`, цикл по `perf_counter` |
| GT-метка времени | `getMultirotorState().timestamp` | метка последнего IMU (`_last_imu_ns`), fallback — state.timestamp |
| Метка старта | `set_start_time(state.timestamp)` в команде `start` | `t0 = time.time()` в `start_recording` |
| Собственный `settings.json` | копируется из `AirSimDatasetCreator` в папку `.exe` вручную | лежит в корне проекта + дубль в `WindowsNoEditor` |
| Инструменты | test.py, README, instruction.txt | test.py, README, verify_dataset.py, git-репозиторий |

---

## 2. Системы координат и команды траекторий

**ModelingSystem-1.1** задаёт траектории в F-СК (X=север, Y=вверх, Z=восток) и **переставляет координаты при передаче в AirSim** (NED: X=север, Y=восток, Z=вниз):

| Команда | Вызов в MS | Вызов в CS |
|---|---|---|
| `velocity` | `moveByVelocityAsync(vx, vz, -vy)` | `moveByVelocityAsync(vx, vy, vz)` |
| `velocity_body` | `moveByVelocityBodyFrameAsync(vx, vz, -vy)` | `moveByVelocityBodyFrameAsync(vx, vy, vz)` |
| `move_to` | `moveToPositionAsync(x, z, -y, v)` | `moveToPositionAsync(x, y, z, v)` |
| `move` | `moveToPositionAsync(x+dx, z+dz, -(y+dy), v)` | `moveToPositionAsync(x+dx, y+dy, z+dz, v)` |
| `path` | точки переставляются `(x, z, -y)` | точки передаются как есть |

**CorrectSimulator** работает с NED напрямую. Единственный перевод в выходную СК F выполняется в одном месте — `SensorRecorder._to_drone_frame` / `_quat_to_drone_frame`.

Дополнительно CS при команде `velocity` доворачивает нос на курс движения (`rotateToYawAsync(atan2(vy, vx))`), чтобы камера смотрела вперёд.

---

## 3. Ground truth и углы

| | ModelingSystem-1.1 | CorrectSimulator |
|---|---|---|
| Оценка (ПК) | `record_ground_estimate()`: `kinematics_estimated` из `getMultirotorState()` → `state_ground_estimate0` | нет |
| Истина | `record_ground_truth()`: `simGetGroundTruthKinematics()` → `state_ground_truth0` | `simGetGroundTruthKinematics()` → `state_ground_truth0` |
| Частота | обе функции в камерном цикле (~20 Гц) | 200 Гц в отдельном потоке |
| Углы | roll/pitch/yaw считаются от переставленного кватерниона `(x, -z, y)` и пишутся в строку GT | roll/pitch/yaw считаются от **сырого NED**-кватерниона и пишутся в `angle.dat` (курс против часовой: 0°=север, +90°=запад) |
| Кватернион | `(qw, qx, -qz, qy)` — механическая перестановка, не поворот | `q_F = q_R ⊗ q_NED`, `q_R=(1/√2,1/√2,0,0)` — доворот на 90° вокруг X |

Перестановка компонент кватерниона (как в MS) не эквивалентна смене СК и искажает углы; CS поворачивает кватернион корректным произведением, а углы берёт из исходного NED-кватерниона, где они физически корректны.

---

## 4. Временные метки и формат файлов

**ModelingSystem-1.1**: `_format_timestamp` превращает абсолютный `timestamp_ns` в относительную строку `ЧЧ:ММ:СС.нс` от `start_timestamp_ns`. Каждая строка пишется в два файла (`.csv` запятые и `.data` таб), файл открывается заново при каждой записи (медленно, риск потери при сбое).

**CorrectSimulator**: первая колонка — `time_s` (float, секунды от `t0`, клиентская шкала), вторая — абсолютный `timestamp_ns` сенсора. Один набор `.dat`-файлов, держатся открытыми, защищены `threading.Lock`, сброс через `flush()` при `stop_recording`.

| Файл | MS | CS |
|---|---|---|
| IMU | `imu0/data.csv` + `imu0/data.data` | `imu.dat` |
| GPS | `gps0/data.csv` + `gps0/data.data` | `gps.dat` |
| Оценка | `state_ground_estimate0/data.*` | — |
| Истина | `state_ground_truth0/data.*` (со встроенными углами) | `state_ground_truth0/data.dat` (без углов) |
| Углы | в строках GT | `angle.dat` |
| Камеры | `cam0/data.csv` + `cam0/data.data` + `cam0/data/*.png` | `cam0/data/*.png` + `cam0/data.dat` (2 камеры → 3) |

---

## 5. Архитектура записи

**ModelingSystem-1.1**
- `start_imu_recording(200)` / `start_gps_recording(10)` — отдельные потоки;
- `start_camera_recording(20)` — daemon-поток: `record_stereo_images()` + `record_ground_estimate()` + `record_ground_truth()` в одном цикле;
- старт/стоп всего запускается командами `start` / `stop` внутри траектории;
- циклы: `start = perf_counter(); запись; sleep(dt - elapsed)`.

**CorrectSimulator**
- `start_recording(hz=200, gps_hz=10, gt_hz=200)` — 3 независимых потока IMU / GPS / GT;
- камеры пишутся **синхронно** из потока выполнения команд через `_sleep_with_recording` (только у команд `hover`, `velocity`, `velocity_body`, `velocity_z`, `yaw_rate`, `path`; при `move_to`/`takeoff`/`land` кадры не пишутся);
- `timeBeginPeriod(1)` + цикл по `perf_counter`, итерации ~0.5 мс;
- команды `start`/`stop` не поддерживаются — запись включается в `test.py` до выполнения траектории и выключается в `finally`.

---

## 6. Прочее

- **Settings**: MS требует вручную копировать `settings.json` из `AirSimDatasetCreator` к `.exe`; CS держит свой `settings.json` в корне и предупреждает, что симулятор фактически читает файл из `AirSim\LandscapeMountains\WindowsNoEditor\settings.json` (применяется только при старте симулятора), поэтому оба файла должны быть идентичны.
- **cam2**: добавлена камера вниз `X=0, Y=0, Z=0.3, Pitch=-90, FOV=90°`; положение выбрано так, чтобы корпус дрона не попадал в кадр и камера не «закапывалась» в грунт при взлёте/стоянке.
- **Контроль скорости записи**: `CAMERA_HZ=20` в CS, но фактически ~4.2 fps из-за синхронного `simGetImages` на 3 несжатых кадра 752×480 (осознанно оставлено).
- **verify_dataset.py**: CS дополнительно имеет скрипт проверки целостности датасета (частоты, дыры, согласованность меток).
- **requirements.txt**: CS фиксирует зависимости (numpy 2.2.x, airsim 1.8.1 и др.).
