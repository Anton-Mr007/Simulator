# CorrectSimulator

Максимально корректный генератор синтетических датасетов сенсорных данных
дрона в AirSim, собранный на основе сравнения `AirSimDatasetCreator`
(наш) и `ModelingSystem-1.1` (см. `C:\АО ИТТ\comparison.md`).

## Система координат

AirSim работает в NED (X=север, Y=восток, Z=вниз).

Все команды траекторий (`trajectories/*.json`) задаются **в NED без
перестановок** — поля `vx/vy/vz`, `x/y/z` передаются в AirSim как есть.
Единственное исключение — перед мировой командой `velocity` нос дрона
доворачивается на курс движения (`rotateToYawAsync(atan2(vy, vx))`),
чтобы камера смотрела вперёд.

Запись данных ведётся в единую выходную СК F = (X=север, Y=вверх,
Z=восток). Перевод выполняется ТОЛЬКО в `SensorRecorder`, в одном месте:

- векторы: `(x, -z, y)`;
- кватернион: `q_F = q_R ⊗ q_NED`, `q_R = (1/√2, 1/√2, 0, 0)` — поворот
  на 90° вокруг X (математически корректная смена базиса);
- углы Эйлера считаются от сырого (NED) кватерниона AirSim — только
  они дают физические углы (крен/тангаж относительно горизонта, курс
  от севера); довёрнутый `q_F` записывается только в `state_ground_truth0`.

## Ground truth

Используется `simGetGroundTruthKinematics()` — истинная кинематика
симулятора (не оценка полётного контроллера).

## Формат датасета

```
dataset/mav_{id}/
├── imu.dat            # time_s timestamp_ns wx wy wz ax ay az
├── gps.dat            # time_s timestamp_ns lat lon alt vx vy vz
├── angle.dat          # time_s timestamp_ns roll pitch yaw
├── state_ground_truth0/data.dat        # time_s ... px py pz qw qx qy qz vx vy vz
├── cam0/data.dat, cam0/data/*.png
└── cam1/data.dat, cam1/data/*.png
```

Все файлы — табулированные `.dat`; первая колонка — `time_s`
(единая шкала от момента запуска записи), вторая — абсолютный
`timestamp_ns` сенсора.

## Запуск

1. Запустить симулятор LandscapeMountains (порт 41451).
2. Поместить `settings.json` рядом с `.exe` симулятора.
3. `python test.py` — траектория `trajectories/square_1000m.json`,
   данные сохраняются в `dataset/mav_square_1000m/`.

Требования — `requirements.txt` (Python 3.10, airsim 1.8.1).

## Команды траекторий

`takeoff`, `land`, `hover`, `velocity` (мировая СК, NED, с доворотом носа),
`velocity_body`, `velocity_z`, `move_to`, `move`, `yaw_rate`, `yaw_to`, `path`.
