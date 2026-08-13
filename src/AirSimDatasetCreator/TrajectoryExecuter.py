import math
import time
from typing import Optional

import airsim

from src.AirSimDatasetCreator.AirSimDroneConnector import AirSimDroneConnector
from src.AirSimDatasetCreator.SensorRecorder import SensorRecorder


class TrajectoryExecutor:
    '''
    Выполняет список команд движения из JSON.

    Все координаты и скорости в командах — в мировой системе NED
    (X=север, Y=восток, Z=вниз), без перестановок. Перед командой
    "velocity" нос дрона доворачивается на курс движения, чтобы
    камера смотрела вперёд.
    '''

    CAMERA_HZ = 20

    def __init__(self, drone: AirSimDroneConnector, recorder: Optional[SensorRecorder] = None):
        self.drone = drone
        self.client = drone.client
        self.recorder = recorder
        self.log_of_command = []

    def _get_position(self):
        try:
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            return pos.x_val, pos.y_val, pos.z_val
        except Exception:
            return 0, 0, 0

    def _distance_to(self, target_x, target_y, target_z):
        x, y, z = self._get_position()
        return math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2 + (z - target_z) ** 2)

    def _sleep_with_recording(self, duration):
        start = time.perf_counter()
        next_cam = start
        cam_dt = 1.0 / self.CAMERA_HZ

        while time.perf_counter() - start < duration:
            now = time.perf_counter()
            if self.recorder is not None and now >= next_cam:
                self.recorder.record_camera_images()
                next_cam += cam_dt
            time.sleep(0.005)

    def execute(self, commands):
        for number_of_command, command in enumerate(commands):

            print(f'Команда № {number_of_command} - {command["type"]}')
            command_type = command["type"]

            if command_type == 'takeoff':
                self.log_of_command.append('Взлет')
                try:
                    self.client.takeoffAsync(timeout_sec=command.get("timeout_sec", 15))
                except Exception:
                    pass
                time.sleep(3.0)

            elif command_type == 'land':
                self.log_of_command.append('Посадка')
                try:
                    self.client.landAsync(timeout_sec=command.get("timeout_sec", 15))
                except Exception:
                    pass
                time.sleep(5.0)

            elif command_type == 'hover':
                self.log_of_command.append('Зависание')
                try:
                    self.client.hoverAsync()
                except Exception:
                    pass
                self._sleep_with_recording(command.get("duration", 2.0))

            elif command_type == "velocity":
                self.log_of_command.append('Перемещение со скоростью (мировая СК)')
                try:
                    heading = math.degrees(math.atan2(command["vy"], command["vx"]))
                    self.client.rotateToYawAsync(heading).join()
                    time.sleep(1.0)
                except Exception:
                    pass
                try:
                    self.client.moveByVelocityAsync(
                        command["vx"], command["vy"], command["vz"], command["duration"]
                    )
                except Exception:
                    pass
                self._sleep_with_recording(command["duration"])

            elif command_type == 'velocity_body':
                self.log_of_command.append('Перемещение со скоростью (тело дрона)')
                try:
                    self.client.moveByVelocityBodyFrameAsync(
                        command["vx"], command["vy"], command["vz"], command["duration"]
                    )
                except Exception:
                    pass
                self._sleep_with_recording(command["duration"])

            elif command_type == 'velocity_z':
                self.log_of_command.append(f'Перемещение на высоте {command["z"]}')
                try:
                    self.client.moveByVelocityZAsync(
                        command["vx"], command["vy"], command["z"], command["duration"]
                    )
                except Exception:
                    pass
                self._sleep_with_recording(command["duration"])

            elif command_type == 'move_to':
                self.log_of_command.append(f'Перемещение в ({command["x"]}, {command["y"]}, {command["z"]})')
                try:
                    self.client.moveToPositionAsync(
                        command["x"], command["y"], command["z"], command["velocity"]
                    )
                except Exception:
                    pass
                deadline = time.perf_counter() + 60
                while time.perf_counter() < deadline:
                    if self._distance_to(command["x"], command["y"], command["z"]) < 1.0:
                        break
                    time.sleep(0.1)

            elif command_type == 'move':
                dx = command.get("dx", 0)
                dy = command.get("dy", 0)
                dz = command.get("dz", 0)
                velocity = command.get("velocity", 3)
                self.log_of_command.append(f'Смещение на ({dx}, {dy}, {dz})')
                x, y, z = self._get_position()
                try:
                    self.client.moveToPositionAsync(x + dx, y + dy, z + dz, velocity)
                except Exception:
                    pass
                deadline = time.perf_counter() + 60
                while time.perf_counter() < deadline:
                    if self._distance_to(x + dx, y + dy, z + dz) < 1.0:
                        break
                    time.sleep(0.1)

            elif command_type == 'yaw_rate':
                self.log_of_command.append(f'Вращение {command["yaw_rate"]} deg/s')
                try:
                    self.client.rotateByYawRateAsync(command["yaw_rate"], command["duration"])
                except Exception:
                    pass
                self._sleep_with_recording(command["duration"])

            elif command_type == 'yaw_to':
                self.log_of_command.append(f'Поворот на {command["yaw"]} deg')
                try:
                    self.client.rotateToYawAsync(command["yaw"])
                except Exception:
                    pass
                time.sleep(3.0)

            elif command_type == 'path':
                self.log_of_command.append('Полёт по пути')
                try:
                    self.client.moveOnPathAsync(command['path'], command['velocity'])
                except Exception:
                    pass
                pts = command['path']
                est_time = sum(
                    math.sqrt((pts[i + 1][0] - pts[i][0]) ** 2 +
                              (pts[i + 1][1] - pts[i][1]) ** 2 +
                              (pts[i + 1][2] - pts[i][2]) ** 2)
                    for i in range(len(pts) - 1)
                ) / command['velocity']
                self._sleep_with_recording(est_time + 2.0)

            else:
                raise ValueError(f"Неизвестная команда: {command_type}")

        return self.log_of_command
