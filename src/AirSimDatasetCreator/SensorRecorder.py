import threading
import time

import numpy as np
import airsim


class SensorRecorder:
    '''
    Считывает данные из AirSim и передает их writer.

    Единое преобразование координат NED -> F (X=север, Y=вверх, Z=восток)
    выполняется ТОЛЬКО здесь, в одном месте:
      - векторы:      (x, -z, y)
      - кватернион:   q_F = q_R (x) q,  q_R = (1/sqrt2, 1/sqrt2, 0, 0)
    Углы Эйлера считаются от сырого (NED) кватерниона AirSim — только
    они дают физические углы (крен/тангаж относительно горизонта,
    курс от севера). Довёрнутый q_F записывается только в ground truth.

    Ground truth берётся из simGetGroundTruthKinematics() — истинная
    кинематика симулятора (не оценка полётного контроллера).
    '''

    def __init__(self, cam_client, imu_client, gps_client, gt_client,
                 writer, cam0_name: str = "cam0", cam1_name: str = "cam1",
                 cam2_name: str = "cam2"):
        self.writer = writer
        self.cam_client = cam_client
        self.imu_client = imu_client
        self.gps_client = gps_client
        self.gt_client = gt_client

        self.cam0_name = cam0_name
        self.cam1_name = cam1_name
        self.cam2_name = cam2_name

        self._threads = []
        self._stop_event = threading.Event()

        self.t0 = None

    @staticmethod
    def _to_drone_frame(v):
        return (v.x_val, -v.z_val, v.y_val)

    @staticmethod
    def _quat_to_drone_frame(q):
        s = 0.7071067811865476
        w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
        return s * (w - x), s * (x + w), s * (y - z), s * (z + y)

    @staticmethod
    def _quat_to_euler(w, x, y, z):
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def _time_s(self):
        return time.time() - self.t0

    def record_imu(self):
        imu = self.imu_client.getImuData()
        timestamp_ns = imu.time_stamp
        wx, wy, wz = self._to_drone_frame(imu.angular_velocity)
        ax, ay, az = self._to_drone_frame(imu.linear_acceleration)
        self.writer.write_imu_row(timestamp_ns, wx, wy, wz, ax, ay, az, self._time_s())

    def record_gps(self):
        gps = self.gps_client.getGpsData()
        if not gps.is_valid:
            return
        timestamp_ns = gps.time_stamp
        geo = gps.gnss.geo_point
        vx, vy, vz = self._to_drone_frame(gps.gnss.velocity)
        self.writer.write_gps_row(
            timestamp_ns,
            geo.latitude, geo.longitude, geo.altitude,
            vx, vy, vz,
            self._time_s(),
        )

    def record_ground_truth(self):
        state = self.gt_client.getMultirotorState()
        timestamp_ns = state.timestamp
        kin = state.kinematics_ground_truth

        px, py, pz = self._to_drone_frame(kin.position)
        vx, vy, vz = self._to_drone_frame(kin.linear_velocity)
        qw, qx, qy, qz = self._quat_to_drone_frame(kin.orientation)

        self.writer.write_gt_row(
            timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz, self._time_s(),
        )

        raw = kin.orientation
        roll, pitch, yaw = self._quat_to_euler(raw.w_val, raw.x_val, raw.y_val, raw.z_val)
        self.writer.write_gt_euler_row(timestamp_ns, roll, pitch, yaw, self._time_s())

    def record_camera_images(self):
        responses = self.cam_client.simGetImages([
            airsim.ImageRequest(self.cam0_name, airsim.ImageType.Scene,
                                pixels_as_float=False, compress=False),
            airsim.ImageRequest(self.cam1_name, airsim.ImageType.Scene,
                                pixels_as_float=False, compress=False),
            airsim.ImageRequest(self.cam2_name, airsim.ImageType.Scene,
                                pixels_as_float=False, compress=False),
        ])

        if len(responses) != 3:
            raise RuntimeError(f"Ожидались 3 изображения, получено: {len(responses)}")

        time_s = self._time_s()

        for camera_name, response in zip(["cam0", "cam1", "cam2"], responses):
            if response.width == 0 or response.height == 0:
                raise RuntimeError(
                    f"Камера {camera_name} вернула пустое изображение. "
                    f"Проверь имя камеры в settings.json."
                )
            timestamp_ns = response.time_stamp
            image_1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            image_rgb = image_1d.reshape(response.height, response.width, 3)
            self.writer.write_camera_image(camera_name, timestamp_ns, image_rgb, time_s)

    def start_recording(self, hz=200, gps_hz=10):
        import ctypes
        import sys
        sys.setswitchinterval(0.0005)
        ctypes.windll.winmm.timeBeginPeriod(1)
        self.t0 = time.time()
        self._stop_event.clear()

        def loop(fn, rate):
            dt = 1 / rate
            next_time = time.perf_counter()
            while not self._stop_event.is_set():
                now = time.perf_counter()
                if now >= next_time:
                    fn()
                    next_time += dt
                time.sleep(0.0005)

        self._threads = [
            threading.Thread(target=loop, args=(self.record_imu, hz)),
            threading.Thread(target=loop, args=(self.record_gps, gps_hz)),
            threading.Thread(target=loop, args=(self.record_ground_truth, hz)),
        ]
        for t in self._threads:
            t.start()

    def stop_recording(self):
        self._stop_event.set()
        for t in getattr(self, "_threads", []):
            t.join()
        import ctypes
        ctypes.windll.winmm.timeEndPeriod(1)
        self.writer.flush()

    def record_once(self):
        self.record_ground_truth()
        self.record_camera_images()
