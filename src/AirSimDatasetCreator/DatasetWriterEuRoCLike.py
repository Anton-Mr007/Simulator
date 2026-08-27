from pathlib import Path

import cv2
import numpy as np


class EuRoCDatasetWriter:
    '''
    Создает структуру датасета, похожую на EuRoC MAV.

    IMU пишется в плоский файл imu.dat, данные СНС — в gps.dat,
    эталонные углы — в angle.dat. Все файлы — табулированные; первая
    колонка — time_s (секунды с момента запуска записи), вторая —
    абсолютный timestamp_ns сенсора.
    '''

    def __init__(self, root_dir: str, dataset_name: str):
        import threading
        self._locks = {}
        self._files = {}
        self.root_dir = Path(root_dir)
        self.dataset_name = dataset_name

        self.dataset_dir = self.root_dir / self.dataset_name

        self.cam0_dir = self.dataset_dir / "cam0" / "data"
        # self.cam1_dir = self.dataset_dir / "cam1" / "data"
        self.cam2_dir = self.dataset_dir / "cam2" / "data"

        self.cam0_csv_path = self.cam0_dir.parent / "data.dat"
        # self.cam1_csv_path = self.cam1_dir.parent / "data.dat"
        self.cam2_csv_path = self.cam2_dir.parent / "data.dat"
        self.imu_csv_path = self.dataset_dir / "imu.dat"
        self.gps_csv_path = self.dataset_dir / "gps.dat"
        self.gt_csv_path = self.dataset_dir / "state_ground_truth0" / "data.dat"
        self.angle_csv_path = self.dataset_dir / "angle.dat"

    def create_structure(self):
        for path in (self.cam0_dir, self.cam2_dir, self.gt_csv_path.parent):
            path.mkdir(parents=True, exist_ok=True)

    def _open(self, path: Path):
        import threading
        self._locks[str(path)] = threading.Lock()
        self._files[str(path)] = open(path, "a", newline="", encoding="utf-8")

    def _write_line(self, path: Path, line: str):
        with self._locks[str(path)]:
            self._files[str(path)].write(line)

    def flush(self):
        for path, lock in self._locks.items():
            with lock:
                self._files[path].flush()

    def init_csv_files(self):
        self.imu_csv_path.write_text("# time_s\ttimestamp_ns\twx\twy\twz\tax\tay\taz\n")
        self.gps_csv_path.write_text("# time_s\ttimestamp_ns\tlatitude\tlongitude\taltitude\tvx\tvy\tvz\n")
        self.gt_csv_path.write_text("# time_s\ttimestamp_ns\tpx\tpy\tpz\tqw\tqx\tqy\tqz\tvx\tvy\tvz\n")
        self.angle_csv_path.write_text("# time_s\ttimestamp_ns\troll\tpitch\tyaw\n")
        self.cam0_csv_path.write_text("# time_s\ttimestamp_ns\tfilename\n")
        # self.cam1_csv_path.write_text("# time_s\ttimestamp_ns\tfilename\n")
        self.cam2_csv_path.write_text("# time_s\ttimestamp_ns\tfilename\n")
        for path in (self.imu_csv_path, self.gps_csv_path, self.gt_csv_path,
                     self.angle_csv_path, self.cam0_csv_path, self.cam2_csv_path):
            self._open(path)

    def close(self):
        for f in self._files.values():
            f.close()
        self._files.clear()

    def write_camera_image(self, camera_name: str, timestamp_ns: int, image_rgb: np.ndarray, time_s: float):
        if camera_name == "cam0":
            image_dir, csv_path = self.cam0_dir, self.cam0_csv_path
        # elif camera_name == "cam1":
        #     image_dir, csv_path = self.cam1_dir, self.cam1_csv_path
        elif camera_name == "cam2":
            image_dir, csv_path = self.cam2_dir, self.cam2_csv_path
        else:
            raise ValueError(f"Неизвестная камера: {camera_name}. Используй 'cam0' или 'cam2'.")

        filename = f"{timestamp_ns}.png"
        image_path = image_dir / filename

        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(image_path), image_bgr)

        self._write_line(csv_path, f"{time_s:.6f}\t{timestamp_ns}\t{filename}\n")

    def write_imu_row(self, timestamp_ns, wx, wy, wz, ax, ay, az, time_s):
        self._write_line(self.imu_csv_path,
                         f"{time_s:.6f}\t{timestamp_ns}\t{wx}\t{wy}\t{wz}\t{ax}\t{ay}\t{az}\n")

    def write_gps_row(self, timestamp_ns, latitude, longitude, altitude, vx, vy, vz, time_s):
        self._write_line(self.gps_csv_path,
                         f"{time_s:.6f}\t{timestamp_ns}\t{latitude}\t{longitude}\t{altitude}\t{vx}\t{vy}\t{vz}\n")

    def write_gt_row(self, timestamp_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz, time_s):
        self._write_line(self.gt_csv_path,
                         f"{time_s:.6f}\t{timestamp_ns}\t{px}\t{py}\t{pz}\t{qw}\t{qx}\t{qy}\t{qz}\t{vx}\t{vy}\t{vz}\n")

    def write_gt_euler_row(self, timestamp_ns, roll, pitch, yaw, time_s):
        self._write_line(self.angle_csv_path,
                         f"{time_s:.6f}\t{timestamp_ns}\t{roll:.10f}\t{pitch:.10f}\t{yaw:.10f}\n")

    def write_command_log(self, commands):
        path = self.root_dir / f"trajectory_commands_{self.dataset_name}.txt"
        with open(path, "w", encoding="utf-8") as f:
            for i, command in enumerate(commands):
                f.write(f"{i}: {command}\n")
