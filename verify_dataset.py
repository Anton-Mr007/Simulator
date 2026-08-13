import argparse
import math
import sys
from pathlib import Path

import numpy as np


def _read(path: Path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            rows.append([float(x) for x in parts])
    return np.array(rows)


def main():
    parser = argparse.ArgumentParser(description="Проверка датасета CorrectSimulator")
    parser.add_argument("dataset", nargs="?", default="dataset/mav_square_1000m",
                        help="Каталог датасета (с imu.dat, gps.dat, angle.dat)")
    args = parser.parse_args()

    ds = Path(args.dataset)
    if not ds.is_dir():
        sys.exit(f"Нет каталога датасета: {ds}")

    imu_path, gps_path, angle_path = ds / "imu.dat", ds / "gps.dat", ds / "angle.dat"
    for p in (imu_path, gps_path, angle_path):
        if not p.is_file():
            sys.exit(f"Нет файла: {p}")

    imu = _read(imu_path)
    gps = _read(gps_path)
    ang = _read(angle_path)

    print(f"Датасет: {ds}")
    print(f"  imu.dat  : {imu.shape[0]} строк")
    print(f"  gps.dat  : {gps.shape[0]} строк")
    print(f"  angle.dat: {ang.shape[0]} строк")

    t_min = min(imu[0, 0], gps[0, 0], ang[0, 0])
    t_max = max(imu[-1, 0], gps[-1, 0], ang[-1, 0])
    print(f"  диапазон времени time_s: {t_min:.1f} .. {t_max:.1f} с")

    dt = np.diff(imu[:, 0])
    print(f"  imu dt: среднее {dt.mean()*1000:.2f} мс, пропуски >6 мс: {(dt > 0.006).sum()}")

    ay = imu[:, 6]
    mask_flat = (np.abs(ang[:, 2]) < np.deg2rad(5)) & (np.abs(ang[:, 3]) < np.deg2rad(5))
    if mask_flat.sum() > 0:
        print(f"  горизонтальные участки ({mask_flat.sum()} отсчётов angle.dat):")
        print(f"    ay среднее: {ay[mask_flat].mean():.3f} м/с^2 (ожидается ~ +9.81 при оси Y вверх)")
        print(f"    |ay-9.81| среднее: {np.abs(ay[mask_flat] - 9.81).mean():.3f}")
    else:
        print("  горизонтальных участков не найдено (roll/pitch всюду > 5°)")

    north = gps[:, 5]
    east = gps[:, 7]
    speed = np.hypot(north, east)
    mask_move = speed > 1.0
    n_move = int(mask_move.sum())
    if n_move > 0:
        n = min(len(gps), len(ang))
        hdg = np.degrees(np.arctan2(east[:n], north[:n]))[mask_move[:n]]
        yaw = np.degrees(ang[:n, 4])[mask_move[:n]]
        diff = np.abs(((hdg - yaw + 180) % 360) - 180)
        print(f"  движение (|V|>1 м/с): {n_move} отсчётов GPS")
        print(f"    |yaw - курс| среднее: {diff.mean():.1f}° (ожидается < 10°)")
        print(f"    |yaw - курс| 90-й перцентиль: {np.percentile(diff, 90):.1f}°")
    else:
        print("  движения не обнаружено (проверьте траекторию)")

    print("Проверка завершена.")


if __name__ == "__main__":
    main()
