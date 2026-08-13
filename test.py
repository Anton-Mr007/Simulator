import json
import math

import airsim

from src.AirSimDatasetCreator.AirSimDroneConnector import AirSimDroneConnector
from src.AirSimDatasetCreator.DatasetWriterEuRoCLike import EuRoCDatasetWriter
from src.AirSimDatasetCreator.SensorRecorder import SensorRecorder
from src.AirSimDatasetCreator.TrajectoryExecuter import TrajectoryExecutor

from src.AirSimDatasetCreator.IP import DESKTOP_IP, PORT


with open('trajectories/square_1000m.json', 'r', encoding='utf-8') as f:
    dct = json.load(f)

for _dct in dct['trajectories']:
    print(f"Выполняется симуляция {_dct['description']}")
    commands = _dct['commands']

    drone = AirSimDroneConnector(ip=DESKTOP_IP, port=PORT)
    drone.enable()

    cam_client = airsim.MultirotorClient(ip=DESKTOP_IP, port=PORT)
    cam_client.confirmConnection()

    imu_client = airsim.MultirotorClient(ip=DESKTOP_IP, port=PORT)
    imu_client.confirmConnection()

    gps_client = airsim.MultirotorClient(ip=DESKTOP_IP, port=PORT)
    gps_client.confirmConnection()

    gt_client = airsim.MultirotorClient(ip=DESKTOP_IP, port=PORT)
    gt_client.confirmConnection()

    writer = EuRoCDatasetWriter(root_dir='dataset', dataset_name=f"mav_{_dct['id']}")
    writer.create_structure()
    writer.init_csv_files()
    writer.write_command_log(commands)

    recorder = SensorRecorder(
        cam_client, imu_client=imu_client, gps_client=gps_client, gt_client=gt_client,
        writer=writer, cam0_name='cam0', cam1_name='cam1', cam2_name='cam2',
    )
    executer = TrajectoryExecutor(drone=drone, recorder=recorder)

    try:
        recorder.start_recording(hz=200)

        start_pos = drone.client.getMultirotorState().kinematics_estimated.position
        print(f'Стартовая позиция: ({start_pos.x_val:.2f}, {start_pos.y_val:.2f}, {start_pos.z_val:.2f})')

        command_log = executer.execute(commands)

        end_pos = drone.client.getMultirotorState().kinematics_estimated.position
        print(f'Финальная позиция: ({end_pos.x_val:.2f}, {end_pos.y_val:.2f}, {end_pos.z_val:.2f})')

        dx = end_pos.x_val - start_pos.x_val
        dy = end_pos.y_val - start_pos.y_val
        dz = end_pos.z_val - start_pos.z_val
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        print(f'Отклонение от старта: {dist:.2f} м')

        print('---ЛОГ выполненых команд---')
        print(*command_log, sep='\n--')

    finally:
        recorder.stop_recording()
        drone.disable()
