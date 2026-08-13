from dataclasses import dataclass, field

import airsim


@dataclass
class AirSimDroneConnector:
    '''
    Подключение и базовые команды дрона.
    Все координаты/скорости здесь — в мировой системе AirSim NED
    (X=север, Y=восток, Z=вниз). Пересчёт в выходную СК выполняется
    только в SensorRecorder.
    '''

    ip: str
    port: int
    client: airsim.MultirotorClient = field(init=False)

    def __post_init__(self):
        self.client = airsim.MultirotorClient(ip=self.ip, port=self.port)
        self.client.confirmConnection()

    def enable(self):
        self.client.enableApiControl(True)
        print('Установлено соединение по API')
        self.client.armDisarm(True)
        print('Вертушки включены')

    def disable(self):
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
        print('Соединение API разорвано')

    def takeoff(self, timeout_sec: float = 10.0):
        self.client.takeoffAsync(timeout_sec=timeout_sec).join()
        print('Взлет')

    def land(self, timeout_sec: float = 10.0):
        self.client.landAsync(timeout_sec=timeout_sec).join()
        print('Посадка')

    def get_position_ned(self):
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        return pos.x_val, pos.y_val, pos.z_val
