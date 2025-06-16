import threading
import enum
import dataclasses

@dataclasses.dataclass
class DeviceInfo:
    device_name: str = ''
    model: str = ''
    serial_number: str = ''
    firmware_version: str = ''

class MotorDirection(enum.Enum):
    FORWARD = '-'
    BACKWARD = '+'
    IDLE = None

class Commands(enum.Enum):
    LIST_MOTORS = 'list_motors'
    GET_POSITION = 'get_position'
    STOP = 'stop'
    MOVE_BY = 'move_by'
    MOVE_TO = 'move_to'
    THREADED_MOVE_BY = 'threaded_move_by'
    THREADED_MOVE_TO = 'threaded_move_to'
    JOG = 'jog'
        
class Motor:
    def __init__(
            self,
            serial_number: str
    ) -> None:
        self.device_info = DeviceInfo()
        self.is_moving = False
        self.direction = MotorDirection.IDLE
        self.position = 0.0
        self.step_size = 0.0
        self.acceleration = 0.0
        self.max_velocity = 0.0
        
        self._lock = threading.Lock()
        self._position_polling = 1

    def move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        return True

    def move_to(
            self,
            position: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        return True
    
    def threaded_move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
        pass

    def threaded_move_to(
            self,
            position: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
        pass

    def jog(
            self,
            direction: MotorDirection,
            acceleration: float,
            max_velocity: float
    ) -> None:
        pass

    def stop(self) -> None:
        pass