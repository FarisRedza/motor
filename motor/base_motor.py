import enum
import dataclasses
import struct

@dataclasses.dataclass
class DeviceInfo:
    device_name: str = ''
    model: str = ''
    serial_number: str = ''
    firmware_version: str = ''

    def serialise(self) -> bytes:
        def encode_string(s: str):
            b = s.encode()
            return struct.pack(f'I{len(b)}s', len(b), b)

        return (
            encode_string(self.device_name) +
            encode_string(self.model) +
            encode_string(self.serial_number) +
            encode_string(self.firmware_version)
        )
    
    @classmethod
    def deserialise(cls, payload: bytes) -> 'DeviceInfo':
        offset = 0
        fields = []
        for _ in range(4):
            length = struct.unpack_from('I', payload, offset)[0]
            offset += 4
            value = struct.unpack_from(
                f'{length}s',
                payload,
                offset
            )[0].decode()
            offset += length
            fields.append(value)
        return DeviceInfo(*fields)

class MotorDirection(enum.Enum):
    FORWARD = '-'
    BACKWARD = '+'
    IDLE = ''

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