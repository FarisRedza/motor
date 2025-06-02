import pathlib
import platform
import threading
import time
import math
import enum
import dataclasses

import pylablib.devices.Thorlabs

MAX_ACCELERATION = 20.0
MAX_VELOCITY = 25.0

@dataclasses.dataclass
class DeviceInfo:
    device_name: str
    model: str
    serial_number: str
    firmware_version: str

class MotorDirection(enum.Enum):
    FORWARD = '-'
    BACKWARD = '+'
    IDLE = None

def list_motors() -> list[DeviceInfo]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_thorlabs_motors_linux()
        case 'Windows':
            return list_thorlabs_motors_windows()
        case _:
            raise NotImplementedError(f'Unsupported system: {system}')

def list_thorlabs_motors_linux() -> list[DeviceInfo]:
    device_path = pathlib.Path('/dev/serial/by-id')
    if not device_path.exists():
        return []
    
    motors = []
    for symlink in device_path.iterdir():
        if 'Thorlabs' in symlink.name:
            try:
                device_info = pylablib.devices.Thorlabs.KinesisMotor(
                    conn=str(symlink.resolve())
                ).get_device_info()
                motors.append(DeviceInfo(
                    device_name=device_info.notes,
                    model=device_info.model_no,
                    serial_number=str(device_info.serial_no),
                    firmware_version=device_info.fw_ver
                ))
            except IndexError:
                continue

    return motors

def list_thorlabs_motors_windows() -> list[DeviceInfo]:
    motors = []
    for device in pylablib.devices.Thorlabs.list_kinesis_devices():
        try:
            device_info = pylablib.devices.Thorlabs.KinesisMotor(
                conn=device[0],
            ).get_device_info()
            motors.append(DeviceInfo(
                device_name=device_info.notes,
                model=device_info.model_no,
                serial_number=str(device_info.serial_no),
                firmware_version=device_info.fw_ver
            ))
        except IndexError:
            continue

    return motors

class Motor:
    def __init__(
            self,
            serial_number: str
    ) -> None:
        self.serial_no = serial_number
        self._motor = self._get_motor(serial_number=serial_number)
        if self._motor == None:
            raise NotImplementedError(f'Unable to find motor: {serial_number}')

        device_info = self._motor.get_device_info()
        self.device_info = DeviceInfo(
            device_name=device_info.notes,
            model=device_info.model_no,
            serial_number=str(device_info.serial_no),
            firmware_version=device_info.fw_ver
        )
        self._lock = threading.Lock()
        self._position_thread: threading.Thread | None = None
        self._movement_thread: threading.Thread | None = None
        self.is_moving: bool = False
        self.direction = MotorDirection.IDLE
        self.step_size: float = 5
        self.acceleration: float = MAX_ACCELERATION
        self.max_velocity: float = MAX_VELOCITY
        self._position_polling = 0.1
        self.position = self._motor.get_position()

    def _get_motor(
            self,
            serial_number: str
    ) -> pylablib.devices.Thorlabs.KinesisMotor:
        device_path = pathlib.Path('/dev/serial/by-id')
        if not device_path.exists():
            raise RuntimeError(f'{str(device_path)} does not exist. Are any devices plugged in?')
        
        for symlink in device_path.iterdir():
            if 'Thorlabs' in symlink.name:
                try:
                    motor = pylablib.devices.Thorlabs.KinesisMotor(
                        conn=str(symlink.resolve())
                    )
                    if str(motor.get_device_info().serial_no) == serial_number:
                        return motor
                except Exception:
                    continue
        raise RuntimeError(f'Motor {serial_number} not found')

    # def _get_motor_position(self):
    #     with self._lock:
    #         self.position = self._motor.get_position()
        
    def _track_position(self):
        while self.is_moving == True:
            time.sleep(self._position_polling)
            with self._lock:
                self.position = self._motor.get_position()

    def _start_tracking_position(self):
        self.is_moving = True
        self._position_thread = threading.Thread(
            target=self._track_position
        )
        self._position_thread.start()

    def _stop_tracking_position(self):
        self.is_moving = False
        if self._position_thread:
            self._position_thread.join()

    def move_by(
            self,
            angle: float,
            acceleration: float = MAX_ACCELERATION,
            max_velocity: float = MAX_VELOCITY
    ) -> bool:
        self._start_tracking_position()

        if angle > 0:
            self.direction = MotorDirection.FORWARD
        elif angle < 0:
            self.direction = MotorDirection.BACKWARD
        else:
            self.direction = MotorDirection.IDLE

        if acceleration != self.acceleration or max_velocity != self.max_velocity:
            self.acceleration = acceleration
            self.max_velocity = max_velocity
            self._motor.setup_velocity(
                acceleration=self.acceleration,
                max_velocity=self.max_velocity,
                scale=True
            )

        sleep_time = self._rotation_time(
            angle=angle,
            acceleration=acceleration,
            max_velocity=max_velocity,
            degrees=True
        )

        try:
            self._motor.move_by(distance=angle)
        except Exception as e:
            print('Exception occured when moving motor: {e}')
        else:
            time.sleep(sleep_time)
        self._stop_tracking_position()
        return True

    def move_to(
            self,
            position: float,
            acceleration: float = MAX_ACCELERATION,
            max_velocity: float = MAX_VELOCITY
    ) -> bool:
        with self._lock:
            self._start_tracking_position()
            # move_to doesn't seem to take shortest route
            # angle = abs((position - self._motor.get_position() + 180) % 360 - 180)
            # angle = self._motor.get_position() - position
            angle = self.position - position

            if angle > 0:
                self.direction = MotorDirection.FORWARD
            elif angle < 0:
                self.direction = MotorDirection.BACKWARD
            else:
                self.direction = MotorDirection.IDLE

            if acceleration != self.acceleration or max_velocity != self.max_velocity:
                self.acceleration = acceleration
                self.max_velocity = max_velocity
                self._motor.setup_velocity(
                    acceleration=self.acceleration,
                    max_velocity=self.max_velocity,
                    scale=True
                )
            sleep_time = self._rotation_time(
                angle=abs(angle),
                acceleration=acceleration,
                max_velocity=max_velocity,
                degrees=True
            )
            try:
                self._motor.move_to(position=position)
            except Exception as e:
                print('Exception occured when moving motor: {e}')
            else:
                time.sleep(sleep_time)
            self._stop_tracking_position()
        return True
    
    def threaded_move_by(
            self,
            angle: float,
            acceleration: float = MAX_ACCELERATION,
            max_velocity: float = MAX_VELOCITY
    ) -> None:
        if self._movement_thread is None or not self._movement_thread.is_alive():
            self._movement_thread = threading.Thread(
                target=self.move_by,
                args=(
                    angle,
                    acceleration,
                    max_velocity
                )
            )
            self._movement_thread.start()
        else:
            print('Warning: Motor is busy')

    def threaded_move_to(
            self,
            position: float,
            acceleration: float = MAX_ACCELERATION,
            max_velocity: float = MAX_VELOCITY
    ) -> None:
        if self._movement_thread is None or not self._movement_thread.is_alive():
            self._movement_thread = threading.Thread(
                target=self.move_to,
                args=(
                    position,
                    acceleration,
                    max_velocity
                )
            )
            self._movement_thread.start()
        else:
            print('Warning: Motor is busy')

    def jog(
            self,
            direction: MotorDirection,
            acceleration: float = MAX_ACCELERATION,
            max_velocity: float = MAX_VELOCITY
    ) -> None:
        with self._lock:
            if acceleration != self.acceleration or max_velocity != self.max_velocity:
                self.acceleration = acceleration
                self.max_velocity = max_velocity
                self._motor.setup_jog(
                    mode='continuous',
                    acceleration=self.acceleration,
                    max_velocity=self.max_velocity
                )
            if direction != self.direction:
                self.direction = direction
            self._start_tracking_position()
            self._motor.jog(
                direction=self.direction.value,
                kind='builtin'
            )

    def stop(self) -> None:
        with self._lock:
            self._motor.stop()
            self.direction = MotorDirection.IDLE
            self._stop_tracking_position()

    def _rotation_time(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float,
            degrees=False
    ) -> float:
        if degrees:
            angle = math.radians(angle)
            acceleration = math.radians(acceleration)
            max_velocity = math.radians(max_velocity)

        t_accel = max_velocity / acceleration
        # angle covered during acceleration/deceleration
        angle_accel = 0.5 * acceleration * t_accel**2
        angle_decel = angle_accel
        angle_cruise = angle - angle_accel - angle_decel

        if angle_cruise < 0:
            # the motor never reaches max velocity (triangular profile)
            t_accel = math.sqrt(abs(angle) / acceleration)
            total_time = 2 * t_accel
        else:
            # trapezoidal profile
            t_cruise = angle_cruise / max_velocity
            total_time = 2 * t_accel + t_cruise

        return total_time

def get_motors() -> list[Motor]:
    return [Motor(serial_number=info.serial_number) for info in list_motors()]

if __name__ == '__main__':
    motors = get_motors()
    # print(motors[0].is_moving)
    # motors[0].threaded_move_to(position=0)
    # for i in range(10):
    #     print(motors[0].is_moving)
    #     print(motors[0]._position_thread)
    #     time.sleep(1)

    # motors[0].threaded_move_to(position=-90)
    # for i in range(10):
    #     print(motors[0].is_moving)
    #     print(motors[0]._position_thread)
    #     time.sleep(1)

    for motor in motors:
        motor.stop()
        print(f'Motor {motor.serial_no}: {motor.position}')