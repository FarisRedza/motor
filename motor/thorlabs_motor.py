import pathlib
import platform
import threading
import time
import math

import pylablib.devices.Thorlabs.kinesis

from . import base_motor

MAX_ACCELERATION = 20.0
MAX_VELOCITY = 25.0

# def list_thorlabs_motors(
#         inpsect: bool = False
# ) -> list[pylablib.devices.Thorlabs.kinesis.KinesisMotor]:
#     system = platform.system()
#     if system == 'Linux':
#         return list_thorlabs_motors_linux()
#     elif system == 'Windows':
#         return list_thorlabs_motors_windows()
#     else:
#         raise NotImplementedError(f'Unsupported system: {system}')

# def list_thorlabs_motors_linux() -> list[pylablib.devices.Thorlabs.kinesis.KinesisMotor]:
#     device_path = pathlib.Path('/dev/serial/by-id')
#     if not device_path.exists():
#         return []
#     motors = []
#     for symlink in device_path.iterdir():
#         if 'Thorlabs' in symlink.name:
#             try:
#                 motors.append(
#                     pylablib.devices.Thorlabs.kinesis.KinesisMotor(
#                         conn=str(symlink.resolve()),
#                         scale='stage'
#                     )
#                 )
#             except IndexError:
#                 continue
#     return motors

# def list_thorlabs_motors_windows(
#         inpsect: bool = False
# ) -> list[pylablib.devices.Thorlabs.kinesis.KinesisMotor]:
#     motors = []
#     for device in pylablib.devices.Thorlabs.kinesis.list_kinesis_devices():
#         try:
#             motor = pylablib.devices.Thorlabs.kinesis.KinesisMotor(
#                     conn=device[0],
#                     scale='stage'
#                 )
#             if inpsect == True:
#                 motor.close()
#             motors.append(motor)
#         except IndexError:
#             continue
#     return motors

def list_thorlabs_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_thorlabs_motors_linux()

        case 'Windows':
            return list_thorlabs_motors_windows()

        case _:
            raise NotImplementedError(f'Unsupported system: {system}')
        
def list_thorlabs_motors_linux() -> list[tuple[str, str]]:
    device_path = pathlib.Path('/dev/serial/by-id')
    if not device_path.exists():
        return []

    motors: list[tuple[str, str]] = []
    for symlink in device_path.iterdir():
        if 'Thorlabs' in symlink.name:
            try:
                parts = symlink.name.removeprefix('usb-Thorlabs_').split('-if')[0].split('_')
                serial_number = parts[-1]
                device_name = ' '.join(parts[:-1])
                motors.append(
                    (serial_number, device_name)
                )
            except IndexError:
                continue
    return motors

def list_thorlabs_motors_windows() -> list[tuple[str, str]]:
    motors = []
    for device in pylablib.devices.Thorlabs.kinesis.list_kinesis_devices():
        if device[1] == 'Kinesis K10CR1 Rotary Stage':
            motors.append(device)
    return motors

class Motor(base_motor.Motor):
    def __init__(
            self,
            serial_number: str | None = None,
            motor: pylablib.devices.Thorlabs.kinesis.KinesisMotor | None = None
        ) -> None:
        if serial_number:
            self._get_motor(serial_number=serial_number)
        elif motor:
            self._motor = motor
        else:
            raise RuntimeError('Must provide a serial number or KinesisMotor')

        device_info = self._motor.get_device_info()
        self.device_info = base_motor.DeviceInfo(
            device_name=device_info.notes,
            model=device_info.model_no,
            serial_number=str(device_info.serial_no),
            firmware_version=device_info.fw_ver
        )

        self.position = self._motor.get_position()
        self.direction = base_motor.MotorDirection.IDLE
        self.is_moving = self._motor.is_moving()
        self.step_size = 5.0
        self.acceleration = MAX_ACCELERATION
        self.max_velocity = MAX_VELOCITY

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._position_thread: threading.Thread | None = None
        self._movement_thread: threading.Thread | None = None
        self._position_polling = 0.1

    def stop(self):
        with self._lock:
            self._motor.stop()
            self.direction = base_motor.MotorDirection.IDLE
        self._stop_tracking_position()

    def move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        self.stop()
        with self._lock:
            self.acceleration = acceleration
            self.max_velocity = max_velocity
            self._motor.setup_velocity(
                acceleration=self.acceleration,
                max_velocity=self.max_velocity,
                scale=True
            )
        self._start_tracking_position()
        self.direction = base_motor.MotorDirection.FORWARD if angle > 0 else base_motor.MotorDirection.BACKWARD
        try:
            with self._lock:
                self._motor.move_by(distance=angle)
        except Exception as e:
            print(f'Exception in move_by: {e}')
            return False
        time.sleep(self._rotation_time(angle))
        self._stop_tracking_position()
        with self._lock:
            self.position = self._motor.get_position()
        return True

    def move_to(
            self,
            position: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        self.stop()
        with self._lock:
            current_position = self._motor.get_position()
        return self.move_by(
            angle=position - current_position,
            acceleration=acceleration,
            max_velocity=max_velocity
        )

    def jog(
            self,
            direction: base_motor.MotorDirection,
            acceleration: float,
            max_velocity: float
    ) -> None:
        self.stop()
        with self._lock:
            self.acceleration = acceleration
            self.max_velocity = max_velocity
            self._motor.setup_jog(
                mode='continuous',
                acceleration=self.acceleration,
                max_velocity=self.max_velocity
            )
            self.direction = direction
            self._motor.jog(
                direction=self.direction.value,
                kind='builtin'
            )
        self._start_tracking_position()

    def threaded_move_by(
            self, angle: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
        if self._movement_thread and self._movement_thread.is_alive():
            return
        self._movement_thread = threading.Thread(
            target=self.move_by,
            args=(angle, acceleration, max_velocity),
            daemon=True
        )
        self._movement_thread.start()

    def threaded_move_to(
            self,
            position: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
        if self._movement_thread and self._movement_thread.is_alive():
            return
        self._movement_thread = threading.Thread(
            target=self.move_to,
            args=(position, acceleration, max_velocity),
            daemon=True
        )
        self._movement_thread.start()

    def _get_motor(self, serial_number: str):
        system = platform.system()
        match system:
            case 'Linux':
                symlink = next(
                    (s for s in pathlib.Path('/dev/serial/by-id').iterdir() if serial_number in s.name),
                    None
                )
                if symlink is not None:
                    self._motor = pylablib.devices.Thorlabs.kinesis.KinesisMotor(
                        conn=str(symlink.resolve()),
                        scale='stage'
                    )
                else:
                    raise RuntimeError(f'Motor {serial_number} not found')

            case 'Windows':
                device = next(
                    (s for s in list_thorlabs_motors_windows() if serial_number in s[0]),
                    None
                )
                if device is not None:
                    self._motor = pylablib.devices.Thorlabs.kinesis.KinesisMotor(
                        conn=device[0],
                        scale='stage'
                    )
                else:
                    raise RuntimeError(f'Motor {serial_number} not found')

            case _:
                raise NotImplementedError(f'Unsupported system: {system}')

    def _track_position(self):
        while not self._stop_event.is_set():
            with self._lock:
                try:
                    self.position = self._motor.get_position()
                    self.is_moving = self._motor.is_moving()
                except Exception as e:
                    print(f'[tracking error] {e}')
            time.sleep(self._position_polling)

    def _start_tracking_position(self):
        if self._position_thread and self._position_thread.is_alive():
            return
        self._stop_event.clear()
        self._position_thread = threading.Thread(
            target=self._track_position,
            daemon=True
        )
        self._position_thread.start()

    def _stop_tracking_position(self):
        self._stop_event.set()
        if self._position_thread:
            self._position_thread.join()
        self._position_thread = None
        self.direction = base_motor.MotorDirection.IDLE
        self.is_moving = False

    def _rotation_time(self, angle: float) -> float:
        angle = math.radians(abs(angle))
        acceleration = math.radians(self.acceleration)
        max_velocity = math.radians(self.max_velocity)

        t_accel = max_velocity / acceleration
        angle_accel = 0.5 * acceleration * t_accel ** 2
        angle_cruise = max(angle - 2 * angle_accel, 0)

        if angle_cruise <= 0:
            t_accel = math.sqrt(angle / acceleration)
            return 2 * t_accel
        return 2 * t_accel + angle_cruise / max_velocity

def get_all_motors() -> list[Motor]:
    motors = []
    for m in list_thorlabs_motors():
        try:
            motor = Motor(serial_number=m[0])
        except:
            continue
        else:
            motors.append(motor)
    return motors

if __name__ == '__main__':
    for motor in get_all_motors():
        motor.jog(
            direction=base_motor.MotorDirection.FORWARD,
            acceleration=20.0,
            max_velocity=20.0
        )
        time.sleep(5)
        motor.stop()