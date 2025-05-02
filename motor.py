import pathlib
import platform
import threading
import time
import math
import enum

import pylablib.devices.Thorlabs

def list_thorlabs_motors() -> list[pylablib.devices.Thorlabs.KinesisMotor]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_thorlabs_motors_linux()
        case 'Windows':
            return list_thorlabs_motors_windows()
        case _:
            raise NotImplementedError(f'Unsupported system: {system}')

def list_thorlabs_motors_linux() -> list[pylablib.devices.Thorlabs.KinesisMotor]:
    device_path = pathlib.Path('/dev/serial/by-id')
    if not device_path.exists():
        return []
    
    motors = []
    for symlink in device_path.iterdir():
        if 'Thorlabs' in symlink.name:
            try:
                motors.append(
                    pylablib.devices.Thorlabs.KinesisMotor(
                        conn=str(symlink.resolve()),
                        scale='stage'
                    )
                )
            except IndexError:
                continue

    return motors

def list_thorlabs_motors_windows() -> list[pylablib.devices.Thorlabs.KinesisMotor]:
    motors = []
    for device in pylablib.devices.Thorlabs.list_kinesis_devices():
        try:
            motors.append(
                pylablib.devices.Thorlabs.KinesisMotor(
                    conn=device[0],
                    scale='stage'
                )
            )
        except IndexError:
            continue

    return motors

class Motor:
    class MotorDirection(enum.Enum):
        FORWARD = '+'
        BACKWARD = '-'
        IDLE = None

    def __init__(
            self,
            serial_number: str
    ):
        self.serial_no = serial_number
        self._motor = self._get_motor()
        if self._motor == None:
            raise NotImplementedError(f'Unable to find motor: {serial_number}')
        
        self.position = self._motor.get_position()
        self.motor_thread = None
        self.motor_direction = self.MotorDirection.IDLE
        self.acceleration: float = 5
        self.max_velocity: float = 5

        
    def move_by(
            self,
            angle: float,
            acceleration: float = 20,
            max_velocity: float = 25
    ) -> bool:
        self._motor.setup_velocity(
            acceleration=acceleration,
            max_velocity=max_velocity,
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
            t = threading.Thread(
                target=time.sleep,
                args=(sleep_time,)
            )
            t.start()
            while t.is_alive():
                self.position = self._motor.get_position()
                time.sleep(0.01)

            self.position = self._motor.get_position()
        return True

    def move_to(
            self,
            position: float,
            acceleration: float = 20,
            max_velocity: float = 25
    ) -> bool:
        self._motor.setup_velocity(
            acceleration=acceleration,
            max_velocity=max_velocity,
            scale=True
        )
        # move_to doesn't seem to take shortest route
        # angle_diff = abs((position - self._motor.get_position() + 180) % 360 - 180)
        angle_diff = abs(self._motor.get_position() - position)
        sleep_time = self._rotation_time(
            angle=angle_diff,
            acceleration=acceleration,
            max_velocity=max_velocity,
            degrees=True
        )
        try:
            self._motor.move_to(position=position)
        except Exception as e:
            print('Exception occured when moving motor: {e}')
        else:
            t = threading.Thread(
                target=time.sleep,
                args=(sleep_time,)
            )
            t.start()
            while t.is_alive():
                self.position = self._motor.get_position()
                time.sleep(0.01)

            self.position = self._motor.get_position()
        return True
    
    def threaded_move_by(
            self,
            angle: float,
            acceleration: float = 20,
            max_velocity: float = 25
    ) -> None:
        if self.motor_thread is None or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(
                target=self.move_by,
                args=(
                    angle,
                    acceleration,
                    max_velocity
                )
            )
            self.motor_thread.start()
        else:
            print('Warning: Motor is busy')

    def threaded_move_to(
            self,
            position: float,
            acceleration: float = 20,
            max_velocity: float = 25
    ) -> None:
        if self.motor_thread is None or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(
                target=self.move_to,
                args=(
                    position,
                    acceleration,
                    max_velocity
                )
            )
            self.motor_thread.start()
        else:
            print('Warning: Motor is busy')

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
    
    def _get_motor(self) -> pylablib.devices.Thorlabs.KinesisMotor:
        motors = list_thorlabs_motors()
        return next((motor for motor in motors if str(motor.get_device_info().serial_no) == str(self.serial_no)), None)

if __name__ == '__main__':
    motor = Motor(serial_number=55353314)
    print(motor._motor.get_jog_parameters())
    time.sleep(1)

    print('Moving motor: 10')
    motor._motor.setup_jog(
        mode='continuous',
        max_velocity=10,
    )
    motor._motor.jog(
        direction='-',
        kind='builtin'
    )
    time.sleep(5)

    motor._motor.stop()
    time.sleep(1)

    print('Moving motor: 20')
    motor._motor.setup_jog(
        mode='continuous',
        max_velocity=20
    )
    print(motor._motor.get_jog_parameters())
    motor._motor.jog(
        direction='-',
        kind='builtin'
    )
    time.sleep(5)

    motor._motor.stop()