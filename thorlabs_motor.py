import pathlib
import platform
import threading
import time
import math

import pylablib.devices.Thorlabs

import motor.base_motor as base_motor

MAX_ACCELERATION = 20.0
MAX_VELOCITY = 25.0

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

class Motor(base_motor.Motor):
    def __init__(
            self,
            serial_number: str
    ) -> None:
        self._motor = self._get_motor(serial_number=serial_number)
        device_info = self._motor.get_device_info()
        self.device_info = base_motor.DeviceInfo(
            device_name=device_info.notes,
            model=device_info.model_no,
            serial_number=str(device_info.serial_no),
            firmware_version=device_info.fw_ver
        )
        self.is_moving = self._motor.is_moving()
        self.direction = base_motor.MotorDirection.IDLE
        self.position: float = self._motor.get_position()
        self.step_size = 5.0
        self.acceleration = MAX_ACCELERATION
        self.max_velocity = MAX_VELOCITY

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._position_thread: threading.Thread | None = None
        self._position_polling = 0.1
        self._movement_thread: threading.Thread | None = None

    def _get_motor(
            self,
            serial_number: str
    ) -> pylablib.devices.Thorlabs.KinesisMotor:
        motors = list_thorlabs_motors()
        _motor = next(
            (m for m in motors if str(m.get_device_info().serial_no) == serial_number),
            None
        )
        if _motor == None:
            raise RuntimeError(f'Motor {serial_number} not found')
        return _motor

    def _track_position(self) -> None:
        while True:
            with self._lock:
                try:
                    self.position = self._motor.get_position()
                    moving = self._motor.is_moving()
                except Exception:
                    moving = False

            if self._stop_event.is_set() and not moving:
                break

            time.sleep(self._position_polling)

    def _start_tracking_position(self) -> None:
        self._stop_event.clear()
        self._position_thread = threading.Thread(
            target=self._track_position
        )
        self._position_thread.start()

    def _stop_tracking_position(self) -> None:
        self._stop_event.set()
        if self._position_thread and self._position_thread.is_alive():
            self._position_thread.join()
        print("stop tracking")

    def _rotation_time(
            self,
            angle: float,
    ) -> float:
        angle = math.radians(abs(angle))
        acceleration = math.radians(self.acceleration)
        max_velocity = math.radians(self.max_velocity)

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
    
    def stop(self) -> None:
        # with self._lock:
        self._motor.stop()
        self.direction = base_motor.MotorDirection.IDLE
        self._stop_tracking_position()

    def move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        self._start_tracking_position()
        if angle > 0:
            self.direction = base_motor.MotorDirection.FORWARD
        elif angle < 0:
            self.direction = base_motor.MotorDirection.BACKWARD
        else:
            self.direction = base_motor.MotorDirection.IDLE

        if self.acceleration != acceleration or self.max_velocity != max_velocity:
            with self._lock:
                self.acceleration = acceleration
                self.max_velocity = max_velocity
                self._motor.setup_velocity(
                    acceleration=self.acceleration,
                    max_velocity=self.max_velocity,
                    scale=True
                )

        sleep_time = self._rotation_time(angle=angle)
        try:
            with self._lock:
                self._motor.move_by(distance=angle)
        except Exception as e:
            print(f'Exception occured when moving motor: {e}')
        else:
            time.sleep(sleep_time)
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
        _, r = math.modf(self.position)
        angle = (r*self.position) - position
        self._start_tracking_position()
        if angle > 0:
            self.direction = base_motor.MotorDirection.FORWARD
        elif angle < 0:
            self.direction = base_motor.MotorDirection.BACKWARD
        else:
            self.direction = base_motor.MotorDirection.IDLE

        if self.acceleration != acceleration or self.max_velocity != max_velocity:
            with self._lock:
                self.acceleration = acceleration
                self.max_velocity = max_velocity
                self._motor.setup_velocity(
                    acceleration=self.acceleration,
                    max_velocity=self.max_velocity,
                    scale=True
                )

        sleep_time = self._rotation_time(angle=angle)
        try:
            with self._lock:
                self._motor.move_to(position=position)
        except Exception as e:
            print(f'Exception occured when moving motor: {e}')
        else:
            time.sleep(sleep_time)
        self._stop_tracking_position()
        with self._lock:
            self.position = self._motor.get_position()
        return True
    
    
    def threaded_move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
        # if self._movement_thread is None or not self._movement_thread.is_alive():
        #     self.stop()
        self._movement_thread = threading.Thread(
            target=self.move_by,
            args=(
                angle,
                acceleration,
                max_velocity
            )
        )
        self._movement_thread.start()

    def threaded_move_to(
            self,
            position: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
        # if self._movement_thread is None or not self._movement_thread.is_alive():
        #     self.stop()
        self._movement_thread = threading.Thread(
            target=self.move_to,
            args=(
                position,
                acceleration,
                max_velocity
            )
        )
        self._movement_thread.start()

    def jog(
            self,
            direction: base_motor.MotorDirection,
            acceleration: float,
            max_velocity: float
    ) -> None:
        # with self._lock:
        if self.acceleration != acceleration or self.max_velocity != max_velocity:
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
            kind='continuous'
        )

def list_motors() -> list[Motor]:
    thorlabs_motors = list_thorlabs_motors()
    motors = []
    for m in thorlabs_motors:
        sn = str(m.get_device_info().serial_no)
        motor = Motor(
            serial_number=sn
        )
        motors.append(motor)
    return motors

if __name__ == '__main__':
    for motor in list_motors():
        motor.jog(
            direction=base_motor.MotorDirection.FORWARD,
            acceleration=20.0,
            max_velocity=20.0
        )
        time.sleep(5)
        motor.stop()