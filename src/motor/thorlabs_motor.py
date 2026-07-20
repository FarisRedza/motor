import pathlib
import platform
import threading
import typing
from collections.abc import Callable

try:
    import pylablib.devices.Thorlabs.kinesis as kinesis
except ModuleNotFoundError:
    kinesis = None

from motor import base_motor

PositionCallback = Callable[[float], None]


def is_available() -> bool:
    return kinesis is not None


def list_kinesis_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_kinesis_motors_linux()

        case 'Windows':
            raise NotImplementedError(
                'Windows support not implemented yet'
            )

        case _:
            raise ValueError(f'{system} is not supported')


def list_kinesis_motors_linux() -> list[tuple[str, str]]:
    serial_directory = pathlib.Path(
        '/dev/serial/by-id'
    )

    if not serial_directory.exists():
        # raise RuntimeError(
        #     f'{serial_directory} does not exist'
        # )
        return []

    motors: list[tuple[str, str]] = []
    for device_id in serial_directory.iterdir():
        if 'Thorlabs' not in device_id.name or 'K10CR2' in device_id.name:
            continue

        parts = device_id.name.removeprefix('usb-Thorlabs_').split('-if')[0].split('_')
        serial_number = parts[-1]
        device_name = ' '.join(parts[:-1])
        motors.append(
            (serial_number, device_name)
        )
    return motors


class ThorlabsMotor(base_motor.Motor):
    def __init__(
        self,
        serial_number: str,
        *,
        tracking_interval: float = 0.05,
        start_tracking: bool = True,
    ) -> None:
        if tracking_interval <= 0:
            raise ValueError('tracking_interval must be positive')

        self._tracking_interval = tracking_interval

        self._state_lock = threading.Lock()

        self._motor_lock = threading.Lock()

        self._tracking_stop_event = threading.Event()
        self._tracking_thread: typing.Optional[threading.Thread] = None

        self._position = 0.0
        self._is_moving = False
        self._tracking_error: typing.Optional[Exception] = None

        self._position_callbacks: list[PositionCallback] = []

        self._connect(serial_number=serial_number)

        self.device_info = self._get_device_info()
        self._position = self._read_position()
        self._is_moving = self._read_is_moving()
        self.acceleration = 20.0
        self.max_velocity = 25.0

        if start_tracking:
            self.start_tracking()

    @property
    def position(self) -> float:
        """Return the most recently measured position in degrees.

        This returns immediately and does not communicate with the motor.
        """
        with self._state_lock:
            return self._position

    @property
    def is_moving(self) -> bool:
        """Return the most recently measured movement state."""
        with self._state_lock:
            return self._is_moving

    @property
    def is_tracking(self) -> bool:
        thread = self._tracking_thread
        return thread is not None and thread.is_alive()

    @property
    def tracking_error(self) -> typing.Optional[Exception]:
        """Return the most recent tracking error, if one occurred."""
        with self._state_lock:
            return self._tracking_error

    def add_position_callback(
        self,
        callback: PositionCallback,
    ) -> None:
        """Register a callback invoked after each position update.

        The callback runs in the tracking thread. A GUI must normally
        forward the update to its main UI thread.
        """
        with self._state_lock:
            if callback not in self._position_callbacks:
                self._position_callbacks.append(callback)

    def remove_position_callback(
        self,
        callback: PositionCallback,
    ) -> None:
        with self._state_lock:
            try:
                self._position_callbacks.remove(callback)
            except ValueError:
                pass

    def start_tracking(self) -> None:
        """Start continuously refreshing position and movement state."""
        if self.is_tracking:
            return

        self._tracking_stop_event.clear()

        self._tracking_thread = threading.Thread(
            target=self._tracking_loop,
            name='standa-position-tracker',
            daemon=True,
        )
        self._tracking_thread.start()

    def stop_tracking(self) -> None:
        """Stop continuous position tracking."""
        thread = self._tracking_thread

        if thread is None:
            return

        self._tracking_stop_event.set()

        # Avoid trying to join the current thread if disconnect() was
        # somehow called from a callback.
        if thread is not threading.current_thread():
            thread.join(
                timeout=max(1.0, self._tracking_interval * 4)
            )

        self._tracking_thread = None

    def refresh_position(self) -> float:
        """Synchronously read and cache the current motor position."""
        position = self._read_position()
        moving = self._read_is_moving()

        self._update_cached_state(
            position=position,
            moving=moving,
        )

        return position

    def move_by(
        self,
        angle: float,
        acceleration: typing.Optional[float] = None,
        max_velocity: typing.Optional[float] = None,
    ) -> None:
        """Begin a relative movement by an angle in degrees."""
        requested_acceleration = (
            self.acceleration
            if acceleration is None
            else acceleration
        )
        requested_max_velocity = (
            self.max_velocity
            if max_velocity is None
            else max_velocity
        )

        if acceleration is not None or max_velocity is not None:
            self.update_settings(
                acceleration=requested_acceleration,
                max_velocity=requested_max_velocity,
            )

        self._step(angle=angle)

        with self._state_lock:
            self._is_moving = True

    def move_to(
        self,
        position: float,
        acceleration: typing.Optional[float] = None,
        max_velocity: typing.Optional[float] = None,
    ) -> None:
        """Move to an absolute angular position in degrees."""
        requested_acceleration = (
            self.acceleration
            if acceleration is None
            else acceleration
        )
        requested_max_velocity = (
            self.max_velocity
            if max_velocity is None
            else max_velocity
        )

        if acceleration is not None or max_velocity is not None:
            self.update_settings(
                acceleration=requested_acceleration,
                max_velocity=requested_max_velocity,
            )

        with self._motor_lock:
            self._motor.move_to(position=position)

        with self._state_lock:
            self._is_moving = True

    def jog(
            self,
            direction: base_motor.MotorDirection,
            acceleration: typing.Optional[float] = None,
            max_velocity: typing.Optional[float] = None
    ) -> None:
        requested_acceleration = (
            self.acceleration
            if acceleration is None
            else acceleration
        )
        requested_max_velocity = (
            self.max_velocity
            if max_velocity is None
            else max_velocity
        )

        # if acceleration is not None or max_velocity is not None:
        self.update_settings(
            acceleration=requested_acceleration,
            max_velocity=requested_max_velocity,
        )

        with self._motor_lock:
            self._motor.jog(
                direction=direction.value,
                kind='builtin'
            )
        
        with self._state_lock:
            self._is_moving = True

    def stop(self) -> None:
        """Stop the current movement using normal deceleration."""
        with self._motor_lock:
            self._motor.stop()

    def disconnect(self) -> None:
        """Stop tracking and close the motor connection."""
        self.stop_tracking()

        with self._motor_lock:
            self._motor.close()

    def __enter__(self) -> 'ThorlabsMotor':
        return self

    def __exit__(
        self,
        exc_type: object,
        exc_value: object,
        traceback: object,
    ) -> None:
        self.disconnect()

    def _get_device_info(self) -> base_motor.DeviceInfo:
        dev_info = self._motor.get_device_info()

        return base_motor.DeviceInfo(
            device_name=dev_info.notes,
            model=dev_info.model_no,
            serial_number=f'{dev_info.serial_no}',
            firmware_version=dev_info.fw_ver
        )

    def _tracking_loop(self) -> None:
        while not self._tracking_stop_event.is_set():
            try:
                position, moving = self._read_motor_state()

                self._update_cached_state(
                    position=position,
                    moving=moving,
                )

                with self._state_lock:
                    self._tracking_error = None
                    callbacks = tuple(self._position_callbacks)

                for callback in callbacks:
                    try:
                        callback(position)
                    except Exception:
                        pass

            except Exception as error:
                with self._state_lock:
                    self._tracking_error = error

            self._tracking_stop_event.wait(
                self._tracking_interval
            )

    def _update_cached_state(
        self,
        *,
        position: float,
        moving: bool,
    ) -> None:
        with self._state_lock:
            self._position = position
            self._is_moving = moving

    def _read_motor_state(self) -> tuple[float, bool]:
        """Read position and movement state under one controller lock."""
        with self._motor_lock:
            position = self._motor.get_position()
            moving = self._motor.is_moving()

        return position, moving

    def _connect(self, serial_number: str) -> None:
        system = platform.system()
        motor: typing.Optional[kinesis.KinesisMotor] = None

        match system:
            case 'Linux':
                serial_directory = pathlib.Path(
                    '/dev/serial/by-id'
                )

                if not serial_directory.exists():
                    raise RuntimeError(
                        f'{serial_directory} does not exist'
                    )

                for device_id in serial_directory.iterdir():
                    if 'Thorlabs' not in device_id.name or 'K10CR2' in device_id.name:
                        continue

                    port = device_id

                    parts = port.name.removeprefix('usb-Thorlabs_').split('-if')[0].split('_')
                    candidate_serial = parts[-1]

                    if candidate_serial == serial_number:
                        motor = kinesis.KinesisMotor(
                            conn=str(port.resolve()),
                            scale='stage'
                        )
                        break

            case 'Windows':
                raise NotImplementedError(
                    'Windows support not implemented yet'
                )

            case _:
                raise ValueError(f'{system} is not supported')

        if motor is None:
            raise RuntimeError(
                'Could not find motor with serial number '
                f'{serial_number}'
            )

        self._motor = motor

    def _read_position(self) -> float:
        with self._motor_lock:
            pos = self._motor.get_position()

        return pos

    def _read_is_moving(self) -> bool:
        with self._motor_lock:
            moving = self._motor.is_moving()

        return moving

    def _read_acceleration(self) -> float:
        return self.acceleration

    def _read_max_velocity(self) -> float:
        return self.max_velocity

    def update_settings(
        self,
        acceleration: float,
        max_velocity: float,
    ) -> None:
        if acceleration <= 0:
            raise ValueError('acceleration must be positive')

        if max_velocity <= 0:
            raise ValueError('max_velocity must be positive')

        with self._motor_lock:
            self._motor.setup_jog(
                mode='continuous',
                acceleration=acceleration,
                max_velocity=max_velocity
            )

        self.acceleration = acceleration
        self.max_velocity = max_velocity

    def _step(
        self,
        angle: int
    ) -> None:
        with self._motor_lock:
            self._motor.move_by(distance=angle)


if __name__ == '__main__':
    try:
        with ThorlabsMotor(
            serial_number='55353314',
            tracking_interval=0.05,
        ) as motor:
            print(f'Starting position: {motor.position:.3f}°')

            # motor.move_by(angle=90)
            # motor.move_to(position=90)
            motor.jog(
                direction=base_motor.MotorDirection.FORWARD
            )

            # while motor.is_moving:
            for _ in range(10):
                print(
                    f'\rPosition: {motor.position:8.3f}°',
                    end='',
                    flush=True,
                )
                threading.Event().wait(0.5)
            motor.stop()

            print(f'\nFinal position: {motor.position:.3f}°')

    except KeyboardInterrupt:
        print('\nInterrupted')