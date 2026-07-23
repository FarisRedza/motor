import enum
import math
import pathlib
import platform
import threading
import typing
from collections.abc import Callable

def _patch_libximc_python310() -> None:
    """Work around libximc rejecting unknown MoveFlags bits on Python < 3.11."""

    if hasattr(enum, "KEEP"):
        ximc.MoveFlags._boundary_ = enum.KEEP
        return

    from libximc.highlevel._structure_types import move_settings_t

    original = move_settings_t.MoveFlags

    def _set_move_flags(self, value):
        self._MoveFlags = value

    move_settings_t.MoveFlags = property(
        original.fget,
        _set_move_flags,
        original.fdel,
        original.__doc__,
    )

try:
    import libximc.highlevel as ximc
except ModuleNotFoundError:
    ximc = None
else:
    _patch_libximc_python310()

from motor import base_motor

USTEP_PER_STEP = 256
FULL_STEP = 28_800
MVCMD_RUNNING = 0x80
PositionCallback = Callable[[float], None]


def is_available() -> bool:
    return ximc is not None


def list_standa_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_standa_motors_linux()

        case 'Windows':
            raise NotImplementedError(
                'Windows support not implemented yet'
            )

        case _:
            raise ValueError(f'{system} is not supported')


def list_standa_motors_linux() -> list[tuple[str, str]]:
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
        if 'XIMC_Motor_Controller' not in device_id.name:
            continue

        port = device_id.resolve()
        motor = ximc.Axis(f'xi-com:{port}')
        motor.open_device()
        serial_number = str(motor.get_serial_number())
        device_name = motor.get_device_information().Manufacturer
        motor.close_device()
        motors.append(
            (serial_number, device_name)
        )
    return motors


class StandaMotor(base_motor.Motor):
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

        self.device_info = self._get_device_info(serial_number=serial_number)
        self._position = self._read_position()
        self._is_moving = self._read_is_moving()
        self.acceleration = self._read_acceleration()
        self.max_velocity = self._read_max_velocity()

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

        step, microstep = self._deg_to_step(angle=angle)
        self._step(step=step, microstep=microstep)

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

        step, microstep = self._deg_to_step(angle=position)

        with self._motor_lock:
            self._motor.command_move(
                position=step,
                uposition=microstep,
            )

        with self._state_lock:
            self._is_moving = True

    def stop(self) -> None:
        """Stop the current movement using normal deceleration."""
        with self._motor_lock:
            self._motor.command_sstp()

    def disconnect(self) -> None:
        """Stop tracking and close the motor connection."""
        self.stop_tracking()

        with self._motor_lock:
            self._motor.close_device()

    def __enter__(self) -> 'StandaMotor':
        return self

    def __exit__(
        self,
        exc_type: object,
        exc_value: object,
        traceback: object,
    ) -> None:
        self.disconnect()

    def _get_device_info(self, serial_number: str) -> base_motor.DeviceInfo:
        dev_info = self._motor.get_device_information()
        fw = self._motor.get_firmware_version()
        if len(fw) != 3:
            raise ValueError(f'Invalid firmware version:{fw}')

        return base_motor.DeviceInfo(
            device_name=f'{dev_info.ProductDescription}',
            model=f'{dev_info.Major}.{dev_info.Minor}.{dev_info.Release}',
            serial_number=serial_number,
            firmware_version=f'{fw[0]}.{fw[1]}.{fw[2]}'
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
            pos = self._motor.get_position()
            status = self._motor.get_status()

        position = self._step_to_deg(
            step=pos.Position,
            microstep=pos.uPosition,
        )
        moving = bool(
            int(status.MvCmdSts) & MVCMD_RUNNING
        )

        return position, moving

    def _connect(self, serial_number: str) -> None:
        system = platform.system()
        motor: typing.Optional[ximc.Axis] = None

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
                    if 'XIMC_Motor_Controller' not in device_id.name:
                        continue

                    port = device_id.resolve()

                    try:
                        candidate = ximc.Axis(
                            uri=f'xi-com:{port}'
                        )
                        candidate.open_device()
                        candidate_serial = str(
                            candidate.get_serial_number()
                        )
                    except Exception as error:
                        raise RuntimeError(
                            f'Unable to connect to port {port}: '
                            f'{error}'
                        ) from error

                    if candidate_serial == serial_number:
                        motor = candidate
                        break

                    candidate.close_device()

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

    def _deg_to_step(
        self,
        angle: float,
    ) -> tuple[int, int]:
        if not math.isfinite(angle):
            raise ValueError('angle must be finite')

        total_microsteps = round(
            angle
            * FULL_STEP
            * USTEP_PER_STEP
            / 360.0
        )

        steps = math.trunc(
            total_microsteps / USTEP_PER_STEP
        )
        microsteps = (
            total_microsteps
            - steps * USTEP_PER_STEP
        )

        return steps, microsteps

    def _step_to_deg(
        self,
        step: int,
        microstep: int,
    ) -> float:
        total_microsteps = (
            step * USTEP_PER_STEP
            + microstep
        )

        return (
            total_microsteps
            * 360.0
            / (
                FULL_STEP
                * USTEP_PER_STEP
            )
        )

    def _read_position(self) -> float:
        with self._motor_lock:
            pos = self._motor.get_position()

        return self._step_to_deg(
            step=pos.Position,
            microstep=pos.uPosition,
        )

    def _read_is_moving(self) -> bool:
        with self._motor_lock:
            status = self._motor.get_status()

        return bool(
            int(status.MvCmdSts) & MVCMD_RUNNING
        )

    def _read_acceleration(self) -> float:
        with self._motor_lock:
            settings = self._motor.get_move_settings()

        return self._step_to_deg(
            step=settings.Accel,
            microstep=0,
        )

    def _read_max_velocity(self) -> float:
        with self._motor_lock:
            settings = self._motor.get_move_settings()

        return self._step_to_deg(
            step=settings.Speed,
            microstep=settings.uSpeed,
        )

    def update_settings(
        self,
        acceleration: float,
        max_velocity: float,
    ) -> None:
        if acceleration <= 0:
            raise ValueError('acceleration must be positive')

        if max_velocity <= 0:
            raise ValueError('max_velocity must be positive')

        accel, uaccel = self._deg_to_step(
            angle=acceleration
        )
        speed, uspeed = self._deg_to_step(
            angle=max_velocity
        )

        total_accel = round(
            accel + uaccel / USTEP_PER_STEP
        )

        with self._motor_lock:
            settings = self._motor.get_move_settings()

            changed = (
                total_accel != settings.Accel
                or 2 * total_accel != settings.Decel
                or speed != settings.Speed
                or uspeed != settings.uSpeed
            )

            if changed:
                settings.Accel = total_accel
                settings.Decel = 2 * total_accel
                settings.Speed = speed
                settings.uSpeed = uspeed

                self._motor.set_move_settings(settings)

        self.acceleration = acceleration
        self.max_velocity = max_velocity

    def _step(
        self,
        step: int,
        microstep: int,
    ) -> None:
        with self._motor_lock:
            self._motor.command_movr(
                delta_position=step,
                udelta_position=microstep,
            )


if __name__ == '__main__':
    print(list_standa_motors())
    # try:
    #     with StandaMotor(
    #         serial_number='37398',
    #         tracking_interval=0.05,
    #     ) as motor:
    #         print(f'Starting position: {motor.position:.3f}°')

    #         # motor.move_by(angle=90)
    #         motor.move_to(position=90)

    #         while motor.is_moving:
    #             print(
    #                 f'\rPosition: {motor.position:8.3f}°',
    #                 end='',
    #                 flush=True,
    #             )
    #             threading.Event().wait(0.1)

    #         print(f'\nFinal position: {motor.position:.3f}°')

    # except KeyboardInterrupt:
    #     print('\nInterrupted')