import typing
import threading
import time
from collections.abc import Callable
import math

from motor.base_motor import Motor, DeviceInfo


PositionCallback = Callable[[float], None]


def list_dummy_motors() -> list[tuple[str, str]]:
    return [('dummy_motor_1', 'Dummy Motor')]


class DummyMotor(Motor):
    def __init__(
            self,
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

        self.acceleration = 20.0
        self.max_velocity = 25.0

        self._move_start_time = 0.0
        self._move_start_position = 0.0
        self._move_distance = 0.0
        self._move_acceleration = 0.0
        self._move_peak_velocity = 0.0
        self._move_accel_time = 0.0
        self._move_cruise_time = 0.0
        self._move_total_time = 0.0

        self.device_info = DeviceInfo(
            device_name='Dummy Device',
            model='Motor',
            serial_number='dummy_motor_1',
            firmware_version='0.0.0',
        )

        if start_tracking:
            self.start_tracking()

    @property
    def position(self) -> float:
        with self._state_lock:
            return self._position

    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            return self._is_moving

    @property
    def is_tracking(self) -> bool:
        thread = self._tracking_thread
        return thread is not None and thread.is_alive()

    @property
    def tracking_error(self) -> typing.Optional[Exception]:
        with self._state_lock:
            return self._tracking_error

    def move_by(
            self,
            angle: float,
            acceleration: typing.Optional[float] = None,
            max_velocity: typing.Optional[float] = None,
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

        if acceleration is not None or max_velocity is not None:
            self.update_settings(
                acceleration=requested_acceleration,
                max_velocity=requested_max_velocity,
            )

        if angle == 0:
            return

        distance = abs(angle)

        # time needed to accelerate to max velocity
        accel_time = self.max_velocity / self.acceleration

        # distance travelled during one acceleration phase
        accel_distance = (
            0.5
            * self.acceleration
            * accel_time ** 2
        )

        if 2 * accel_distance >= distance:
            # triangular profile, never reaches max velocity
            accel_time = math.sqrt(distance / self.acceleration)

            peak_velocity = self.acceleration * accel_time
            cruise_time = 0.0

            total_time = 2 * accel_time

        else:
            # trapezoidal profile, accelerate, cruise, decelerate
            peak_velocity = self.max_velocity

            cruise_distance = (
                distance
                - 2 * accel_distance
            )

            cruise_time = (
                cruise_distance
                / peak_velocity
            )

            total_time = (
                2 * accel_time
                + cruise_time
            )

        with self._motor_lock:
            # update an existing move before starting the new one
            self._update_motion_locked()

            self._move_start_time = time.monotonic()
            self._move_start_position = self._position
            self._move_distance = angle

            self._move_acceleration = self.acceleration
            self._move_peak_velocity = peak_velocity
            self._move_accel_time = accel_time
            self._move_cruise_time = cruise_time
            self._move_total_time = total_time

            self._is_moving = True

    def move_to(
            self,
            position: float,
            acceleration: typing.Optional[float] = None,
            max_velocity: typing.Optional[float] = None,
    ) -> None:
        """Begin an absolute movement to a position in degrees."""

        with self._motor_lock:
            self._update_motion_locked()
            current_position = self._position

        distance = position - current_position

        self.move_by(
            angle=distance,
            acceleration=acceleration,
            max_velocity=max_velocity,
        )

    def stop(self) -> None:
        with self._motor_lock:
            self._update_motion_locked()

            self._is_moving = False

    def disconnect(self) -> None:
        self.stop_tracking()

        with self._motor_lock:
            pass

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
            pass

        self.acceleration = acceleration
        self.max_velocity = max_velocity

    def _update_motion_locked(self) -> None:
        if not self._is_moving:
            return

        elapsed = (
            time.monotonic()
            - self._move_start_time
        )

        total_distance = abs(self._move_distance)

        direction = (
            1.0
            if self._move_distance >= 0
            else -1.0
        )

        acceleration = self._move_acceleration
        peak_velocity = self._move_peak_velocity

        accel_time = self._move_accel_time
        cruise_time = self._move_cruise_time

        if elapsed >= self._move_total_time:
            self._position = (
                self._move_start_position
                + self._move_distance
            )

            self._is_moving = False
            return

        if elapsed < accel_time:
            distance = (
                0.5
                * acceleration
                * elapsed ** 2
            )

        elif elapsed < accel_time + cruise_time:
            accel_distance = (
                0.5
                * acceleration
                * accel_time ** 2
            )

            cruise_elapsed = (
                elapsed
                - accel_time
            )

            distance = (
                accel_distance
                + peak_velocity * cruise_elapsed
            )

        else:
            remaining_time = (
                self._move_total_time
                - elapsed
            )

            remaining_distance = (
                0.5
                * acceleration
                * remaining_time ** 2
            )

            distance = (
                total_distance
                - remaining_distance
            )

        self._position = (
            self._move_start_position
            + direction * distance
        )

    def _read_position(self) -> float:
        with self._motor_lock:
            self._update_motion_locked()
            return self._position


    def _read_is_moving(self) -> bool:
        with self._motor_lock:
            self._update_motion_locked()
            return self._is_moving


    def _read_motor_state(self) -> tuple[float, bool]:
        with self._motor_lock:
            self._update_motion_locked()

            return (
                self._position,
                self._is_moving,
            )

    def start_tracking(self) -> None:
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

    def __enter__(self) -> 'DummyMotor':
        return self

    def __exit__(
        self,
        exc_type: object,
        exc_value: object,
        traceback: object,
    ) -> None:
        self.disconnect()

if __name__ == '__main__':
    try:
        with DummyMotor() as motor:
            print(f'Starting position: {motor.position:.3f}°')

            motor.move_by(angle=90)
            while motor.is_moving:
                print(
                    f'\rPosition: {motor.position:8.3f}°',
                    end='',
                    flush=True,
                )
                threading.Event().wait(0.1)

            motor.move_to(position=0)
            while motor.is_moving:
                print(
                    f'\rPosition: {motor.position:8.3f}°',
                    end='',
                    flush=True,
                )
                threading.Event().wait(0.1)

            print(f'\nFinal position: {motor.position:.3f}°')

    except KeyboardInterrupt:
        print('\nInterrupted')