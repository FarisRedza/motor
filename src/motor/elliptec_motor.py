import pathlib
import platform
import threading
import time
import typing
from collections.abc import Callable

try:
    import elliptec
except ModuleNotFoundError:
    elliptec = None

from motor import base_motor

PositionCallback = Callable[[float], None]


def is_available() -> bool:
    return elliptec is not None

def list_elliptec_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_elliptec_motors_linux()
        
        case 'Windows':
            raise NotImplementedError(
                'Windows support not implemented yet'
            )

        case _:
            raise ValueError(f'{system} is not supported')

def list_elliptec_motors_linux() -> list[tuple[str, str]]:
    serial_directory = pathlib.Path('/dev/serial/by-id')
    
    if not serial_directory.exists():
        return []

    motors: list[tuple[str, str]] = []
    for device_id in serial_directory.iterdir():
        if 'FTDI_FT230X' in device_id.name:
            try:
                controller = elliptec.Controller(
                    port=str(device_id.resolve())
                )
                dev_info = controller.send_instruction(instruction=b'in')
                controller.close_connection()
                if isinstance(dev_info, dict):
                    motors.append(
                        (dev_info['Serial No.'], 'Elliptec')
                    )
            except IndexError:
                continue
    return motors

class ElliptecMotor(base_motor.Motor):
    def __init__(
            self,
            serial_number: str,
            *,
            tracking_interval: float = 0.05,
            start_tracking: bool = True,
    ) -> None:
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
        self._motor_max_velocity_serial = 64

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
            position = self._motor.get_angle()
            # moving = self._motor.is_moving()
            moving = True
            # TODO: need to find function for this, may have to use
            # something like _DEVGET_STATUS from Elliptec Thorlabs
            # ELLx Resonant Piezo Motor Communication Protocol

        return position, moving

    def _connect(self, serial_number: str) -> None:
        system = platform.system()
        match system:
            case 'Linux':
                serial_directory = pathlib.Path('/dev/serial/by-id').iterdir()
                for device_id in serial_directory:
                    if 'FTDI_FT230X' in device_id.name:
                        controller = elliptec.Controller(
                            port=str(device_id.resolve())
                        )
                        dev_info = controller.send_instruction(instruction=b'in')
                        if isinstance(dev_info, dict) and serial_number in dev_info.values():
                            self._motor = elliptec.Rotator(controller=controller)
                            self.device_info = base_motor.DeviceInfo(
                                device_name='Elliptec',
                                model=dev_info['Motor Type'],
                                serial_number=serial_number,
                                firmware_version=dev_info['Firmware']
                            )
                            self._range = dev_info['Range']
                            self._pulse_per_rev = dev_info['Pulse/Rev']

                        else:
                            controller.close_connection()

                if not self._motor:
                    raise RuntimeError(f'Motor {serial_number} not found')
                
            case _:
                raise NotImplementedError(f'Unsupported system: {system}')

    
    def _velocity_to_serial(self, velocity: int) -> int:
        return int(velocity/self._motor_max_velocity_serial) * self._motor_max_velocity_serial
    
    def _serial_to_velocity(self, serial: int) -> float:
        return (serial / self._motor_max_velocity_serial) * self._motor_max_velocity

    def disconnect(self) -> None:
        self._motor.close_connection()

    def stop(self) -> None:
        with self._motor_lock:
            self._motor.send_instruction(instruction=b'st')

    def __enter__(self) -> 'ElliptecMotor':
        return self

    def __exit__(
        self,
        exc_type: object,
        exc_value: object,
        traceback: object,
    ) -> None:
        self.disconnect()

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

        self._motor.shift_angle(angle=angle)
    
    def move_to(
            self,
            position: float,
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

        delta = position - self.position
        self.move_by(angle=delta)
    
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

        with self._state_lock:
            self._is_moving = True

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
                self._motor.send_instruction(
                    instruction=b'sv',
                    # message='44'
                    message='32'
                )
                self._motor.set_jog_step(angle=0)
                self._motor.send_instruction(
                    instruction=dir.encode(encoding='utf-8')
                )

    def _get_angle(self) -> float:
        angle = None
        while angle is None:
            with self._motor_lock:
                angle = self._motor.get_angle()
                # position = self._motor.send_instruction(instruction=b'gp')[2]
                # angle = self._position_to_angle(position=int(position))
                # print(position, angle)
        return float(angle)
    
    def _position_to_angle(self, position: int) -> float:
        return position / self._pulse_per_rev * self._range 

    def _angle_to_position(self, angle: float) -> int:
        return int(angle / self._range * self._pulse_per_rev)

def get_all_motors() -> list[ElliptecMotor]:
    motors = []
    for m in list_elliptec_motors():
        try:
            motor = ElliptecMotor(serial_number=m[0])
        except:
            continue
        else:
            motors.append(motor)
    return motors


if __name__ == '__main__':
    print(list_elliptec_motors())
    motor = get_all_motors()[0]

    motor._motor.send_instruction(
        instruction=b'sv',
        message='32'
    )
    angle = 360
    for _ in range(2):
        motor._motor.set_angle(angle=0)
        time.sleep(1)
        start_time = time.time()
        motor._motor.shift_angle(angle=angle)
        end_time = time.time() - start_time
        print(end_time)
        print(f'degrees per second = {angle/end_time}')