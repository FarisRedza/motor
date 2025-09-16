import pathlib
import platform
import threading
import time
import enum

import elliptec

from . import base_motor

def list_elliptec_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_elliptec_motors_linux()
        
        case _:
            raise NotImplementedError(f'Unsupported system: {system}')

def list_elliptec_motors_linux() -> list[tuple[str, str]]:
    symlinks = pathlib.Path('/dev/serial/by-id')
    if not symlinks.exists():
        return []

    motors: list[tuple[str, str]] = []
    for s in symlinks.iterdir():
        if 'FTDI_FT230X' in s.name:
            try:
                controller = elliptec.Controller(
                    port=str(s.resolve())
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
            serial_number: str
    ) -> None:
        self._motor_max_velocity_serial = 64
        self._motor_max_velocity = 430
        self._motor_min_velocity = 0.5 * self._motor_max_velocity

        self.direction = base_motor.MotorDirection.IDLE
        self.step_size = 5.0
        self.acceleration = 0.0
        self.max_velocity = 0.0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._movement_thread: threading.Thread | None = None
        self._position_thread: threading.Thread | None = None
        self._position_polling = 0.1

        self._get_motor(serial_number=serial_number)
        self.position = self._get_angle()        
        
        self._motor.send_instruction(
            instruction=b'sv',
            message='64'
        )
    
    def _velocity_to_serial(self, velocity: int) -> int:
        return int(velocity/self._motor_max_velocity_serial) * self._motor_max_velocity_serial
    
    def _serial_to_velocity(self, serial: int) -> float:
        return (serial / self._motor_max_velocity_serial) * self._motor_max_velocity

    def disconnect(self) -> None:
        self._motor.close_connection()

    def stop(self) -> None:
        with self._lock:
            self._motor.send_instruction(instruction=b'st')
        self.position = self._get_angle()

    def move_by(
            self,
            angle: float,
            acceleration: float = 0,
            max_velocity: float = 0
    ) -> bool:
        self._start_tracking_position()
        match angle:
            case a if a > 0:
                self.direction = base_motor.MotorDirection.FORWARD
            case a if a < 0:
                self.direction = base_motor.MotorDirection.BACKWARD
            case 0:
                self.direction = base_motor.MotorDirection.IDLE
            case _:
                raise RuntimeError('Invalid angle')

        try:
            with self._lock:
                self._motor.shift_angle(angle=angle)
        except Exception as e:
            print(f'Exception in move_by: {e}')
            self._stop_tracking_position()
            return False
        else:
            self._stop_tracking_position()
            return True
    
    def move_to(
            self,
            position: float,
            acceleration: float = 0,
            max_velocity: float = 0
    ) -> bool:
        delta = position - self.position
        self.move_by(angle=delta)

        # self._motor.set_angle(angle=position)

        return True
    
    def jog(
            self,
            direction: base_motor.MotorDirection,
            acceleration: float = 0,
            max_velocity: float = 0
    ) -> None:
        match direction:
            case base_motor.MotorDirection.FORWARD:
                dir = 'fw'
            case base_motor.MotorDirection.BACKWARD:
                dir = 'bw'
            case base_motor.MotorDirection.IDLE:
                dir = None
            case _:
                raise RuntimeError('Invalid direction')

        if dir:
            with self._lock:
                self._motor.send_instruction(
                    instruction=b'sv',
                    # message='44'
                    message='32'
                )
                self._motor.set_jog_step(angle=0)
                self._motor.send_instruction(
                    instruction=dir.encode(encoding='utf-8')
                )

    def threaded_move_by(
            self,
            angle: float,
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
    
    def _get_motor(self, serial_number: str) -> None:
        system = platform.system()
        match system:
            case 'Linux':
                symlinks = pathlib.Path('/dev/serial/by-id').iterdir()
                for s in symlinks:
                    if 'FTDI_FT230X' in s.name:
                        controller = elliptec.Controller(
                            port=str(s.resolve())
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

    def _track_position(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.position = self._get_angle()
            except Exception as e:
                print(f'[tracking error] {e}')
            time.sleep(self._position_polling)

    def _start_tracking_position(self) -> None:
        if self._position_thread and self._position_thread.is_alive():
            return
        self._stop_event.clear()
        self._position_thread = threading.Thread(
            target=self._track_position,
            daemon=True
        )
        self._position_thread.start()

    def _stop_tracking_position(self) -> None:
        self._stop_event.set()
        if self._position_thread:
            self._position_thread.join()
        self._position_thread = None
        self.direction = base_motor.MotorDirection.IDLE

    def _get_angle(self) -> float:
        angle = None
        while angle is None:
            with self._lock:
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