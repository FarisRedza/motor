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
        self._get_motor(serial_number=serial_number)
        position = self._motor.get_angle()
        if isinstance(position, float):
            self.position = float(position)
        else:    
            raise RuntimeError('Unable to get motor position')           
        
        self.direction = base_motor.MotorDirection.IDLE
        self.step_size = 5.0
        self.acceleration = 0.0
        self.max_velocity = 0.0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._movement_thread: threading.Thread | None = None
        self._position_thread: threading.Thread | None = None
        self._position_polling = 0.1

    def disconnect(self) -> None:
        self._motor.close_connection()

    def stop(self) -> None:
        self._motor.send_instruction(instruction=b'st')

    def move_by(
            self,
            angle: float,
            acceleration: float = 0,
            max_velocity: float = 0
    ) -> bool:
        self._start_tracking_position()
        direction = 1 if angle >= 0 else -1
        self.direction = base_motor.MotorDirection.FORWARD if angle > 0 else base_motor.MotorDirection.BACKWARD

        remaining = abs(angle)
        while remaining > 0:
            step = min(remaining, 360) * direction
            try:
                with self._lock:
                    self._motor.shift_angle(angle=step)
            except Exception as e:
                print(f'Exception in move_by: {e}')
                return False
            remaining -= min(remaining, 360)

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
    
    def jog(
            self,
            direction: base_motor.MotorDirection,
            acceleration: float = 0,
            max_velocity: float = 0
    ) -> None:
        self._motor.set_jog_step(angle=0)
        self.position = self._motor.jog(direction=direction.name.lower())

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
                    controller = elliptec.Controller(
                        port=str(s.resolve())
                    )
                    dev_info = controller.send_instruction(instruction=b'in')
                    if isinstance(dev_info, dict) and serial_number in dev_info.values():
                        self._motor = elliptec.Rotator(controller=controller)
                        self.device_info = base_motor.DeviceInfo(
                            device_name='Elliptec',
                            model='14',
                            serial_number=serial_number,
                            firmware_version=dev_info['Firmware']
                        )

                    else:
                        controller.close_connection()

                if not self._motor:
                    raise RuntimeError(f'Motor {serial_number} not found')
                
            case _:
                raise NotImplementedError(f'Unsupported system: {system}')

    def _track_position(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                try:
                    self.position = self._motor.get_angle()
                    print(self.position)
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

    def _stop_tracking_position(self):
        self._stop_event.set()
        if self._position_thread:
            self._position_thread.join()
        self._position_thread = None
        self.direction = base_motor.MotorDirection.IDLE


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
    
    motors = get_all_motors()
    
    for motor in motors:
        motor.move_to(position=300)
        motor.move_to(position=-300)
        motor.move_to(position=0)
        print(motor.position)