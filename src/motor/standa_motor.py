import pathlib
import platform
import threading
import time
import math
import typing
import enum

import libximc.highlevel as ximc
ximc.MoveFlags._boundary_ = enum.KEEP
MVCMD_RUNNING = 0x80

from motor import base_motor

def list_standa_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_standa_motors_linux()

        case 'Windows':
            return list_standa_motors_windows()

        case _:
            raise NotImplementedError(f'Unsupported system: {system}')
        
def list_standa_motors_linux() -> list[tuple[str, str]]:
    device_path = pathlib.Path('/dev/serial/by-id')
    if not device_path.exists():
        return []

    motors: list[tuple[str, str]] = []
    for symlink in device_path.iterdir():
        if 'XIMC_Motor_Controller' in symlink.name:
            try:
                if 'XIMC_Motor_Controller' in symlink.name:
                    port = symlink.resolve()
                    motor = ximc.Axis(f'xi-com:{port}')
                    motor.open_device()
                    serial_number = str(motor.get_serial_number())
                    device_name = motor.get_device_information().Manufacturer
                    motor.close_device()
                    motors.append(
                        (serial_number, device_name)
                    )
            except IndexError:
                continue
    return motors

def list_standa_motors_windows() -> list[tuple[str, str]]:
    motors = []
    return motors


class StandaMotor(base_motor.Motor):
    def __init__(
            self,
            serial_number: str
        ) -> None:
        self._full_step = 28800
        self._connect_motor(serial_number=serial_number)

        dev_info = self._motor.get_device_information()
        fw = self._motor.get_firmware_version()
        self.device_info = base_motor.DeviceInfo(
            device_name=dev_info.ProductDescription,
            model=f'{dev_info.Major}.{dev_info.Minor}.{dev_info.Release}',
            serial_number=serial_number,
            firmware_version=f'{fw[0]}.{fw[1]}.{fw[2]}'
        )

        init_settings = self._motor.get_move_settings()
        self.position = self._step_to_angle(
            step=self._motor.get_position().Position
        )
        self.direction = base_motor.MotorDirection.IDLE
        self.step_size = 5.0

        self.acceleration = self._step_to_angle(
            step=init_settings.Accel
        )
        self.max_velocity = self._step_to_angle(
            step=init_settings.Speed
        )

        # print(self._motor.get_device_information())

    def move_by(self, angle: float, acceleration: float, max_velocity: float) -> bool:
        step = self._angle_to_step(angle)
        self._check_settings(accel=acceleration, speed=max_velocity)
        self._step_motor(step)
        return True

    def move_to(self, position: float, acceleration: float, max_velocity: float) -> bool:
        pass

    def threaded_move_by(self, angle: float, acceleration: float, max_velocity: float) -> None:
        self.move_by(
            angle=angle,
            acceleration=acceleration,
            max_velocity=max_velocity
        )

    def stop(self) -> None:
        pass

    def disconnect(self) -> None:
        self._motor.close_device()

    def _check_settings(self, accel: float, speed: float) -> None:
        accel_step = self._angle_to_step(angle=accel)
        speed_step = self._angle_to_step(angle=speed)

        settings = self._motor.get_move_settings()

        if settings.Speed != speed_step or settings.Speed != accel_step:
            settings.Speed = speed_step
            settings.Accel = accel_step
            settings.Decel = 2*accel_step
            self._motor.set_move_settings(settings=settings)
            self.acceleration = accel
            self.max_velocity = speed

    def _connect_motor(self, serial_number: str) -> None:
        system = platform.system()
        match system:
            case 'Linux':
                symlinks = pathlib.Path('/dev/serial/by-id').iterdir()
                for s in symlinks:
                    if 'XIMC_Motor_Controller' in s.name:
                        port = s.resolve()
                        motor = ximc.Axis(uri=f'xi-com:{port}')
                        motor.open_device()
                        sn = str(motor.get_serial_number())
                        if sn == serial_number:
                            self._motor = motor
                            break
                        else:
                            motor.close_device()
            case _:
                raise ValueError(f'Unsupported system: {system}')

    def _step_motor(self, step: int) -> None:
        self._motor.command_movr(step, 0)
        self._update_pos_while_moving()
    
    def _update_pos_while_moving(self) -> None:
        while True:
            self.position = self._step_to_angle(
                step=self._motor.get_position().Position
            )
            print(self.position)
            status = self._motor.get_status()
            moving = bool(int(status.MvCmdSts) & MVCMD_RUNNING)

            if not moving:
                self.position = self._step_to_angle(
                    step=self._motor.get_position().Position
                )
                return
            
            time.sleep(0.1)

    def _angle_to_step(self, angle: float) -> int:
        step = int((angle / 360.0) * self._full_step)
        return step

    def _step_to_angle(self, step: int) -> float:
        angle = (step / self._full_step) * 360.0
        return angle


def get_all_motors() -> list[StandaMotor]:
    motors = []
    for m in list_standa_motors():
        try:
            motor = StandaMotor(serial_number=m[0])
        except:
            continue
        else:
            motors.append(motor)
    return motors

if __name__ == '__main__':
    # motor = StandaMotor(serial_number='55356974')
        #     port = '/dev/ttyACM' + str(port_num)
        # motor = Motor(
        #     full_step = 28800,
        #     port = port,
        #     motor = ximc.Axis('xi-com:' + port),
    motor = StandaMotor(
        serial_number='37398'
    )
    motor.move_by(angle=90, acceleration=25, max_velocity=12.5)