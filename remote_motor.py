import threading
import socket
import typing
import json
import time

import motor.base_motor as base_motor

server_host = '127.0.0.1'
server_host = '137.195.89.222'
server_port = 5002

MAX_ACCELERATION = 20.0
MAX_VELOCITY = 25.0

def send_request(
        host: str,
        port: int,
        command: base_motor.Commands,
        arguments: list = [],
        timeout: int = 5
) -> typing.Any:
    request = {'command': command.value}
    try:
        with socket.create_connection(
            address=(host, port),
            timeout=timeout
        ) as s:
            match command:
                case base_motor.Commands.LIST_MOTORS:
                    pass

                case base_motor.Commands.GET_POSITION:
                    request['serial_number'] = arguments[0]

                case base_motor.Commands.STOP:
                    request['serial_number'] = arguments[0]

                case base_motor.Commands.MOVE_BY:
                    request['serial_number'] = arguments[0]
                    request['angle'] = arguments[1]
                    request['acceleration'] = arguments[2]
                    request['max_velocity'] = arguments[3]

                case base_motor.Commands.MOVE_TO:
                    request['serial_number'] = arguments[0]
                    request['position'] = arguments[1]
                    request['acceleration'] = arguments[2]
                    request['max_velocity'] = arguments[3]

                case base_motor.Commands.JOG:
                    request['serial_number'] = arguments[0]
                    request['direction'] = arguments[1]
                    request['acceleration'] = arguments[2]
                    request['max_velocity'] = arguments[3]

                case _:
                    raise Exception('Unknown command')

            s.sendall(json.dumps(request).encode())
            buffer = ""
            while True:
                data = s.recv(1024).decode()
                if not data:
                    break
                buffer += data
                if "\n" in buffer:
                    break
            response = json.loads(buffer.strip())
            return response

    except Exception as e:
        return {f'Error sending request {request}': str(e)}
    
def list_device_info(
        host: str,
        port: int
) -> list[base_motor.DeviceInfo]:
    result = send_request(
        host=host,
        port=port,
        command=base_motor.Commands.LIST_MOTORS
    )
    return [base_motor.DeviceInfo(**motor) for motor in result['motors']]

class Motor(base_motor.Motor):
    def __init__(
            self,
            serial_number: str,
            host: str,
            port: int
    ) -> None:
        self.host = host
        self.port = port
        self._get_motor(serial_number=serial_number)
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
    ) -> None:
        motors = list_device_info(
            host=self.host,
            port=self.port
        )
        motor_index = next(
            (i for i, motor in enumerate(motors) if str(motor.serial_number) == serial_number),
            None
        )
        if motor_index is None:
            raise Exception
        else:
            self.device_info = motors[motor_index]
            
            status = send_request(
                host=self.host,
                port=self.port,
                command=base_motor.Commands.GET_POSITION,
                arguments=[self.device_info.serial_number]
            )
            self.position = float(status['position'])
            self.is_moving = bool(status['moving'])
            self.direction = base_motor.MotorDirection(status['direction'])

            print(f'Connected to motor {self.device_info.serial_number}')
            print(f'Motor status: {status}')

    def _track_position(self) -> None:
        while True:
            time.sleep(self._position_polling)
            with self._lock:
                try:
                    update = send_request(
                        host=self.host,
                        port=self.port,
                        command=base_motor.Commands.GET_POSITION,
                        arguments=[self.device_info.serial_number]
                    )
                    self.position = float(update['position'])
                    self.direction = base_motor.MotorDirection(
                        update['direction']
                    )
                    moving = bool(update['moving'])
                except Exception:
                    moving = False

            if self._stop_event.is_set() and not moving:
                break

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

    def stop(self) -> None:
        send_request(
            host=self.host,
            port=self.port,
            command=base_motor.Commands.STOP,
            arguments=[self.device_info.serial_number],
        )
        self.is_moving = False
        self._stop_tracking_position()

    def move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        with self._lock:
            response = send_request(
                host=self.host,
                port=self.port,
                command=base_motor.Commands.MOVE_BY,
                arguments=[
                    self.device_info.serial_number,
                    angle,
                    acceleration,
                    max_velocity
                ]
            )
            self.is_moving = bool(response['moving'])
        self._start_tracking_position()
        while True:
            if self.is_moving == False:
                break
        return True

    def move_to(
            self,
            position: float,
            acceleration: float,
            max_velocity: float
    ) -> bool:
        with self._lock:
            response = send_request(
                host=self.host,
                port=self.port,
                command=base_motor.Commands.MOVE_TO,
                arguments=[
                    self.device_info.serial_number,
                    position,
                    acceleration,
                    max_velocity
                ]
            )
            self.is_moving = bool(response['moving'])
        self._start_tracking_position()
        while True:
            if self.is_moving == False:
                break
        return True
    
    def threaded_move_by(
            self,
            angle: float,
            acceleration: float,
            max_velocity: float
    ) -> None:
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
        response = send_request(
            host=self.host,
            port=self.port,
            command=base_motor.Commands.JOG,
            arguments=[
                self.device_info.serial_number,
                direction.value,
                acceleration,
                max_velocity
            ]
        )
        # self.is_moving = response['moving']
        self.is_moving = True
        self._start_tracking_position()

def list_motors(
        host: str,
        port: int
) -> list[Motor]:
    result = send_request(
        host=host,
        port=port,
        command=base_motor.Commands.LIST_MOTORS
    )
    return [
        Motor(
            serial_number=base_motor.DeviceInfo(**motor).serial_number,
            host=host,
            port=port
        ) for motor in result['motors']
    ]

if __name__ == '__main__':
    motor = Motor(
        serial_number='55353314',
        host=server_host,
        port=server_port
    )

    motor.jog(
        direction=base_motor.MotorDirection.FORWARD,
        acceleration=20.0,
        max_velocity=5.0
    )
    time.sleep(5)
    motor.stop()