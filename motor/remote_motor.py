import socket
import struct
import threading
import time
import json

from . import base_motor
from . import remote_server

def send_command(
        sock: socket.socket,
        command: remote_server.Command,
        args : tuple | None = None
) -> None:
    encoded_args = [
        str(arg).encode(encoding='utf-8') for arg in args
    ] if args else []
    payload = struct.pack('II', command, len(encoded_args))

    for arg in encoded_args:
        payload += struct.pack('I', len(arg)) + arg

    sock.sendall(payload)

def recvall(size: int, sock: socket.socket) -> bytes:
    data = bytearray()
    while len(data) < size:
        part = sock.recv(size - len(data))
        if not part:
            raise ConnectionError('Socket closed')
        data.extend(part)
    return data

def receive_response(sock: socket.socket) -> tuple[int, bytes]:
    header = recvall(size=5, sock=sock)
    total_len, response_id = struct.unpack('IB', header)
    try:
        response = remote_server.Response(response_id)
    except ValueError:
        raise ValueError(f'Invalid response ID: {response_id}')
    else:
        payload = recvall(
            size=total_len - 1,
            sock=sock
        )
        return response, payload

def list_thorlabs_motors(
        host: str | None = None,
        port: int | None = None,
        sock: socket.socket | None = None
) -> list[tuple[str,str]]:
    if sock:
        host, port = sock.getpeername()
    elif host and port:
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )
        sock.settimeout(5)
        sock.connect((host, port))
    else:
        raise ValueError('Must provide either a socket or host and port')

    send_command(
        sock=sock,
        command=remote_server.Command.LIST_DEVICES
    )
    response, payload = receive_response(sock=sock)
    
    devices: list[tuple[str, str]] = []
    if response == remote_server.Response.LIST_DEVICES:
        devs = json.loads(payload.decode(encoding='utf-8'))
        for d in devs:
            devices.append((str(d[0]), str(d[1])))

    else:
        print(f'Unexpected response: {response}')

    return devices

class RemoteMotor(base_motor.Motor):
    def __init__(
            self,
            serial_number: str,
            host: str | None = None,
            port: int | None = None,
            sock: socket.socket | None = None
    ) -> None:
        if sock:
            self.host, self.port = sock.getpeername()
            self._sock = sock
        elif host and port:
            self.host = host
            self.port = port
            self._sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )
            self._sock.settimeout(5)
            self._sock.connect((self.host, self.port))
        else:
            raise NameError('Must provide either a socket or host and port')
        self._get_device_info(serial_number=serial_number)
        self.step_size = 5.0
        self._get_status()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._position_thread: threading.Thread | None = None
        self._position_polling = 0.1
        self._movement_thread: threading.Thread | None = None
    
    def disconnect(self) -> None:
        self.stop()

    def stop(self) -> None:
        send_command(
            sock=self._sock,
            command=remote_server.Command.STOP,
            args=(self.device_info.serial_number,)
        )
        payload = self._handle_response(
            expected_response_id=remote_server.Response.STATUS
        )
        msg_len, = struct.unpack('I', payload[:4])
        status_msg = payload[4:4 + msg_len].decode(encoding='utf-8')
        self.direction = base_motor.MotorDirection(value=status_msg)
        self._stop_tracking_position()
        
    def move_by(
            self,
            angle: float,
            acceleration: float | None = None,
            max_velocity: float | None = None
    ) -> bool:
        with self._lock:
            send_command(
                sock=self._sock,
                command=remote_server.Command.MOVE_BY,
                args=(
                    self.device_info.serial_number,
                    angle,
                    acceleration if acceleration else None,
                    max_velocity if max_velocity else None
                )
        )
        payload = self._handle_response(
            expected_response_id=remote_server.Response.STATUS
        )
        msg_len, = struct.unpack('I', payload[:4])
        status_msg = payload[4:4 + msg_len].decode(encoding='utf-8')
        update = status_msg.split(',')
        self.direction = base_motor.MotorDirection(value=update[1])
        self._start_tracking_position()
        while True:
            time.sleep(self._position_polling)
            if self.direction == base_motor.MotorDirection.IDLE:
                self._stop_tracking_position()
                break
        return True
    
    def move_to(
            self,
            position: float,
            acceleration: float | None = None,
            max_velocity: float | None = None
    ) -> bool:
        with self._lock:
            send_command(
                sock=self._sock,
                command=remote_server.Command.MOVE_TO,
                args=(
                    self.device_info.serial_number,
                    position,
                    acceleration if acceleration else None,
                    max_velocity if max_velocity else None
                )
        )
        payload = self._handle_response(
            expected_response_id=remote_server.Response.STATUS
        )
        msg_len, = struct.unpack('I', payload[:4])
        status_msg = payload[4:4 + msg_len].decode(encoding='utf-8')
        update = status_msg.split(',')
        self.direction = base_motor.MotorDirection(value=update[1])
        self._start_tracking_position()
        while True:
            time.sleep(self._position_polling)
            if self.direction == base_motor.MotorDirection.IDLE:
                self._stop_tracking_position()
                break
        return True
    
    def threaded_move_by(
            self,
            angle: float,
            acceleration: float | None = None,
            max_velocity: float | None = None
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
            acceleration: float | None = None,
            max_velocity: float | None = None
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
        send_command(
            sock=self._sock,
            command=remote_server.Command.JOG,
            args=(
                self.device_info.serial_number,
                direction.value,
                acceleration if acceleration else None,
                max_velocity if max_velocity else None
            )
        )
        payload = self._handle_response(
            expected_response_id=remote_server.Response.STATUS
        )
        msg_len, = struct.unpack('I', payload[:4])
        status_msg = payload[4:4 + msg_len].decode(encoding='utf-8')
        update = status_msg.split(',')
        self.direction = base_motor.MotorDirection(value=update[0])
        self._start_tracking_position()

    def _handle_response(
            self,
            expected_response_id: remote_server.Response
    ):
        response, payload = receive_response(self._sock)

        match response:
            case r if r == expected_response_id:
                return payload
            
            case remote_server.Response.ERROR:
                error_msg = payload.decode(encoding='utf-8')
                raise RuntimeError(f'Server error: {error_msg}')
            
            case _:
                raise ValueError(f'Unexpected response: {response}')

    def _get_device_info(
            self,
            serial_number: str
    ) -> None:
        send_command(
            sock=self._sock,
            command=remote_server.Command.DEVICE_INFO,
            args=(serial_number,)
        )
        payload = self._handle_response(
            expected_response_id=remote_server.Response.DEVICE_INFO,
        )
        self.device_info = base_motor.DeviceInfo.deserialise(
            payload=payload
        )

    def _get_status(self) -> None:
        send_command(
            sock=self._sock,
            command=remote_server.Command.GET_STATUS,
            args=(self.device_info.serial_number,)
        )
        payload = self._handle_response(
            expected_response_id=remote_server.Response.STATUS
        )
        msg_len, = struct.unpack('I', payload[:4])
        status_msg = payload[4:4 + msg_len].decode(encoding='utf-8')

        update = status_msg.split(',')
        self.position = float(update[0])
        self.direction = base_motor.MotorDirection(value=update[1])
        self.acceleration = float(update[2])
        self.max_velocity = float(update[3])


    def _track_position(self) -> None:
        while True:
            time.sleep(self._position_polling)
            with self._lock:
                try:
                    send_command(
                        sock=self._sock,
                        command=remote_server.Command.GET_STATUS,
                        args=(self.device_info.serial_number,)
                    )
                    payload = self._handle_response(
                        expected_response_id=remote_server.Response.STATUS
                    )
                    msg_len, = struct.unpack('I', payload[:4])
                    status_msg = payload[4:4 + msg_len].decode(encoding='utf-8')

                    update = status_msg.split(',')
                    self.position = float(update[0])
                    self.direction = base_motor.MotorDirection(value=update[1])
                    self.acceleration = float(update[2])
                    self.max_velocity = float(update[3])

                except Exception as e:
                    raise RuntimeError(f'Error: {e}')   

            if self.direction == base_motor.MotorDirection.IDLE:
                break

    def _start_tracking_position(self) -> None:
        self._stop_event.clear()
        self._position_thread = threading.Thread(
            target=self._track_position
        )
        self._position_thread.start()

    def _stop_tracking_position(self) -> None:
        self._stop_event.set()
        if self._position_thread:
            self._position_thread.join()

if __name__ == '__main__':
    # m = RemoteMotor(
    #     serial_number='55356974',
    #     host='127.0.0.1',
    #     port=5002
    # )
    # m.move_to(
    #     position=0,
    #     acceleration=20,
    #     max_velocity=25
    # )
    list_thorlabs_motors(
        host='127.0.0.1',
        port=5002
    )