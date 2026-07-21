import socket
import struct
import threading
import typing
import time
import json
from collections.abc import Callable

from motor import base_motor
from motor import remote_server

PositionCallback = Callable[[float], None]

def send_command(
        sock: socket.socket,
        command: remote_server.Command,
        args : typing.Optional[tuple] = None
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

def list_motors(
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
            *,
            host: str,
            port: int,
            tracking_interval: float = 0.05,
            start_tracking: bool = True,
    ) -> None:
        if tracking_interval <= 0:
            raise ValueError('tracking_interval must be positive')

        self.host = host
        self.port = port

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

    def _connect(self, serial_number: str) -> None:
        self._sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )
        self._sock.settimeout(5)
        self._sock.connect((self.host, self.port))
        
        send_command(
            sock=self._sock,
            command=remote_server.Command.LIST_DEVICES
        )
        response, payload = receive_response(sock=self._sock)

        if response != remote_server.Response.LIST_DEVICES:
            raise ValueError(f'Unexpected response: {response}')

        devs = json.loads(payload.decode(encoding='utf-8'))
        # TODO: fix the decoding, it's all lists for some reason
        try:
            next(str(sn[0]) for sn in devs if str(sn[0]) == serial_number)
        except StopIteration:
            raise ValueError(f"Motor '{serial_number}' not found")
            
        send_command(
            sock=self._sock,
            command=remote_server.Command.REQUEST_DEVICE,
            args=(serial_number,)
        )
        response, payload = receive_response(sock=self._sock)
        if response != remote_server.Response.REQUEST_DEVICE:
            raise ValueError(payload)
        
        else:
            print(payload)

    def move_by(
        self,
        angle: float,
        acceleration: typing.Optional[float] = None,
        max_velocity: typing.Optional[float] = None,
    ) -> None:
        if acceleration:
            self.acceleration = acceleration
        if max_velocity:
            self.max_velocity = max_velocity

        send_command(
            self._sock,
            command=remote_server.Command.MOVE_BY,
            args=(angle, self.acceleration, self.max_velocity)
        )
        response, payload = receive_response(sock=self._sock)
        if response != remote_server.Response.STATUS:
            raise ValueError(payload)
        
        else:
            print(payload)
        

if __name__ == '__main__':
    m = RemoteMotor(
        serial_number='dummy_motor_1',
        host='127.0.0.1',
        port=5001
    )
    m.move_by(
        angle=90,
        acceleration=15,
        max_velocity=15
    )