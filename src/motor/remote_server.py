import socket
import threading
import struct
import enum
import json
import typing

from motor.base_motor import Motor
from motor import dummy_motor, k10cr2_motor, thorlabs_motor, standa_motor


class Command(enum.IntEnum):
    LIST_DEVICES = 1
    REQUEST_DEVICE = 2
    DEVICE_INFO = 3
    GET_STATUS = 4
    STOP = 5
    MOVE_BY = 6
    MOVE_TO = 7
    JOG = 8
    GET_POSITION = 9

class Response(enum.IntEnum):
    ERROR = 0
    LIST_DEVICES = 1
    REQUEST_DEVICE = 2
    DEVICE_INFO = 3
    STATUS = 4
    POSITION = 5

def recvall(size: int, sock: socket.socket) -> bytes:
    data = bytearray()
    while len(data) < size:
        part = sock.recv(size - len(data))
        if not part:
            raise ConnectionError('Socket closed')
        data.extend(part)
    return data

def receive_command(
        sock: socket.socket
) -> tuple[Command, list]:
    header = recvall(size=8, sock=sock)
    command_id, num_args = struct.unpack('II', header)

    try:
        command = Command(command_id)
    except ValueError:
        raise ValueError(f'Invalid command ID: {command_id}')

    args = []    
    for _ in range(num_args):
        arg_len = struct.unpack('I', recvall(size=4, sock=sock))[0]
        arg = recvall(size=arg_len, sock=sock)
        args.append(arg.decode(encoding='utf-8'))

    return command, args

def send_message(
        sock: socket.socket,
        message: str,
        response_id: Response
) -> None:
    payload = struct.pack(
        f'I{len(message)}s',
        len(message),
        message.encode(encoding='utf-8')
    )
    header = struct.pack(
        'IB',
        len(payload) + 1,
        response_id
    )
    sock.sendall(header + payload)

def send_payload(
        sock: socket.socket,
        payload: bytes,
        response_id: Response
) -> None:
    header = struct.pack('IB', len(payload) +1, response_id)
    sock.sendall(header + payload)

def handle_client(
        sock: socket.socket,
        address,
        available_motors: list[tuple[str, str]]
) -> None:
    with sock:
        try:
            motor: typing.Optional[Motor] = None
            while True:
                try:
                    command, args = receive_command(sock=sock)
                except (ValueError, ConnectionError) as e:
                    print(f'[{address}] Disconnected: {e}')
                    break
                else:
                    match command:
                        case Command.LIST_DEVICES:
                            send_payload(
                                sock=sock,
                                payload=json.dumps(available_motors).encode(encoding='utf-8'),
                                response_id=Response.LIST_DEVICES
                            )
                        
                        case Command.REQUEST_DEVICE:
                            serial_number = args[0]
                            if motor is not None:
                                send_message(
                                    sock=sock,
                                    message=f"Motor '{serial_number}' already requested",
                                    response_id=Response.ERROR
                                )
                            else:
                                try:
                                    dev_sn, dev_type = next(d for d in available_motors if d[0] == serial_number)
                                except StopIteration:
                                    raise ValueError(f"Motor '{serial_number}' not found")
                                
                                match dev_type:
                                    case 'Dummy Motor':
                                        motor = dummy_motor.DummyMotor()
                                        send_message(
                                            sock=sock,
                                            message=f'Motor successfully requested',
                                            response_id=Response.REQUEST_DEVICE
                                        )

                                    case _:
                                        send_message(
                                            sock=sock,
                                            message=f'Unknown motor type: {dev_type}',
                                            response_id=Response.ERROR
                                        )
                        
                        case Command.MOVE_BY:
                            angle: float = args[0]
                            acceleration: float = args[1]
                            max_velocity: float = args[2]

                            motor.move_by(
                                angle=angle,
                                acceleration=acceleration,
                                max_velocity=max_velocity
                            )

                            send_message(
                                sock=sock,
                                message=f'Moving motor',
                                response_id=Response.STATUS
                            )


                        case _:
                            send_message(
                                sock=sock,
                                message=f'Unknown command: {command}',
                                response_id=Response.ERROR
                            )

        except Exception as e:
            print(f'[{address}] Unexpected error: {e}')
        finally:
            print(f'Disconnected from {address}')

def start_server(
        host: str = '0.0.0.0',
        port: int = 5002,
        available_motors: list[tuple[str, str]] = []
) -> None:
    sock = socket.socket(
        family=socket.AF_INET,
        type=socket.SOCK_STREAM
    )
    sock.bind((host, port))
    sock.listen()
    print(f'Motor  server listening on {host}:{port}')
    try:
        while True:
            conn, addr = sock.accept()
            threading.Thread(
                target=handle_client,
                args=(conn, addr, available_motors),
                daemon=True
            ).start()

    except KeyboardInterrupt:
        print('Motor server shutting down')
    finally:
        sock.close()
        # for dev in devices:
        #     dev.disconnect()

def main() -> None:
    available_motors = []

    available_motors.extend(dummy_motor.list_dummy_motors())

    # if thorlabs_motor.is_available():
    #     motors.extend(thorlabs_motor.list_kinesis_motors())

    # if k10cr2_motor.is_available():
    #     motors.extend(k10cr2_motor.list_k10cr2_motors())

    # if standa_motor.is_available():
    #     motors.extend(standa_motor.list_standa_motors())

    start_server(
        available_motors=available_motors,
        port=5001
    )

if __name__ == '__main__':
    main()