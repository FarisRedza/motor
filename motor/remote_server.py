import socket
import threading
import struct
import enum
import json

from . import thorlabs_motor
from . import elliptec_motor
from . import base_motor

class Command(enum.IntEnum):
    LIST_DEVICES = 1
    DEVICE_INFO = 2
    GET_STATUS = 3
    STOP = 4
    MOVE_BY = 5
    MOVE_TO = 6
    THREADED_MOVE_BY = 7
    THREADED_MOVE_TO = 8
    JOG = 9
    GET_POSITION = 10

class Response(enum.IntEnum):
    ERROR = 0
    LIST_DEVICES = 1
    DEVICE_INFO = 2
    STATUS = 3
    POSITION = 4

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
        address
) -> None:
    with sock:
        try:
            while True:
                try:
                    command, args = receive_command(sock=sock)
                except (ValueError, ConnectionError) as e:
                    print(f'[{address}] Disconnected: {e}')
                    break

                if command == Command.LIST_DEVICES:
                    devices = [(m.device_info.serial_number, m.device_info.device_name) for m in motors]
                    send_payload(
                        sock=sock,
                        payload=json.dumps(devices).encode(encoding='utf-8'),
                        response_id=Response.LIST_DEVICES
                    )


                elif args:
                    serial_number = str(args[0])
                    device = next(
                        (d for d in motors if d.device_info.serial_number == serial_number),
                        None
                    )
                    if not device:
                        send_message(
                            sock=sock,
                            message=f'Device {serial_number} not found',
                            response_id=Response.ERROR
                        )
                        continue

                    match command:
                        case Command.DEVICE_INFO:
                            send_payload(
                                sock=sock,
                                payload=device.device_info.serialise(),
                                response_id=Response.DEVICE_INFO
                            )

                        case Command.GET_STATUS:
                            send_message(
                                sock=sock,
                                message=f'{device.position},{device.direction.value},{device.acceleration},{device.max_velocity}',
                                response_id=Response.STATUS
                            )

                        case Command.GET_POSITION:
                            send_message(
                                sock=sock,
                                message=f'{device.position},{device.direction.value},{device.acceleration},{device.max_velocity}',
                                response_id=Response.POSITION
                            )

                        case Command.STOP:
                            device.stop()
                            send_message(
                                sock=sock,
                                message=f'{device.direction.value}',
                                response_id=Response.STATUS
                            )

                        case Command.MOVE_BY:
                            if len(args) < 2:
                                send_message(
                                    sock=sock,
                                    message='No angle provided',
                                    response_id=Response.ERROR
                                )
                                continue
                            try:
                                angle=float(args[1])
                                acceleration=float(args[2]) if args[2] else device.acceleration
                                max_velocity=float(args[3]) if args[3] else device.max_velocity
                                device.threaded_move_by(
                                    angle=angle,
                                    acceleration=acceleration,
                                    max_velocity=max_velocity
                                )
                                send_message(
                                    sock=sock,
                                    message=f'{angle},{device.direction.value},{acceleration},{max_velocity}',
                                    response_id=Response.STATUS
                                )
                            except Exception as e:
                                send_message(
                                    sock=sock,
                                    message=str(e),
                                    response_id=Response.ERROR
                                )

                        case Command.MOVE_TO:
                            if len(args) < 2:
                                send_message(
                                    sock=sock,
                                    message='No position provided',
                                    response_id=Response.ERROR
                                )
                                continue
                            try:
                                position=float(args[1])
                                acceleration=float(args[2]) if args[2] else device.acceleration
                                max_velocity=float(args[3]) if args[3] else device.max_velocity
                                device.threaded_move_to(
                                    position=position,
                                    acceleration=acceleration,
                                    max_velocity=max_velocity
                                )
                                send_message(
                                    sock=sock,
                                    message=f'{position},{device.direction.value},{acceleration},{max_velocity}',
                                    response_id=Response.STATUS
                                )
                            except Exception as e:
                                send_message(
                                    sock=sock,
                                    message=str(e),
                                    response_id=Response.ERROR
                                )

                        case Command.JOG:
                            if len(args) < 2:
                                send_message(
                                    sock=sock,
                                    message='No direction provided',
                                    response_id=Response.ERROR
                                )
                                continue
                            try:
                                direction=base_motor.MotorDirection(value=args[1])
                                acceleration=float(args[2]) if args[2] else device.acceleration
                                max_velocity=float(args[3]) if args[3] else device.max_velocity
                                device.jog(
                                    direction=direction,
                                    acceleration=acceleration,
                                    max_velocity=max_velocity
                                )
                                send_message(
                                    sock=sock,
                                    message=f'{device.direction.value},{acceleration},{max_velocity}',
                                    response_id=Response.STATUS
                                )
                            except Exception as e:
                                send_message(
                                    sock=sock,
                                    message=str(e),
                                    response_id=Response.ERROR
                                )

                        case _:
                            send_message(
                                sock=sock,
                                message=f'Unsupported command: {command}',
                                response_id=Response.ERROR
                            )
                    
                else:
                    send_message(
                        sock=sock,
                        message=f'No arguments provided',
                        response_id=Response.ERROR    
                    )

        except Exception as e:
            print(f'[{address}] Unexpected error: {e}')
        finally:
            print(f'Disconnected from {address}')

def start_server(
        host: str = '0.0.0.0',
        port: int = 5002
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
                args=(conn, addr),
                daemon=True
            ).start()

    except KeyboardInterrupt:
        print('Motor server shutting down')
    finally:
        sock.close()
        # for dev in devices:
        #     dev.disconnect()

if __name__ == '__main__':
    motors = thorlabs_motor.get_all_motors() + elliptec_motor.get_all_motors()
    start_server()