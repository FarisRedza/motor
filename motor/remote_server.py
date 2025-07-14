import socket
import threading
import json
import dataclasses

from . import thorlabs_motor
from . import base_motor

def handle_client(
        connection: socket.socket,
        address
) -> None:
    print(f'Connected by {address}')
    with connection:
        try:
            while True:
                data = connection.recv(1024).decode()
                if not data:
                    break

                request = json.loads(data)
                command = str(request['command'])
                match base_motor.Commands(command):
                    case base_motor.Commands.LIST_MOTORS:
                        response = {'motors': [dataclasses.asdict(m.device_info) for m in motors]}

                    case base_motor.Commands.GET_POSITION:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(
                                m for m in motors if m.device_info.serial_number == serial_number
                            )
                        except:
                            response = {'error': f'Motor {serial_number} not found'}
                        else:
                            response = {
                                'position': motor.position,
                                'moving': motor.is_moving,
                                'direction': motor.direction.value,
                                'acceleration': motor.acceleration,
                                'max_velocity': motor.max_velocity
                            }

                    case base_motor.Commands.STOP:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(
                                m for m in motors if m.device_info.serial_number == serial_number
                            )
                        except:
                            response = {'error': f'Motor {serial_number} not found'}
                        else:
                            motor.stop()
                            response = {
                                'status': f'Stopping {serial_number}',
                                'moving': motor.is_moving,
                            }

                    case base_motor.Commands.MOVE_BY:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(
                                m for m in motors if m.device_info.serial_number == serial_number
                            )
                        except:
                            response = {'error': f'Motor {serial_number} not found'}
                        else:
                            angle = float(request['angle'])
                            motor.threaded_move_by(
                                angle=angle,
                                acceleration=float(request['acceleration']),
                                max_velocity=float(request['max_velocity'])
                            )

                            response = {
                                'status': f'Moving motor {serial_number} by {angle}',
                                'moving': motor.is_moving
                            }

                    case base_motor.Commands.MOVE_TO:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(
                                m for m in motors if m.device_info.serial_number == serial_number
                            )
                        except:
                            response = {'error': f'Motor {serial_number} not found'}
                        else:
                            position = float(request['position'])
                            motor.threaded_move_to(
                                position=position,
                                acceleration=float(request['acceleration']),
                                max_velocity=float(request['max_velocity'])
                            )

                            response = {
                                'status': f'Moving motor {serial_number} to {position}',
                                'moving': motor.is_moving
                            }

                    case base_motor.Commands.JOG:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(
                                m for m in motors if m.device_info.serial_number == serial_number
                            )
                        except:
                            response = {'error': f'Motor {serial_number} not found'}
                        else:
                            motor.jog(
                                direction=base_motor.MotorDirection(request['direction']),
                                acceleration=float(request['acceleration']),
                                max_velocity=float(request['max_velocity'])
                            )

                            response = {
                                'status': f'Jogging motor {serial_number}',
                                'moving': motor.is_moving
                            }

                    case _:
                        response = {'error': 'Unkown command'}

                connection.sendall((json.dumps(response) + "\n").encode())

        except ConnectionResetError:
            print(f'Connection lost with {address}')

        finally:
            connection.close()
            print(f'Disconnected from {address}')
        

def start_server(
        host: str = '0.0.0.0',
        port: int = 5002
) -> None:
    server = socket.socket(
        family=socket.AF_INET,
        type=socket.SOCK_STREAM
    )
    try:
        server.bind((host, port))
        server.listen()

        print(f'Motor server listening on {host}:{port}')
        while True:
            conn, addr = server.accept()
            threading.Thread(
                target=handle_client,
                args=(conn, addr),
            ).start()
    except KeyboardInterrupt:
        server.close()

if __name__ == '__main__':
    motors = thorlabs_motor.list_motors()
    start_server()