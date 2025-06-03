import sys
import os
import socket
import threading
import json
import dataclasses

sys.path.append(
    os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        os.path.pardir
    ))
)
import motor.thorlabs_motor as thorlabs_motor
import motor.base_motor as base_motor

def handle_client(connection: socket.socket, address) -> None:
    print(f'Connected by {address}')
    with connection:
        while True:
            try:
                data = connection.recv(1024).decode()
                if not data:
                    break

                request = json.loads(data)
                command = str(request['command'])
                print(f'Command received: {command}')
                match command:
                    case base_motor.Commands.LIST_MOTORS.value:
                        response = {'motors': [dataclasses.asdict(m.device_info) for m in motors]}

                    case base_motor.Commands.GET_POSITION.value:
                        serial_number = str(request.get('serial_number'))
                        try:
                            motor = next(m for m in motors if m.device_info.serial_number == serial_number)
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

                    case base_motor.Commands.STOP.value:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(m for m in motors if m.device_info.serial_number == serial_number)
                        except:
                            response = {'error': f'Motor {serial_number} not found'}
                        else:
                            motor.stop()
                            response = {
                                'status': f'Stopping {serial_number}',
                                'moving': motor.is_moving,
                            }

                    case base_motor.Commands.MOVE_BY.value:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(m for m in motors if m.device_info.serial_number == serial_number)
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

                    case base_motor.Commands.MOVE_TO.value:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(m for m in motors if m.device_info.serial_number == serial_number)
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

                    case base_motor.Commands.JOG.value:
                        serial_number = str(request['serial_number'])
                        try:
                            motor = next(m for m in motors if m.device_info.serial_number == serial_number)
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

                print(response)
                connection.sendall((json.dumps(response) + "\n").encode())

            except Exception as e:
                connection.sendall(json.dumps({'error': str(e)}).encode())
                break

def start_server(host: str = '0.0.0.0', port: int = 5002) -> None:
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
                # daemon=True
            ).start()
    except KeyboardInterrupt:
        server.close()

if __name__ == '__main__':
    motors = thorlabs_motor.list_motors()
    start_server()