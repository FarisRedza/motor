from __future__ import annotations

import dataclasses
import logging
import socket
import threading
import typing
from collections.abc import Callable

from motor import base_motor
from motor.remote_protocol import ProtocolError, receive_message, send_message

MotorFactory = Callable[[], base_motor.Motor]


@dataclasses.dataclass(frozen=True)
class MotorRegistration:
    serial_number: str
    device_name: str
    factory: MotorFactory


@dataclasses.dataclass
class _Reservation:
    owner_id: int
    motor: base_motor.Motor


class MotorRegistry:
    """Thread-safe collection of available and reserved motors.

    A motor object is constructed only when a client reserves it. This avoids
    opening every physical device at server startup and gives each reservation
    exclusive ownership of the local motor instance.
    """

    def __init__(self, registrations: list[MotorRegistration]) -> None:
        self._registrations = {
            registration.serial_number: registration
            for registration in registrations
        }
        if len(self._registrations) != len(registrations):
            raise ValueError('Motor serial numbers must be unique')

        self._reservations: dict[str, _Reservation] = {}
        self._lock = threading.Lock()

    def list_motors(self) -> list[dict[str, typing.Any]]:
        with self._lock:
            return [
                {
                    'serial_number': registration.serial_number,
                    'device_name': registration.device_name,
                    'available': serial_number not in self._reservations,
                }
                for serial_number, registration
                in self._registrations.items()
            ]

    def reserve(self, serial_number: str, owner_id: int) -> base_motor.Motor:
        with self._lock:
            registration = self._registrations.get(serial_number)
            if registration is None:
                raise KeyError(f"Motor '{serial_number}' was not found")
            if serial_number in self._reservations:
                raise RuntimeError(
                    f"Motor '{serial_number}' is already reserved"
                )
            # Construct while holding the lock so two clients cannot both open
            # the same physical device between the availability check and the
            # reservation being recorded.
            motor = registration.factory()
            self._reservations[serial_number] = _Reservation(
                owner_id=owner_id,
                motor=motor,
            )
            return motor

    def release(self, serial_number: str, owner_id: int) -> None:
        reservation: typing.Optional[_Reservation]
        with self._lock:
            reservation = self._reservations.get(serial_number)
            if reservation is None:
                return
            if reservation.owner_id != owner_id:
                raise RuntimeError('Client does not own this reservation')
            del self._reservations[serial_number]

        # Do not hold the registry lock during hardware I/O.
        try:
            reservation.motor.stop()
        except Exception:
            logging.exception('Failed to stop motor during release')
        try:
            reservation.motor.disconnect()
        except Exception:
            logging.exception('Failed to disconnect motor during release')


class MotorServer:
    def __init__(
        self,
        registrations: list[MotorRegistration],
        *,
        host: str = '0.0.0.0',
        port: int = 5001,
        client_timeout: typing.Optional[float] = 30.0,
    ) -> None:
        self.host = host
        self.port = port
        self.client_timeout = client_timeout
        self.registry = MotorRegistry(registrations=registrations)
        self._stop_event = threading.Event()
        self._listen_socket: typing.Optional[socket.socket] = None

    def serve_forever(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listen_socket:
            listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listen_socket.bind((self.host, self.port))
            listen_socket.listen()
            listen_socket.settimeout(1.0)
            self._listen_socket = listen_socket

            server_start_msg = f'Motor server listening on {self.host}:{self.port}'
            print(server_start_msg)
            logging.info(server_start_msg)
            while not self._stop_event.is_set():
                try:
                    client_socket, address = listen_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise

                threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, address),
                    name=f'motor-client-{address[0]}:{address[1]}',
                    daemon=True,
                ).start()

    def shutdown(self) -> None:
        self._stop_event.set()
        listen_socket = self._listen_socket
        if listen_socket is not None:
            listen_socket.close()

    def _handle_client(
        self,
        client_socket: socket.socket,
        address: tuple[str, int],
    ) -> None:
        owner_id = id(client_socket)
        reserved_serial: typing.Optional[str] = None
        motor: typing.Optional[base_motor.Motor] = None

        with client_socket:
            client_socket.settimeout(self.client_timeout)
            try:
                while True:
                    request = receive_message(sock=client_socket)
                    request_id = request.get('id')
                    command = request.get('command')
                    args = request.get('args', {})

                    if not isinstance(request_id, int):
                        raise ProtocolError('Request id must be an integer')
                    if not isinstance(command, str):
                        raise ProtocolError('Command must be a string')
                    if not isinstance(args, dict):
                        raise ProtocolError('args must be an object')

                    try:
                        result, should_close = self._dispatch(
                            command=command,
                            args=args,
                            owner_id=owner_id,
                            motor=motor,
                            reserved_serial=reserved_serial,
                        )

                        if command == 'reserve':
                            reserved_serial = str(args['serial_number'])
                            motor = result.pop('_motor')
                        send_message(client_socket, {
                            'id': request_id,
                            'ok': True,
                            'result': result,
                        })
                        if should_close:
                            break
                    except Exception as error:
                        send_message(client_socket, {
                            'id': request_id,
                            'ok': False,
                            'error': {
                                'type': type(error).__name__,
                                'message': str(error),
                            },
                        })
            except (ConnectionError, socket.timeout):
                logging.info('Client %s disconnected', address)
            except Exception:
                logging.exception('Client %s failed', address)
            finally:
                if reserved_serial is not None:
                    self.registry.release(reserved_serial, owner_id)

    def _dispatch(
        self,
        *,
        command: str,
        args: dict[str, typing.Any],
        owner_id: int,
        motor: typing.Optional[base_motor.Motor],
        reserved_serial: typing.Optional[str],
    ) -> tuple[dict[str, typing.Any], bool]:
        if command == 'list_motors':
            return {'motors': self.registry.list_motors()}, False

        if command == 'reserve':
            if motor is not None:
                raise RuntimeError('This client already reserved a motor')
            serial_number = str(args['serial_number'])
            new_motor = self.registry.reserve(
                serial_number=serial_number,
                owner_id=owner_id
            )
            return {
                '_motor': new_motor,
                'device_info': dataclasses.asdict(new_motor.device_info),
                'state': self._motor_state(motor=new_motor),
            }, False

        if motor is None or reserved_serial is None:
            raise RuntimeError('Reserve a motor before issuing this command')
        else:
            match command: # these commands require a reserved motor
                case 'get_state':
                    return self._motor_state(motor=motor), False
                case 'move_by':
                    motor.move_by(
                        angle=float(args['angle']),
                        acceleration=self._optional_float(
                            value=args.get('acceleration')
                        ),
                        max_velocity=self._optional_float(
                            value=args.get('max_velocity')
                        ),
                    )
                    return self._motor_state(motor=motor), False
                case 'move_to':
                    motor.move_to(
                        position=float(args['position']),
                        acceleration=self._optional_float(
                            value=args.get('acceleration')
                        ),
                        max_velocity=self._optional_float(
                            value=args.get('max_velocity')
                        ),
                    )
                    return self._motor_state(motor=motor), False
                case 'jog':
                    motor.jog(
                        direction=base_motor.MotorDirection(
                            value=str(args['direction'])
                        ),
                        acceleration=self._optional_float(
                            value=args.get('acceleration')
                        ),
                        max_velocity=self._optional_float(
                            value=args.get('max_velocity')
                        ),
                    )
                    return self._motor_state(motor=motor), False
                case 'stop':
                    motor.stop()
                    return self._motor_state(motor=motor), False
                case 'update_settings':
                    motor.update_settings(
                        acceleration=float(args['acceleration']),
                        max_velocity=float(args['max_velocity']),
                    )
                    return self._motor_state(motor=motor), False
                case 'disconnect':
                    self.registry.release(
                        serial_number=reserved_serial,
                        owner_id=owner_id
                    )
                    return {}, True

                case _:
                    raise ValueError(f'Unknown command: {command}')

    @staticmethod
    def _optional_float(value: typing.Any) -> typing.Optional[float]:
        return None if value is None else float(value)

    @staticmethod
    def _motor_state(motor: base_motor.Motor) -> dict[str, typing.Any]:
        error = motor.tracking_error
        direction = getattr(
            motor,
            'direction',
            base_motor.MotorDirection.IDLE,
        )
        if not isinstance(direction, base_motor.MotorDirection):
            direction = base_motor.MotorDirection.IDLE

        return {
            'position': float(motor.position),
            'is_moving': bool(motor.is_moving),
            'direction': direction.value,
            'step_size': float(getattr(motor, 'step_size', 0.0)),
            'acceleration': float(getattr(motor, 'acceleration', 0.0)),
            'max_velocity': float(getattr(motor, 'max_velocity', 0.0)),
            'tracking_error': None if error is None else str(error),
        }


def main() -> None:
    from motor import dummy_motor, standa_motor, thorlabs_motor, k10cr2_motor

    registrations = []

    # registrations.extend(MotorRegistration(
    #         serial_number=serial_number,
    #         device_name=device_name,
    #         factory=lambda sn=serial_number: dummy_motor.DummyMotor(
    #             # serial_number=sn
    #         ),
    #     )
    #     for serial_number, device_name in dummy_motor.list_dummy_motors()
    # )

    registrations.extend(MotorRegistration(
            serial_number=serial_number,
            device_name=device_name,
            factory=lambda sn=serial_number: standa_motor.StandaMotor(
                serial_number=sn
            ),
        )
        for serial_number, device_name in standa_motor.list_standa_motors()
        if standa_motor.is_available()
    )

    registrations.extend(MotorRegistration(
            serial_number=serial_number,
            device_name=device_name,
            factory=lambda sn=serial_number: thorlabs_motor.ThorlabsMotor(
                serial_number=sn
            ),
        )
        for serial_number, device_name in thorlabs_motor.list_kinesis_motors()
        if thorlabs_motor.is_available()
    )

    registrations.extend(MotorRegistration(
            serial_number=serial_number,
            device_name=device_name,
            factory=lambda sn=serial_number: k10cr2_motor.K10CR2Motor(
                serial_number=sn
            ),
        )
        for serial_number, device_name in k10cr2_motor.list_k10cr2_motors()
        if k10cr2_motor.is_available()
    )

    server = MotorServer(
        registrations=registrations,
        host='0.0.0.0',
        port=5001
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()