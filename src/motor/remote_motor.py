from __future__ import annotations

import socket
import threading
import typing
from collections.abc import Callable

from motor import base_motor
from motor.remote_protocol import receive_message, send_message

PositionCallback = Callable[[float], None]


class RemoteMotorError(RuntimeError):
    pass


def list_motors(
    *,
    host: str,
    port: int,
    timeout: float = 5.0,
) -> list[tuple[str, str]]:
    """Return currently available remote motors."""
    with socket.create_connection((host, port), timeout=timeout) as sock:
        send_message(sock, {
            'id': 1,
            'command': 'list_motors',
            'args': {},
        })
        response = receive_message(sock)
        if not response.get('ok'):
            error = response.get('error', {})
            raise RemoteMotorError(str(error.get('message', error)))

        motors = response['result']['motors']
        return [
            (str(item['serial_number']), str(item['device_name']))
            for item in motors
            if bool(item['available'])
        ]


class RemoteMotor(base_motor.Motor):
    """A local-looking proxy for one exclusively reserved server motor.

    Position and movement properties read from a local cache. A background
    thread refreshes that cache using one small get_state request per interval.
    All socket request/response pairs are protected by _request_lock, so motion
    commands and tracking polls cannot consume one another's responses.
    """

    def __init__(
        self,
        serial_number: str,
        *,
        host: str,
        port: int,
        tracking_interval: float = 0.1,
        timeout: float = 5.0,
        start_tracking: bool = True,
    ) -> None:
        if tracking_interval <= 0:
            raise ValueError('tracking_interval must be positive')

        self.host = host
        self.port = port
        self._tracking_interval = tracking_interval
        self._timeout = timeout

        self._state_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._tracking_stop_event = threading.Event()
        self._tracking_thread: typing.Optional[threading.Thread] = None
        self._position_callbacks: list[PositionCallback] = []
        self._next_request_id = 1
        self._closed = False

        self._position = 0.0
        self._is_moving = False
        self._direction = base_motor.MotorDirection.IDLE
        self._step_size = 0.0
        self._acceleration = 0.0
        self._max_velocity = 0.0
        self._tracking_error: typing.Optional[Exception] = None

        self._sock = socket.create_connection(
            (self.host, self.port),
            timeout=self._timeout,
        )
        self._sock.settimeout(self._timeout)

        try:
            result = self._request(
                'reserve',
                serial_number=serial_number,
            )
        except Exception:
            self._sock.close()
            raise

        self.device_info = base_motor.DeviceInfo(
            **result['device_info']
        )
        self._update_cached_state(result['state'])

        if start_tracking:
            self.start_tracking()

    @property
    def position(self) -> float:
        with self._state_lock:
            return self._position

    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            return self._is_moving

    @property
    def direction(self) -> base_motor.MotorDirection:
        with self._state_lock:
            return self._direction

    @property
    def step_size(self) -> float:
        with self._state_lock:
            return self._step_size

    @property
    def acceleration(self) -> float:
        with self._state_lock:
            return self._acceleration

    @property
    def max_velocity(self) -> float:
        with self._state_lock:
            return self._max_velocity

    @property
    def tracking_error(self) -> typing.Optional[Exception]:
        with self._state_lock:
            return self._tracking_error

    @property
    def is_tracking(self) -> bool:
        thread = self._tracking_thread
        return thread is not None and thread.is_alive()

    def add_position_callback(self, callback: PositionCallback) -> None:
        with self._state_lock:
            if callback not in self._position_callbacks:
                self._position_callbacks.append(callback)

    def remove_position_callback(self, callback: PositionCallback) -> None:
        with self._state_lock:
            try:
                self._position_callbacks.remove(callback)
            except ValueError:
                pass

    def start_tracking(self) -> None:
        if self.is_tracking:
            return
        if self._closed:
            raise RuntimeError('Motor is disconnected')

        self._tracking_stop_event.clear()
        self._tracking_thread = threading.Thread(
            target=self._tracking_loop,
            name=f'remote-motor-tracker-{self.device_info.serial_number}',
            daemon=True,
        )
        self._tracking_thread.start()

    def stop_tracking(self) -> None:
        thread = self._tracking_thread
        if thread is None:
            return

        self._tracking_stop_event.set()
        if thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self._tracking_interval * 4))
        self._tracking_thread = None

    def refresh_position(self) -> float:
        state = self._request('get_state')
        self._update_cached_state(state)
        return self.position

    def move_by(
        self,
        angle: float,
        acceleration: typing.Optional[float] = None,
        max_velocity: typing.Optional[float] = None,
    ) -> None:
        state = self._request(
            'move_by',
            angle=angle,
            acceleration=acceleration,
            max_velocity=max_velocity,
        )
        self._update_cached_state(state)

    def move_to(
        self,
        position: float,
        acceleration: typing.Optional[float] = None,
        max_velocity: typing.Optional[float] = None,
    ) -> None:
        state = self._request(
            'move_to',
            position=position,
            acceleration=acceleration,
            max_velocity=max_velocity,
        )
        self._update_cached_state(state)

    def jog(
        self,
        direction: base_motor.MotorDirection,
        acceleration: typing.Optional[float] = None,
        max_velocity: typing.Optional[float] = None,
    ) -> None:
        state = self._request(
            'jog',
            direction=direction.value,
            acceleration=acceleration,
            max_velocity=max_velocity,
        )
        self._update_cached_state(state)

    def stop(self) -> None:
        state = self._request('stop')
        self._update_cached_state(state)

    def update_settings(
        self,
        acceleration: float,
        max_velocity: float,
    ) -> None:
        state = self._request(
            'update_settings',
            acceleration=acceleration,
            max_velocity=max_velocity,
        )
        self._update_cached_state(state)

    def disconnect(self) -> None:
        if self._closed:
            return

        self.stop_tracking()
        try:
            self._request('disconnect')
        except (ConnectionError, OSError, RemoteMotorError):
            # The server also releases the reservation when the socket vanishes.
            pass
        finally:
            self._closed = True
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()

    def __enter__(self) -> 'RemoteMotor':
        return self

    def __exit__(
        self,
        exc_type: object,
        exc_value: object,
        traceback: object,
    ) -> None:
        self.disconnect()

    def _request(
        self,
        command: str,
        **args: typing.Any,
    ) -> dict[str, typing.Any]:
        if self._closed:
            raise RuntimeError('Motor is disconnected')

        with self._request_lock:
            request_id = self._next_request_id
            self._next_request_id += 1

            send_message(self._sock, {
                'id': request_id,
                'command': command,
                'args': args,
            })
            response = receive_message(self._sock)

            if response.get('id') != request_id:
                raise RemoteMotorError('Mismatched response id')
            if not response.get('ok'):
                error = response.get('error', {})
                error_type = error.get('type', 'RemoteError')
                message = error.get('message', 'Unknown server error')
                raise RemoteMotorError(f'{error_type}: {message}')

            result = response.get('result', {})
            if not isinstance(result, dict):
                raise RemoteMotorError('Invalid result from server')
            return result

    def _tracking_loop(self) -> None:
        while not self._tracking_stop_event.is_set():
            try:
                state = self._request('get_state')
                self._update_cached_state(state)
                with self._state_lock:
                    self._tracking_error = None
                    position = self._position
                    callbacks = tuple(self._position_callbacks)

                for callback in callbacks:
                    try:
                        callback(position)
                    except Exception:
                        pass
            except Exception as error:
                with self._state_lock:
                    self._tracking_error = error

            self._tracking_stop_event.wait(self._tracking_interval)

    def _update_cached_state(
        self,
        state: dict[str, typing.Any],
    ) -> None:
        remote_error = state.get('tracking_error')
        with self._state_lock:
            self._position = float(state['position'])
            self._is_moving = bool(state['is_moving'])
            self._direction = base_motor.MotorDirection(
                str(state['direction'])
            )
            self._step_size = float(state['step_size'])
            self._acceleration = float(state['acceleration'])
            self._max_velocity = float(state['max_velocity'])
            if remote_error is not None:
                self._tracking_error = RemoteMotorError(
                    f'Server-side tracking error: {remote_error}'
                )

if __name__ == '__main__':
    motor = RemoteMotor(
        serial_number='37398',
        host='127.0.0.1',
        port=5001
    )
    motor.move_by(angle=90)
    while motor.is_moving:
        print(
            f'\rPosition: {motor.position:8.3f}°',
            end='',
            flush=True,
        )
        threading.Event().wait(0.5)
    motor.stop()

    print(f'\nFinal position: {motor.position:.3f}°')
