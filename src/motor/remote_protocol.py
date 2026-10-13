from __future__ import annotations

import json
import socket
import struct
import typing

_HEADER = struct.Struct('!I')
_MAX_MESSAGE_SIZE = 1024 * 1024


class ProtocolError(RuntimeError):
    pass


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError('Socket closed by peer')
        data.extend(chunk)
    return bytes(data)


def send_message(sock: socket.socket, message: dict[str, typing.Any]) -> None:
    payload = json.dumps(
        message,
        separators=(',', ':'),
        ensure_ascii=False,
    ).encode('utf-8')

    if len(payload) > _MAX_MESSAGE_SIZE:
        raise ProtocolError('Message is too large')

    sock.sendall(_HEADER.pack(len(payload)) + payload)


def receive_message(sock: socket.socket) -> dict[str, typing.Any]:
    payload_size = _HEADER.unpack(_recv_exact(sock, _HEADER.size))[0]
    if payload_size > _MAX_MESSAGE_SIZE:
        raise ProtocolError(
            f'Message is too large: {payload_size} bytes'
        )

    payload = _recv_exact(sock, payload_size)
    try:
        message = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ProtocolError('Invalid JSON message') from error

    if not isinstance(message, dict):
        raise ProtocolError('Protocol message must be a JSON object')
    return message
