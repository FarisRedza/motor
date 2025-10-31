import sys
import pathlib
import platform
import dataclasses
import struct
import typing
import enum
import time
import threading

import serial

sys.path.append(str(pathlib.Path.cwd()))
from motor import base_motor
from motor import thorlabs_motor

'''
apt format | encoding    | format
word       | 16-bit uint | H
short      | 16-bit int  | h
dword      | 32-bit uint | I
long       | 32-bit int  | i
char       | 1 byte      | c
char[N]    | n bytes     | nc
'''

class MotorStatusBits(enum.IntFlag):
    CWHARDLIMIT      = 0x00000001  # Forward (CW) hardware limit
    CCWHARDLIMIT     = 0x00000002  # Reverse (CCW) hardware limit
    CWSOFTLIMIT      = 0x00000004  # Forward (CW) software limit
    CCWSOFTLIMIT     = 0x00000008  # Reverse (CCW) software limit
    INMOTIONCW       = 0x00000010  # In motion (CW)
    INMOTIONCCW      = 0x00000020  # In motion (CCW)
    JOGGINGCW        = 0x00000040  # Jogging CW
    JOGGINGCCW       = 0x00000080  # Jogging CCW
    CONNECTED        = 0x00000100  # Motor connected
    HOMING           = 0x00000200  # Homing in progress
    HOMED            = 0x00000400  # Homing complete
    TRACKING         = 0x00001000  # Tracking position
    SETTLED          = 0x00002000  # Motion settled
    POSITIONERROR    = 0x00004000  # Position error present
    INSTRERROR       = 0x00008000  # Instruction error
    INTERLOCK        = 0x00010000  # Interlock enabled
    OVERTEMP         = 0x00020000  # Over-temperature
    BUSVOLTFAULT     = 0x00040000  # Bus-voltage fault
    COMMUTATIONERROR = 0x00080000  # Commutation error
    OVERCURRENT      = 0x04000000  # Over-current
    BUSCURRENTFAULT  = 0x08000000  # Bus-current fault
    POWEROK          = 0x10000000  # Power supplies OK
    ACTIVE           = 0x20000000  # Motion command executing
    ERROR            = 0x40000000  # General error condition
    ENABLED          = 0x80000000  # Motor outputs enabled

@dataclasses.dataclass
class StatusUpdate:
    chan_ident: int
    position: int
    velocity: int
    status_bits: int

    FORMAT = '<hiiI'
    SIZE = struct.calcsize(FORMAT)  # 14 bytes

    @classmethod
    def unpack(cls, data: bytes) -> 'StatusUpdate':
        if len(data) < 20:
            raise ValueError(f'Incomplete StatusUpdate message ({len(data)} bytes)')
        chan_ident, position, velocity, status_bits = struct.unpack_from(
            cls.FORMAT,
            buffer=data,
            offset=6
        )
        return cls(chan_ident, position, velocity, status_bits)

    def flags(self) -> MotorStatusBits:
        return MotorStatusBits(self.status_bits)

    def active_flags(self) -> list[typing.Optional[str]]:
        return [flag.name for flag in MotorStatusBits if self.status_bits & flag]

@dataclasses.dataclass
class DeviceInfo:
    manufacturer: str
    model: str
    serial_number: str
    firmware_version: str

@dataclasses.dataclass
class JogParametersInternal:
    chan_ident: int
    jog_mode: int
    jog_step_size: int
    jog_min_velocity: int
    jog_acceleration: int
    jog_max_velocity: int
    jog_stop_mode: int

    FORMAT = '<HHiiiiH'
    SIZE = struct.calcsize(FORMAT)

    def pack(self) -> bytes:
        return struct.pack(
            self.FORMAT,
            self.chan_ident,
            self.jog_mode,
            self.jog_step_size,
            self.jog_min_velocity,
            self.jog_acceleration,
            self.jog_max_velocity,
            self.jog_stop_mode,
        )
    
    @classmethod
    def unpack(cls, data: bytes, header: bool = True) -> 'JogParametersInternal':
        header_offset = 0 if header else -APTHeader.SIZE

        packet_size = cls.SIZE + header_offset
        if len(data) < packet_size:
            raise ValueError(
                f'Incomplete JogParameters (need {packet_size} bytes, received {len(data)})'
            )

        chan_ident = struct.unpack_from('<H', buffer=data, offset=6+header_offset)[0]
        jog_mode = struct.unpack_from('<H', buffer=data, offset=8+header_offset)[0]
        jog_step_size = struct.unpack_from('<i', buffer=data, offset=10+header_offset)[0]
        jog_min_velocity = struct.unpack_from('<i', buffer=data, offset=14+header_offset)[0]
        jog_acceleration = struct.unpack_from('<i', buffer=data, offset=18+header_offset)[0]
        jog_max_velocity = struct.unpack_from('<i', buffer=data, offset=22+header_offset)[0]
        jog_stop_mode = struct.unpack_from('<H', buffer=data, offset=26+header_offset)[0]

        return cls(
            chan_ident=chan_ident,
            jog_mode=jog_mode,
            jog_step_size=jog_step_size,
            jog_min_velocity=jog_min_velocity,
            jog_acceleration=jog_acceleration,
            jog_max_velocity=jog_max_velocity,
            jog_stop_mode=jog_stop_mode
        )

@dataclasses.dataclass
class JogParameters:
    mode: typing.Literal['continuous', 'step']
    step_size: float
    min_velocity: float
    acceleration: float
    max_velocity: float
    stop_mode: typing.Literal['immediate','profiled']
    _chan_ident: int

    def to_internal(
            self,
            scale: tuple[float, float, float]
    ) -> JogParametersInternal:
        return JogParametersInternal(
            chan_ident=self._chan_ident,
            jog_mode=1 if self.mode == 'continuous' else 2,
            jog_step_size=int(self.step_size*scale[0]),
            jog_min_velocity=int(self.min_velocity*scale[1]),
            jog_acceleration=int(self.acceleration*scale[2]),
            jog_max_velocity=int(self.max_velocity*scale[1]),
            jog_stop_mode=1 if self.stop_mode == 'immediate' else 2
        )

    @classmethod
    def from_internal(
            cls,
            jog_params_internal: JogParametersInternal,
            scale: tuple[float, float, float]
    ) -> 'JogParameters':
        return cls(
            _chan_ident=jog_params_internal.chan_ident,
            mode='continuous' if jog_params_internal.jog_mode == 1 else 'step',
            step_size=jog_params_internal.jog_step_size/scale[0],
            min_velocity=jog_params_internal.jog_min_velocity/scale[1],
            acceleration=jog_params_internal.jog_acceleration/scale[2],
            max_velocity=jog_params_internal.jog_max_velocity/scale[1],
            stop_mode='immediate' if jog_params_internal.jog_stop_mode == 1 else 'profiled'
        )

@dataclasses.dataclass
class VelocityParametersInternal:
    chan_ident: int
    vel_min_velocity: int
    vel_acceleration: int
    vel_max_velocity: int

    FORMAT = '<Hiii'
    SIZE = struct.calcsize(FORMAT)

    def pack(self) -> bytes:
        return struct.pack(
            self.FORMAT,
            self.chan_ident,
            self.vel_min_velocity,
            self.vel_acceleration,
            self.vel_max_velocity,
        )
    
    @classmethod
    def unpack(cls, data: bytes, header: bool = True) -> 'VelocityParametersInternal':
        header_offset = 0 if header else -APTHeader.SIZE

        packet_size = cls.SIZE + header_offset
        if len(data) < packet_size:
            raise ValueError(
                f'Incomplete JogParameters (need {packet_size} bytes, received {len(data)})'
            )

        chan_ident = struct.unpack_from('<H', buffer=data, offset=6+header_offset)[0]
        vel_min_velocity = struct.unpack_from('<i', buffer=data, offset=8+header_offset)[0]
        vel_acceleration = struct.unpack_from('<i', buffer=data, offset=12+header_offset)[0]
        vel_max_velocity = struct.unpack_from('<i', buffer=data, offset=16+header_offset)[0]

        return cls(
            chan_ident=chan_ident,
            vel_min_velocity=vel_min_velocity,
            vel_acceleration=vel_acceleration,
            vel_max_velocity=vel_max_velocity,
        )

@dataclasses.dataclass
class VelocityParameters:
    min_velocity: float
    acceleration: float
    max_velocity: float
    _chan_ident: int

    def to_internal(
            self,
            scale: tuple[float, float, float]
    ) -> VelocityParametersInternal:
        return VelocityParametersInternal(
            chan_ident=self._chan_ident,
            vel_min_velocity=int(self.min_velocity*scale[1]),
            vel_acceleration=int(self.acceleration*scale[2]),
            vel_max_velocity=int(self.max_velocity*scale[1]),
        )

    @classmethod
    def from_internal(
            cls,
            vel_params_internal: VelocityParametersInternal,
            scale: tuple[float, float, float]
    ) -> 'VelocityParameters':
        return cls(
            _chan_ident=vel_params_internal.chan_ident,
            min_velocity=vel_params_internal.vel_min_velocity/scale[1],
            acceleration=vel_params_internal.vel_acceleration/scale[2],
            max_velocity=vel_params_internal.vel_max_velocity/scale[1],
        )

@dataclasses.dataclass
class APTHeader:
    message_id: int
    param1: int = 0
    param2: int = 0
    dest: int = 0x01
    src: int = 0x50
    has_data: bool = False

    FORMAT = '<HHBB'
    SIZE = struct.calcsize(FORMAT)

    def pack(self, data: bytes = b'') -> bytes:
        if self.has_data:
            data_packet_length = len(data)
            return struct.pack(
                '<HHBB',
                self.message_id,
                data_packet_length,
                self.dest,
                self.src
            )
        else:
            return struct.pack(
                '<HBBBB',
                self.message_id,
                self.param1,
                self.param2,
                self.dest,
                self.src
            )

    @classmethod
    def unpack(cls, data: bytes) -> 'APTHeader':
        if len(data) < cls.SIZE:
            raise ValueError(f'Incomplete APT header (need {cls.SIZE} bytes)')
        msg_id = struct.unpack_from('<H', data, 0)[0]
        param1 = data[2]
        param2 = data[3]
        dest = data[4]
        src = data[5]
        return cls(
            message_id=msg_id,
            param1=param1,
            param2=param2,
            dest=dest,
            src=src
        )

class APTMotor():
    def __init__(
            self,
            conn: str,
            baudrate: int = 115200,
            timeout: float = 0.5,
            scale: typing.Literal['step', 'stage'] = 'step'
    ) -> None:
        self._serial = serial.Serial(
            port=conn,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            rtscts=True
        )
        self._flush_serial()
        self._scale = (1,1,1)
        self._moving = False
    
    def close(self) -> None:
        self._serial.close()

    def _flush_serial(self) -> None:
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._serial.flush()


    def _p2d(self, value, kind: typing.Literal['p','v','a'], scale: bool = True) -> int:
        idx = 'pva'.index(kind)
        scale = self._scale[idx] if scale else 1
        if scale != 1:
            value*=scale
        return int(value)
    
    def _d2p(self, value, kind: typing.Literal['p','v','a'], scale: bool = True) -> float:
        idx = 'pva'.index(kind)
        scale = self._scale[idx] if scale else 1
        if scale != 1:
            value/=scale
        return value

    def _hardware_get_info(self) -> tuple:
        self._mgmsg_hw_req_info()
        resp = self._mgmsg_hw_get_info()

        serial_number = struct.unpack_from('<i', buffer=resp, offset=6)[0]
        
        model_number = resp[10:18].decode()
        
        hw_type = struct.unpack_from('<H', buffer=resp, offset=18)[0]
        
        fw_minor, fw_interim, fw_major, _ = struct.unpack_from(
            '<BBBB',
            buffer=resp,
            offset=20
        )
        fw_version = f'{fw_major}.{fw_interim}.{fw_minor}'

        internal_data = resp[24:84]

        hw_version = struct.unpack_from('<H', buffer=resp, offset=84)[0]

        mod_state = struct.unpack_from('<H', buffer=resp, offset=86)[0]

        number_of_channels = struct.unpack_from('<H', buffer=resp, offset=88)[0]

        return (
            serial_number,
            model_number,
            hw_type,
            fw_version,
            internal_data,
            hw_version,
            mod_state,
            number_of_channels
        )
    
    def _get_position(self, scale: bool = True):
        position_d = self._get_motor_position_counter()
        position = position_d / self._scale[0]
        print(position)
    
    def _get_channel_enable_state(self, channel: int = 1) -> bool:
        self._mgmsg_mod_req_chanenablestate(chan_ident=channel)
        resp = self._mgmsg_mod_get_chanenablestate()
        state = resp[3]
        match state:
            case 1:
                return True
            case 2:
                return False
            case _:
                raise RuntimeError(f'Unknown channel state: {state}')

    def _get_motor_position_counter(self, channel: int = 1) -> int:
        self._mgmsg_mot_req_poscounter(chan_ident=channel)
        data = self._mgmsg_mot_get_poscounter(chan_ident=channel)
        position = struct.unpack_from(
            '<Hi',
            data,
            offset=6
        )[1]
        return position

    def _set_jog_params(
            self,
            jog_params: JogParameters
    ) -> None:
        jog_params_internal = jog_params.to_internal(scale=self._scale)
        self._mgmsg_mot_set_jogparams(data=jog_params_internal.pack())

    def _get_jog_params(self, channel: int = 1) -> JogParametersInternal:
        self._mgmsg_mot_req_jogparams(chan_ident=channel)
        resp = self._mgmsg_mot_get_jogparams()

        return JogParametersInternal.unpack(data=resp)
    
    def _set_vel_params(
            self,
            vel_params: VelocityParameters
    ) -> None:
        vel_params_internal = vel_params.to_internal(scale=self._scale)
        self._mgmsg_mot_set_velparams(data=vel_params_internal.pack())

    def _get_vel_params(self, channel: int = 1) -> VelocityParametersInternal:
        self._mgmsg_mot_req_velparams(chan_ident=channel)
        resp = self._mgmsg_mot_get_velparams()

        return VelocityParametersInternal.unpack(data=resp)

    def _get_potparams(self, channel: int = 1):
        '''
        Does not work with K10CR2
        '''
        self._mgmsg_mot_req_potparams(chan_ident=channel)
        data = self._mgmsg_mot_get_potparams()
        print(data)

    def _send(
            self,
            msg_id: int,
            param1: int = 0,
            param2: int = 0,
            dest: int = 0x50,
            src: int = 0x01,
            data: bytes = b''
    ) -> None:
        has_data = len(data) > 0
        hdr = APTHeader(
            message_id=msg_id,
            param1=param1,
            param2=param2,
            dest=dest,
            src=src,
            has_data=has_data
        )
        packet = hdr.pack(data) + data
        
        self._serial.reset_output_buffer()
        # self._serial.flush()
        self._serial.write(packet)
        self._serial.flush()
        time.sleep(0.03)

    def _recv(
            self,
            expected_msg_id: typing.Optional[int] = None,
            length: typing.Optional[int] = None,
            discard_async: bool = True,
            max_attempts: int = 10
    ) -> bytes:
        self._serial.flush()
        for _ in range(max_attempts):
            header_bytes = self._serial.read(6)
            if len(header_bytes) < 6:
                raise TimeoutError('Timeout: no header received')

            header = APTHeader.unpack(header_bytes)

            data_len = 0
            payload = b''
            if length and length > 6:
                data_len = struct.unpack_from('<H', header_bytes, 2)[0]
                header.has_data = True
                if data_len > 0:
                    payload = self._serial.read(data_len)
                    if len(payload) < data_len:
                        raise TimeoutError(f'Incomplete payload: expected {data_len}, got {len(payload)}')
            else:
                header.has_data = False

            msg_id = header.message_id

            if expected_msg_id is not None and msg_id != expected_msg_id:
                if discard_async and msg_id in (0x0464, 0x0466): #MGMSG_MOT_MOVE_COMPLETED, MGMSG_MOT_MOVE_STOPPED
                    # status = StatusUpdate.unpack(data=header_bytes+payload).active_flags()
                    # if any(s in ['INMOTIONCW','INMOTIONCCW','JOGGINGCW','JOGGINGCCW'] for s in status):
                    #     self._moving = True
                    # else:
                    #     self._moving = False
                    continue
                else:
                    raise ValueError(
                        f'Unexpected message ID: got 0x{msg_id:04X}, expected 0x{expected_msg_id:04X}'
                    )

            return header_bytes + payload

        raise TimeoutError(f'Expected message 0x{expected_msg_id:04X} not received after {max_attempts} attempts')


    def _mgmsg_mod_identify(
            self,
            chan_ident: int = 0x00,
            dest: int = 0x50,
            src: int = 0x01
    ) -> None:
        self._send(
            msg_id=0x0223,
            param1=chan_ident,
            dest=dest,
            src=src
        )

    def _mgmsg_mod_set_chanenablestate(
            self,
            chan_ident: int = 0x01,
            enable_state: int = 0x01
    ) -> None:
        self._send(
            msg_id=0x0210,
            param1=chan_ident,
            param2=enable_state
        )

    def _mgmsg_mod_req_chanenablestate(self, chan_ident: int = 0x01) -> None:
        self._send(msg_id=0x0211, param1=chan_ident)

    def _mgmsg_mod_get_chanenablestate(self) -> bytes:
        return self._recv(expected_msg_id=0x0212, length=6)

    def _mgmsg_hw_start_updatemsgs(self) -> None:
        self._send(msg_id=0x0011)

    def _mgmsg_hw_stop_updatemsgs(self) -> None:
        self._send(msg_id=0x0012)

    def _mgmsg_hw_req_info(self) -> None:
        self._send(msg_id=0x0005)

    def _mgmsg_hw_get_info(self) -> bytes:
        return self._recv(expected_msg_id=0x0006, length=90)
    
    def _mgmsg_mot_set_poscounter(self, chan_ident: int, position: int) -> None:
        data = struct.pack(
            '<hi',
            chan_ident,
            position
        )
        self._send(msg_id=0x0410, data=data)

    def _mgmsg_mot_req_poscounter(self, chan_ident: int) -> None:
        self._send(msg_id=0x0411, param1=chan_ident)

    def _mgmsg_mot_get_poscounter(self, chan_ident: int) -> bytes:
        return self._recv(expected_msg_id=0x0412, length=12)
    
    def _mgmsg_mot_set_encounter(self):
        pass

    def _mgmsg_mot_req_encounter(self):
        pass

    def _mgmsg_mot_get_encounter(self):
        pass

    def _mgmsg_mot_set_velparams(self, data: bytes) -> None:
        self._send(
            msg_id=0x0413,
            param1=0x0E,
            param2=0x00,
            dest=0x80,
            data=data
        )

    def _mgmsg_mot_req_velparams(self, chan_ident: int = 1) -> None:
        self._send(msg_id=0x0414, param1=chan_ident)

    def _mgmsg_mot_get_velparams(self) -> bytes:
        return self._recv(expected_msg_id=0x0415, length=14)

    def _mgmsg_mot_set_jogparams(self, data: bytes) -> None:
        self._send(
            msg_id=0x416,
            param1=0x16,
            dest=0x80,
            # src=0x01,
            data=data
        )

    def _mgmsg_mot_req_jogparams(self, chan_ident: int = 1) -> None:
        self._send(msg_id=0x0417, param1=chan_ident)

    def _mgmsg_mot_get_jogparams(self) -> bytes:
        return self._recv(expected_msg_id=0x0418, length=28)

    def _mgmsg_mot_set_genmoveparams(self):
        pass

    def _mgmsg_mot_req_genmoveparams(self):
        pass

    def _mgmsg_mot_get_genmoveparams(self):
        pass

    def _mgmsg_mot_set_moverelparams(self):
        pass

    def _mgmsg_mot_req_moverelparams(self):
        pass

    def _mgmsg_mot_get_moverelparams(self):
        pass

    def _mgmsg_mot_set_moveabsparams(self):
        pass

    def _mgmsg_mot_req_moveabsparams(self):
        pass

    def _mgmsg_mot_get_moveabsparams(self):
        pass

    def _mgmsg_mot_set_homeparams(self):
        pass

    def _mgmsg_mot_req_homeparams(self):
        pass

    def _mgmsg_mot_get_homeparams(self):
        pass

    def _mgmsg_mot_set_limswitchparams(self):
        pass

    def _mgmsg_mot_req_limswitchparams(self):
        pass

    def _mgmsg_mot_get_limswitchparams(self):
        pass

    def _mgmsg_mot_move_home(self, chan_ident: int) -> None:
        self._send(msg_id=0x0443, param1=chan_ident)

    def _mgmsg_mot_move_homed(self) -> bytes:
        return self._recv(expected_msg_id=0x0444, length=6)

    def _mgmsg_mot_move_relative(
            self,
            chan_ident: int = 1,
            distance: int = 0
    ) -> None:
        data = struct.pack(
            '<Hi',
            chan_ident,
            distance
        )
        self._send(
            msg_id=0x0448,
            param1=0x0006,
            param2=0x0D0,
            dest=0x80,
            data=data
        )

    def _mgmsg_mot_move_completed(self) -> bytes:
        return self._recv(expected_msg_id=0x0464, length=20)

    def _mgmsg_mot_move_absolute(self):
        pass

    def _mgmsg_mot_move_jog(
            self,
            chan_ident: int = 1,
            direction: int = 1
    ) -> None:
        self._send(msg_id=0x046A, param1=chan_ident, param2=direction)

    def _mgmsg_mot_move_velocity(
            self,
            chan_ident: int = 1,
            direction: int = 1
    ) -> None:
        self._send(msg_id=0x0457, param1=chan_ident, param2=direction)

    def _mgmsg_mot_move_stop(self, chan_ident: int = 1, stop_mode = 2) -> None:
        self._send(msg_id=0x0465, param1=chan_ident, param2=stop_mode)

    def _mgmsg_mot_move_stopped(self) -> bytes:
        return self._recv(expected_msg_id=0x0466, length=20)

    def _mgmsg_mot_set_avmodes(self):
        pass

    def _mgmsg_mot_req_avmodes(self):
        pass

    def _mgmsg_mot_get_avmodes(self):
        pass

    def _mgmsg_mot_set_potparams(self):
        pass

    def _mgmsg_mot_req_potparams(self, chan_ident: int = 1):
        self._send(msg_id=0x04B1, param1=chan_ident)

    def _mgmsg_mot_get_potparams(self) -> bytes:
        return self._recv(expected_msg_id=0x04B2, length=28)

    def _mgmsg_mot_set_buttonparams(self):
        pass

    def _mgmsg_mot_req_buttonparams(self):
        pass

    def _mgmsg_mot_get_buttonparams(self):
        pass

    def _mgmsg_mot_set_eepromparams(self):
        pass

    def _mgmsg_mot_req_statusbits(self):
        pass

    def _mgmsg_mot_get_statusbits(self):
        pass

class ThorlabsCageRotator(APTMotor):
    def __init__(
            self,
            conn: str,
            baudrate: int = 115200,
            timeout: float = 0.5,
            scale: typing.Literal['step', 'stage'] = 'step'
    ) -> None:
        super().__init__(
            conn=conn,
            baudrate=baudrate,
            timeout=timeout,
            scale=scale
        )
        match scale:
            case 'step':
                self._scale = (1,1,1)
            case 'stage':
                self._scale = (136533.33333333334, 7329109.333333334, 1502.0276667733335)

    def get_device_info(self) -> DeviceInfo:
        hw_info = self._hardware_get_info()
        if str(hw_info[1]).strip('\x00') in str(hw_info[4]):
            model = bytes(hw_info[4]).decode().split('\x00')[0]
        else:
            model = str(hw_info[1]).strip('\x00')

        return DeviceInfo(
            manufacturer='Thorlabs',
            model=model,
            serial_number=str(hw_info[0]),
            firmware_version=str(hw_info[3])
        )
    
    def setup_velocity(
            self,
            min_velocity: typing.Optional[float] = None,
            acceleration: typing.Optional[float] = None,
            max_velocity: typing.Optional[float] = None,
            scale: bool = True
    ) -> None:
        current_params = self.get_velocity_parameters()
        new_params = VelocityParameters(
            min_velocity=min_velocity if min_velocity else current_params.min_velocity,
            acceleration=acceleration if acceleration else current_params.acceleration,
            max_velocity=max_velocity if max_velocity else current_params.max_velocity,
            _chan_ident=1
        )
        self._set_vel_params(vel_params=new_params)

    def get_velocity_parameters(self) -> VelocityParameters:
        return VelocityParameters.from_internal(
            vel_params_internal=self._get_vel_params(),
            scale=self._scale
        )

    def setup_jog(
            self,
            mode: typing.Optional[typing.Literal['continuous', 'step']] = None,
            step_size: typing.Optional[float] = None,
            min_velocity: typing.Optional[float] = None,
            acceleration: typing.Optional[float] = None,
            max_velocity: typing.Optional[float] = None,
            stop_mode: typing.Optional[typing.Literal['immediate','profiled']] = None,
    ) -> None:
        current_params = self.get_jog_parameters()
        new_params = JogParameters(
            mode=mode if mode else current_params.mode,
            step_size=step_size if step_size else current_params.step_size,
            min_velocity=min_velocity if min_velocity else current_params.min_velocity,
            acceleration=acceleration if acceleration else current_params.acceleration,
            max_velocity=max_velocity if max_velocity else current_params.max_velocity,
            stop_mode=stop_mode if stop_mode else current_params.stop_mode,
            _chan_ident=1
        )
        self._set_jog_params(jog_params=new_params)

    def get_jog_parameters(self) -> JogParameters:
        return JogParameters.from_internal(
            jog_params_internal=self._get_jog_params(),
            scale=self._scale
        )

    def get_position(self) -> float:
        position = self._get_motor_position_counter()
        return self._d2p(value=position, kind='p')

    def move_by(self, distance: int, scale: bool = True) -> None:
        self._moving = True

        if scale:
            distance = self._p2d(value=distance, kind='p')
        self._mgmsg_mot_move_relative(distance=distance)

    def stop(
            self,
            immediate: bool = False,
            channel: int = 1
    ) -> None:
        self._mgmsg_mot_move_stop(
            chan_ident=channel,
            stop_mode=1 if immediate else 2
        )
    
    # def is_moving(self) -> bool:
    #     return self._moving

    def home(self) -> None:
        self._mgmsg_mot_move_home(chan_ident=1)
        # print(self._mgmsg_mot_move_homed())

    def jog(
            self,
            direction: typing.Literal['+', '-'],
            channel: int = 1,
            kind: typing.Literal['continuous', 'builtin'] = 'continuous'
    ) -> None:
        self._moving = True
        match direction:
            case '+':
                mot_dir = 0x01
            case '-':
                mot_dir = 0x02
            case _:
                raise RuntimeError(f'Unknown value for "direction": {direction}')

        match kind:
            case 'continuous':
                self._mgmsg_mot_move_velocity(
                    chan_ident=channel,
                    direction=mot_dir
                )
            case 'builtin':
                self._mgmsg_mot_move_jog(
                    chan_ident=channel,
                    direction=mot_dir
                )
            case _:
                raise RuntimeError(f'Unknown value for "kind": {kind}')

class ThorlabsMotor(thorlabs_motor.ThorlabsMotor):
    def __init__(
            self,
            serial_number: str | None = None,
            motor: ThorlabsCageRotator | None = None
    ) -> None:
        if serial_number:
            self._get_motor(serial_number=serial_number)
        elif motor:
            self._motor = motor
        else:
            raise RuntimeError('Must provide a serial number or ThorlabsCageRotator')

        device_info = self._motor.get_device_info()
        self.device_info = base_motor.DeviceInfo(
            device_name=device_info.manufacturer,
            model=device_info.model,
            serial_number=device_info.serial_number,
            firmware_version=device_info.firmware_version
        )

        self.position = self._motor.get_position()
        self.direction = base_motor.MotorDirection.IDLE
        self.step_size = 5.0
        self.acceleration = 20.0
        self.max_velocity = 25.0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._position_thread: threading.Thread | None = None
        self._movement_thread: threading.Thread | None = None
        self._position_polling = 0.1

    def _get_motor(self, serial_number: str) -> None:
        system = platform.system()
        match system:
            case 'Linux':
                symlink = next(
                    (s for s in pathlib.Path('/dev/serial/by-id').iterdir() if serial_number in s.name),
                    None
                )
                if symlink is not None:
                    self._motor = ThorlabsCageRotator(
                        conn=str(symlink.resolve()),
                        scale='stage'
                    )
                else:
                    raise RuntimeError(f'Motor {serial_number} not found')

            case _:
                raise NotImplementedError(f'Unsupported system: {system}')

def list_thorlabs_motors() -> list[tuple[str, str]]:
    system = platform.system()
    match system:
        case 'Linux':
            return list_thorlabs_motors_linux()

        case _:
            raise NotImplementedError(f'Unsupported system: {system}')

def list_thorlabs_motors_linux() -> list[tuple[str, str]]:
    device_path = pathlib.Path('/dev/serial/by-id')
    if not device_path.exists():
        return []

    motors: list[tuple[str, str]] = []
    for symlink in device_path.iterdir():
        if 'Thorlabs' in symlink.name and 'K10CR1' not in symlink.name:
            try:
                parts = symlink.name.removeprefix('usb-Thorlabs_').split('-if')[0].split('_')
                serial_number = parts[-1]
                device_name = ' '.join(parts[:-1])
                motors.append(
                    (serial_number, device_name)
                )
            except IndexError:
                continue
    return motors

def get_all_motors() -> list[ThorlabsMotor]:
    motors = []
    for m in list_thorlabs_motors():
        try:
            motor = ThorlabsMotor(serial_number=m[0])
        except:
            continue
        else:
            motors.append(motor)
    return motors

if __name__ == '__main__':
    serial_ids: list[pathlib.Path] = [s for s in pathlib.Path('/dev/serial/by-id').iterdir()]
    
    port = ''
    if serial_ids is not None:
        port = str(serial_ids[0].resolve())
    else:
        RuntimeError('No devices found')

    motor = ThorlabsMotor(serial_number='55536714')
    motor.move_by(angle=45.0, acceleration=20.0, max_velocity=25.0)
    # stage = ThorlabsCageRotator(
    #     conn=port,
    #     scale='stage'
    # )

    # print(stage.get_position())
    # stage.move_by(distance=45)
    # time.sleep(7)
    # print(stage.get_position())
    # stage.move_by(distance=-45)
    # time.sleep(7)
    # print(stage.get_position())
    # stage.close()