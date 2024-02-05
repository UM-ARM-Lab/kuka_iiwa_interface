"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class robotiq_3finger_actuator_command(object):
    __slots__ = ["timestamp", "position", "speed", "force"]

    def __init__(self):
        self.timestamp = 0.0
        self.position = 0.0
        self.speed = 0.0
        self.force = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(robotiq_3finger_actuator_command._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">dddd", self.timestamp, self.position, self.speed, self.force))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != robotiq_3finger_actuator_command._get_packed_fingerprint():
            raise ValueError("Decode error")
        return robotiq_3finger_actuator_command._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = robotiq_3finger_actuator_command()
        self.timestamp, self.position, self.speed, self.force = struct.unpack(">dddd", buf.read(32))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if robotiq_3finger_actuator_command in parents: return 0
        tmphash = (0x530204c46cbbbb36) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if robotiq_3finger_actuator_command._packed_fingerprint is None:
            robotiq_3finger_actuator_command._packed_fingerprint = struct.pack(">Q", robotiq_3finger_actuator_command._get_hash_recursive([]))
        return robotiq_3finger_actuator_command._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

