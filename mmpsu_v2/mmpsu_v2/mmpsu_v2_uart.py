"""Specific implementations of the MMPSU v2 communication"""

import mmpsu_v2.mmpsu_base as mmpsu
import serial
from typing import Dict, List, Any
from threading import Lock
from rclpy.impl.rcutils_logger import RcutilsLogger


class MmpsuV2Uart(mmpsu.MmpsuV2Base):
    """Communicate with and control the MMPSU via a generic UART"""

    # Mins, maxes, etc
    MAX_VOUT = 24.0
    MIN_VOUT = 0.0
    MAX_PHASE_CURR_LIMIT = 32.0
    MIN_PHASE_CURR_LIMIT = 0.0

    def __init__(self, comport: str, logger: RcutilsLogger):
        super().__init__(mmpsu.MMPSU_V2_REGS)
        self._ser = serial.Serial(comport)
        self._ser.baudrate = mmpsu.MMPSU_UART_BAUD
        self._ser.timeout = 0.25
        self.logger = logger
        self._rx_crc_err_count = 0
        self._mmpsu_crc_err_count = 0

        if not self._ser.is_open:
            self._ser.open()

        self._uart_lock = Lock()

    def test_comms(self) -> bool:
        """Test comms interface, return true on success, false on failure."""
        payload_len = 4
        total_len = mmpsu.HDDR_SIZE + payload_len + mmpsu.CRC_SIZE
        packet = bytearray(total_len)
        packet[0] = mmpsu.START_BYTE
        packet[mmpsu.CMD_IND] = mmpsu.MmpsuV2Commands.CMD_TEST_COMMS
        packet[mmpsu.HDDR_SIZE] = 0xDE
        packet[mmpsu.HDDR_SIZE + 1] = 0xAD
        packet[mmpsu.HDDR_SIZE + 2] = 0xBE
        packet[mmpsu.HDDR_SIZE + 3] = 0xEF
        mmpsu.set_packet_size(packet, payload_len)
        self._add_crc(packet)
        s = ""
        for b in packet:
            s += f"{b:02X} "

        with self._uart_lock:
            if not self._send_packet(packet):
                return False

            rpy = self._read_packet()
            s = ""
            for b in rpy:
                s += f"{b:02X} "

            if not self._verify_packet(packet, rpy):
                return False

            if (
                rpy[mmpsu.HDDR_SIZE : mmpsu.HDDR_SIZE + mmpsu.get_payload_size(rpy)]
                != packet[mmpsu.HDDR_SIZE : mmpsu.HDDR_SIZE + mmpsu.get_payload_size(packet)]
            ):
                return False

        return True

    def write_fields(self, field_values: Dict[str, Any]) -> bool:
        """Write a dictionary of reg_name:value pairs to the MMPSU. Returns
        True once a valid reply packet has been received from mmpsu."""
        payload_len = 5 * len(field_values)
        total_len = mmpsu.HDDR_SIZE + payload_len + mmpsu.CRC_SIZE
        packet = bytearray(total_len)
        offset = mmpsu.HDDR_SIZE
        for field, data in field_values.items():
            offset += self._regmap[field].pack_into(packet, data, offset)

        if offset != mmpsu.HDDR_SIZE + payload_len:
            # something is wrong
            return False

        packet[0] = mmpsu.START_BYTE
        packet[mmpsu.CMD_IND] = mmpsu.MmpsuV2Commands.CMD_WRITE
        mmpsu.set_packet_size(packet, payload_len)
        self._add_crc(packet)

        with self._uart_lock:
            if not self._send_packet(packet):
                return False
            # we care if the MMPSU returns an error
            rpy = self._read_packet()
            if not self._verify_packet(packet, rpy):
                return False

        return True

    def read_fields(self, fields: List[str]) -> Dict[str, Any]:
        """Read a list of registers given by fields, and return a dictionary
        of reg_name:value pairs."""
        payload_len = len(fields)
        total_len = mmpsu.HDDR_SIZE + payload_len + mmpsu.CRC_SIZE
        packet = bytearray(total_len)
        packet[0] = mmpsu.START_BYTE
        packet[mmpsu.CMD_IND] = mmpsu.MmpsuV2Commands.CMD_READ
        mmpsu.set_packet_size(packet, payload_len)

        ind = mmpsu.HDDR_SIZE
        for regstr in fields:
            packet[ind] = self._regmap[regstr].reg
            ind += 1

        self._add_crc(packet)

        with self._uart_lock:
            if not self._send_packet(packet):
                return {}

            rpy_pack = self._read_packet()
            if not self._verify_packet(packet, rpy_pack):
                return {}

        return self._parse_read_packet(rpy_pack)

    def read_all_fields(self) -> Dict[str, Any]:
        """Issue the READ_ALL command, reading ALL fields, and return the
        result as a dictionary of reg_name:value pairs."""
        packet = bytearray(mmpsu.HDDR_SIZE + mmpsu.CRC_SIZE)
        packet[0] = mmpsu.START_BYTE
        packet[mmpsu.CMD_IND] = mmpsu.MmpsuV2Commands.CMD_READ_ALL
        mmpsu.set_packet_size(packet, 0)
        self._add_crc(packet)

        with self._uart_lock:
            if not self._send_packet(packet):
                return {}

            rpy_pack = self._read_packet()

            if not self._verify_packet(packet, rpy_pack):
                return {}

        return self._parse_read_packet(rpy_pack)

    def _verify_packet(self, tx_packet: bytearray, rx_packet: bytearray) -> bool:
        """Verify that a received packet passes CRC check and match packet
        type. If not, print relevant error message."""
        if len(rx_packet) < mmpsu.HDDR_SIZE + mmpsu.CRC_SIZE:
            # minimum packet is a header and CRC zero-length payload
            return False

        if not self._check_crc(rx_packet):
            self._rx_crc_err_count += 1
            self.logger.warning("Rx CRC Error.")
            return False

        if rx_packet[mmpsu.CMD_IND] != tx_packet[mmpsu.CMD_IND]:
            if rx_packet[mmpsu.CMD_IND] == mmpsu.MmpsuV2Commands.CMD_ERROR:
                # some error occured, print error code
                self.logger.warning(
                    f"Comms error: {mmpsu.MMPSU_ERR_CODES[rx_packet[mmpsu.HDDR_SIZE]]}"
                )
                if rx_packet[mmpsu.HDDR_SIZE] == mmpsu.MmpsuV2ErrorCodes.ERR_CRC:
                    self._mmpsu_crc_err_count += 1
            else:
                # other unexpected packet type received
                self.logger.warning("Received unexpected packet type.")
            return False

        return True

    def _parse_read_packet(self, packet) -> Dict[str, Any]:
        """Parse a response to a READ command."""
        payload_len = mmpsu.get_payload_size(packet)
        payload = packet[mmpsu.HDDR_SIZE : mmpsu.HDDR_SIZE + payload_len]
        retval = {}

        for i in range(0, payload_len, 5):  # 5 is the size of reg_num and value
            regnum = payload[i]
            retval[self._regmap_by_num[regnum].name] = self._regmap_by_num[regnum].unpack_from(
                payload, i + 1
            )

        return retval

    def _send_packet(self, packet: bytearray) -> bool:
        """Send a packet, return False if packet is malformed and True on
        success."""
        send_size = mmpsu.get_packet_size(packet)
        if send_size < len(packet):
            # malformed packet
            return False
        # only send as much as we need to send
        self._ser.write(packet[:send_size])
        return True

    def _read_packet(self) -> bytearray:
        """Read an incoming packet, return an empty bytearray on timeout."""
        hddr = self._ser.read(mmpsu.HDDR_SIZE)
        try:
            payload_len = mmpsu.get_payload_size(bytearray(hddr))
        except IndexError:
            # presumably a timeout
            self.logger.warning(f"Read packet timeout. {hddr}")
            return bytearray()

        payload_and_crc = self._ser.read(payload_len + mmpsu.CRC_SIZE)

        return bytearray(hddr + payload_and_crc)

    def _compute_crc(self, payload: bytearray):
        """Compute CRC over the entire payload array. payload argument should
        be only the payload and nothing else"""
        return 0

    def _add_crc(self, packet: bytearray | bytes):
        """Stuff the packet's CRC bits with the computed CRC."""
        chksum_ind = mmpsu.HDDR_SIZE + mmpsu.get_payload_size(packet)
        chksum = self._compute_crc(packet[mmpsu.HDDR_SIZE : chksum_ind])
        packet[chksum_ind] = (chksum >> 8) & 0xFF
        packet[chksum_ind + 1] = chksum & 0xFF

    def _check_crc(self, packet: bytearray | bytes) -> bool:
        """Verify that the received checksum matches a computed checksum over
        the received payload."""
        rx_chk = mmpsu.get_checksum(packet)
        chksum_ind = mmpsu.HDDR_SIZE + mmpsu.get_payload_size(packet)
        expect_chk = self._compute_crc(packet[mmpsu.HDDR_SIZE : chksum_ind])
        return rx_chk == expect_chk

    @property
    def rx_crc_err_count(self) -> int:
        """Count of CRC errors on packets received from MMPSU. Clears on
        read."""
        retval = self._rx_crc_err_count
        self._rx_crc_err_count = 0
        return retval

    @property
    def mmpsu_crc_err_count(self) -> int:
        """Count of CRC errors reported by mmpsu. Clears on read."""
        retval = self._mmpsu_crc_err_count
        self._mmpsu_crc_err_count = 0
        return retval
