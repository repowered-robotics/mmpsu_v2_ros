"""Specific implementations of the MMPSU v2 communication"""

from mmpsu_base import *
import serial
from typing import Dict, List
import crc

class MmpsuV2Uart(MmpsuV2Base):
    """Communicate with and control the MMPSU via a generic UART"""
    
    # Mins, maxes, etc
    MAX_VOUT = 24.0
    MIN_VOUT = 0.0
    MAX_PHASE_CURR_LIMIT = 32.0
    MIN_PHASE_CURR_LIMIT = 0.0

    # CAN bus 16-bit CRC
    CRC_CONFIG = crc.Configuration(
        width=16,
        polynomial=0x4599,
        init_value=0x0000,
        final_xor_value=0x0000,
        reverse_input=False,
        reverse_output=False,
    )
    
    def __init__(self, comport: str, logger):
        super().__init__(MMPSU_V2_REGS)
        self._ser = serial.Serial(comport)
        self._ser.baudrate = MMPSU_UART_BAUD
        self._ser.timeout = 0.25
        self.logger = logger
        self._rx_crc_err_count = 0
        self._mmpsu_crc_err_count

        if not self._ser.is_open:
            self._ser.open()
        
        self._crc_calc = crc.Calculator(self.CRC_CONFIG)

    def write_fields(self, field_values: Dict[str, Any]):
        """Write a dictionary of reg_name:value pairs to the MMPSU."""
        payload_len = 5*len(field_values)
        total_len = HDDR_SIZE + payload_len + CRC_SIZE
        packet = bytearray(total_len)
        offset = HDDR_SIZE
        for field, data in field_values.items():
            offset += self._regmap[field].pack_into(packet, data, offset)

        if offset != HDDR_SIZE + payload_len:
            # something is wrong
            return
        packet[0] = START_BYTE
        packet[CMD_IND] = MmpsuV2Commands.CMD_WRITE
        set_packet_size(packet, payload_len)
        self._add_crc(packet)

        if self._send_packet(packet):
            # TODO we might care if the MMPSU returns an error
            rpy = self._read_packet()
            self._verify_packet(packet, rpy)

    def read_fields(self, fields: List[str]) -> Dict[str, Any]:
        """Read a list of registers given by fields, and return a dictionary 
        of reg_name:value pairs."""
        payload_len = len(fields)
        total_len = HDDR_SIZE + payload_len + CRC_SIZE
        packet = bytearray(total_len)
        packet[0] = START_BYTE
        packet[CMD_IND] = MmpsuV2Commands.CMD_READ
        set_packet_size(packet, payload_len)

        ind = HDDR_SIZE
        for regstr in fields:
            packet[ind] = self._regmap[regstr].reg

        self._add_crc(packet)

        rpy_pack = self._read_packet()

        if not self._verify_packet(packet, rpy_pack):
            return {}
        
        return self._parse_read_packet(rpy_pack)

    def read_all_fields(self) -> Dict[str, Any]:
        """Issue the READ_ALL command, reading ALL fields, and return the 
        result as a dictionary of reg_name:value pairs."""
        packet = bytearray(HDDR_SIZE + CRC_SIZE)
        packet[0] = START_BYTE
        packet[CMD_IND] = MmpsuV2Commands.CMD_READ_ALL
        set_packet_size(packet, 0)
        self._add_crc(packet)

        if not self._send_packet(packet):
            return {}
        
        rpy_pack = self._read_packet()

        if not self._verify_packet(packet, rpy_pack):
            return {}

        return self._parse_read_packet(rpy_pack)

    def _verify_packet(self, tx_packet: bytearray, rx_packet: bytearray) -> bool:
        """Verify that a received packet passes CRC check and match packet 
        type. If not, print relevant error message."""
        if not self._check_crc(rx_packet):
            self._rx_crc_err_count += 1
            self.logger.warning("Rx CRC Error.")
            return False

        if rx_packet[CMD_IND] != tx_packet[CMD_IND]:
            if rx_packet[CMD_IND] == MmpsuV2Commands.CMD_ERROR:
                # some error occured, print error code
                self.logger.warning(f"Comms error: {MMPSU_ERR_CODES[rx_packet[HDDR_SIZE]]}")
                if rx_packet[HDDR_SIZE] == MmpsuV2ErrorCodes.ERR_CRC:
                    self._mmpsu_crc_err_count += 1
            else:
                # other unexpected packet type received
                self.logger.warning("Received unexpected packet type.")
            return False

        return True

    def _parse_read_packet(self, packet) -> Dict[str, Any]:
        """Parse a response to a READ command."""
        payload_len = get_payload_size(packet)
        payload = packet[HDDR_SIZE : HDDR_SIZE + payload_len]
        retval = {}

        for i in range(0, payload_len, 5): # 5 is the size of reg_num and value
            regnum = payload[i]
            retval[self._regmap_by_num[regnum].name] = self._regmap_by_num[regnum].unpack_from(payload, i + 1)

        return retval


    def _send_packet(self, packet: bytearray) -> bool:
        """Send a packet, return False if malformed and True if successful."""
        send_size = get_packet_size(packet)
        if send_size < len(packet):
            # malformed packet
            return False
        # only send as much as we need to send
        self._ser.write(packet[:send_size])
        return True
    
    def _read_packet(self) -> bytearray:
        """Read an incoming packet, return an empty bytearray on timeout."""
        hddr = self._ser.read(HDDR_SIZE)
        try:
            payload_len = get_payload_size(bytearray(hddr))
        except IndexError:
            # presumably a timeout
            return bytearray()

        payload_and_crc = self._ser.read(payload_len + CRC_SIZE)

        return bytearray(hddr + payload_and_crc)

    def _compute_crc(self, payload: bytearray):
        """Compute CRC over the entire payload array. payload argument should 
        be only the payload and nothing else"""
        return self._crc_calc.checksum(payload, optimized=True)

    def _add_crc(self, packet: bytearray | bytes):
        """Stuff the packet's CRC bits with the computed CRC."""
        chksum_ind = HDDR_SIZE + get_payload_size(packet)
        chksum = self._compute_crc(packet[HDDR_SIZE : chksum_ind])
        packet[chksum_ind] = (chksum >> 8) & 0xFF
        packet[chksum_ind + 1] = chksum & 0xFF
    
    def _check_crc(self, packet: bytearray | bytes) -> bool:
        """Verify that the received checksum matches a computed checksum over
        the received payload."""
        rx_chk = get_checksum(packet)
        chksum_ind = HDDR_SIZE + get_payload_size(packet)
        expect_chk = self._compute_crc(packet[HDDR_SIZE : chksum_ind])
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
