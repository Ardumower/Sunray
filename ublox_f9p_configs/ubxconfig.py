#!/usr/bin/env python3
"""
Apply u-blox CFG-VALSET configuration dumps captured from CFG-VALGET output.
"""

from __future__ import print_function

import argparse
import os
import select
import struct
import sys
import termios
import time


SYNC1 = 0xB5
SYNC2 = 0x62

UBX_CLASS_ACK = 0x05
UBX_ID_ACK_ACK = 0x01
UBX_ID_ACK_NAK = 0x00
UBX_CLASS_CFG = 0x06
UBX_ID_VALSET = 0x8A
UBX_ID_VALGET = 0x8B

DEFAULT_SERIAL_WRITE_TIMEOUT = 5.0
DEFAULT_ACK_RETRIES = 2
DEFAULT_BAUD_SWITCH_OLD_ACK_TIMEOUT = 0.35
DEFAULT_BAUD_SWITCH_NEW_ACK_TIMEOUT = 1.50
DEFAULT_BAUD_SWITCH_SETTLE_DELAY = 0.15
KEY_SIZE_TO_VALUE_BYTES = {
    0x01: 1,
    0x02: 1,
    0x03: 2,
    0x04: 4,
    0x05: 8,
}
MAX_VALSET_PAYLOAD = 512

VAL_LAYER_RAM = 1 << 0
VAL_LAYER_BBR = 1 << 1
VAL_LAYER_FLASH = 1 << 2
VAL_LAYER_ALL = VAL_LAYER_RAM | VAL_LAYER_BBR | VAL_LAYER_FLASH

CFG_UART1_BAUD_KEY = 0x40520001
CFG_UART2_BAUD_KEY = 0x40530001


def checksum(data):
    ck_a = 0
    ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def make_ubx_packet(msg_class, msg_id, payload):
    header = struct.pack("<BBH", msg_class, msg_id, len(payload))
    ck_a, ck_b = checksum(header + payload)
    return bytes(bytearray([SYNC1, SYNC2])) + header + payload + bytes(bytearray([ck_a, ck_b]))


def lookup_baud(baud_rate):
    attr = "B%d" % int(baud_rate)
    if not hasattr(termios, attr):
        raise ValueError("baud rate %s is not supported on this system" % baud_rate)
    return getattr(termios, attr)


class SerialPort(object):
    def __init__(self, path, baud_rate):
        self.path = path
        self.fd = None
        self.baud_rate = int(baud_rate)
        self.open(self.baud_rate)

    def open(self, baud_rate=None):
        if baud_rate is not None:
            self.baud_rate = int(baud_rate)
        if self.fd is not None:
            self.close()
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = termios.IGNBRK
        attrs[1] = 0
        attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[2] = attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
        attrs[3] = 0
        baud = lookup_baud(self.baud_rate)
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def set_baud(self, baud_rate):
        if self.fd is None:
            raise RuntimeError("serial port is not open: %s" % self.path)
        attrs = termios.tcgetattr(self.fd)
        self.baud_rate = int(baud_rate)
        baud = lookup_baud(self.baud_rate)
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        if self.fd is None:
            raise RuntimeError("serial port is not open: %s" % self.path)
        written = 0
        deadline = time.time() + DEFAULT_SERIAL_WRITE_TIMEOUT
        while written < len(data):
            remaining = deadline - time.time()
            if remaining <= 0:
                raise RuntimeError(
                    "serial write stalled after %d/%d bytes on %s at %d baud"
                    % (written, len(data), self.path, self.baud_rate)
                )
            _readable, writable, _errors = select.select([], [self.fd], [], min(0.25, remaining))
            if not writable:
                continue
            written += os.write(self.fd, data[written:])

    def read_ubx_frame(self, timeout_seconds):
        deadline = time.time() + timeout_seconds
        state = 0
        header = bytearray()
        payload = bytearray()
        payload_len = None
        checksum_bytes = bytearray()
        while time.time() < deadline:
            chunk = os.read(self.fd, 1)
            if not chunk:
                continue
            byte = chunk[0]
            if state == 0:
                if byte == SYNC1:
                    state = 1
                continue
            if state == 1:
                if byte == SYNC2:
                    state = 2
                    header = bytearray()
                else:
                    state = 0
                continue
            if state == 2:
                header.append(byte)
                if len(header) == 4:
                    payload_len = struct.unpack("<H", bytes(header[2:4]))[0]
                    payload = bytearray()
                    state = 3
                continue
            if state == 3:
                payload.append(byte)
                if len(payload) == payload_len:
                    checksum_bytes = bytearray()
                    state = 4
                continue
            checksum_bytes.append(byte)
            if len(checksum_bytes) == 2:
                ck_a, ck_b = checksum(bytes(header) + bytes(payload))
                if checksum_bytes[0] == ck_a and checksum_bytes[1] == ck_b:
                    return header[0], header[1], bytes(payload)
                state = 0
        return None

    def expect_ack_state(self, expected_class, expected_id, timeout_seconds):
        deadline = time.time() + timeout_seconds
        while time.time() < deadline:
            frame = self.read_ubx_frame(timeout_seconds)
            if frame is None:
                return "timeout"
            msg_class, msg_id, payload = frame
            if msg_class != UBX_CLASS_ACK or len(payload) < 2:
                continue
            if payload[0] != expected_class or payload[1] != expected_id:
                continue
            if msg_id == UBX_ID_ACK_ACK:
                return "ack"
            if msg_id == UBX_ID_ACK_NAK:
                return "nak"
        return "timeout"


def parse_hex_line(line, line_number):
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    if " - " not in line:
        raise ValueError("line %d: expected '<name> - <hex bytes>' format" % line_number)
    _, hex_bytes = line.split(" - ", 1)
    hex_bytes = hex_bytes.strip()
    if not hex_bytes:
        return None
    parts = hex_bytes.split()
    values = bytearray()
    for item in parts:
        if len(item) != 2:
            raise ValueError("line %d: invalid byte '%s'" % (line_number, item))
        values.append(int(item, 16))
    return bytes(values)


def parse_cfg_dump(config_path):
    entries = []
    with open(config_path, "r") as handle:
        for line_number, line in enumerate(handle, 1):
            raw = parse_hex_line(line, line_number)
            if raw is None or len(raw) < 4:
                continue
            msg_class = raw[0]
            msg_id = raw[1]
            payload_len = struct.unpack("<H", raw[2:4])[0]
            payload = raw[4:]
            if len(payload) != payload_len:
                raise ValueError(
                    "line %d: payload length mismatch (%d bytes declared, %d present)"
                    % (line_number, payload_len, len(payload))
                )
            if msg_class != UBX_CLASS_CFG or msg_id != UBX_ID_VALGET:
                continue
            if payload_len < 4:
                raise ValueError("line %d: CFG-VALGET payload too short" % line_number)
            offset = 4
            while offset < payload_len:
                if offset + 4 > payload_len:
                    raise ValueError("line %d: truncated key at offset %d" % (line_number, offset))
                key_bytes = payload[offset:offset + 4]
                key = struct.unpack("<I", key_bytes)[0]
                offset += 4
                key_size = (key >> 28) & 0x07
                value_len = KEY_SIZE_TO_VALUE_BYTES.get(key_size)
                if value_len is None:
                    raise ValueError(
                        "line %d: unsupported key size nibble 0x%X for key 0x%08X"
                        % (line_number, key_size, key)
                    )
                if offset + value_len > payload_len:
                    raise ValueError(
                        "line %d: truncated value for key 0x%08X at offset %d"
                        % (line_number, key, offset)
                    )
                value = payload[offset:offset + value_len]
                offset += value_len
                entries.append((key_bytes, value, key))
    if not entries:
        raise ValueError("no CFG-VALGET entries found in %s" % config_path)
    return entries


def build_valset_transactions(entries, layer_mask):
    transactions = []
    payload = bytearray([0, layer_mask, 0, 0])
    payload_entries = []
    for key_bytes, value, _key in entries:
        item = key_bytes + value
        if len(payload) + len(item) > MAX_VALSET_PAYLOAD:
            transactions.append({
                "packet": make_ubx_packet(UBX_CLASS_CFG, UBX_ID_VALSET, bytes(payload)),
                "entries": list(payload_entries),
            })
            payload = bytearray([0, layer_mask, 0, 0])
            payload_entries = []
        payload.extend(item)
        payload_entries.append((key_bytes, value, _key))
    if len(payload) > 4:
        transactions.append({
            "packet": make_ubx_packet(UBX_CLASS_CFG, UBX_ID_VALSET, bytes(payload)),
            "entries": list(payload_entries),
        })
    return transactions


def resolve_layer(name):
    mapping = {
        "ram": VAL_LAYER_RAM,
        "bbr": VAL_LAYER_BBR,
        "flash": VAL_LAYER_FLASH,
        "all": VAL_LAYER_ALL,
    }
    return mapping[name]


def extract_packet_baud_change(entries):
    baud_updates = {}
    for _key_bytes, value, key in entries:
        if key in (CFG_UART1_BAUD_KEY, CFG_UART2_BAUD_KEY) and len(value) == 4:
            baud_updates[key] = struct.unpack("<I", value)[0]
    return baud_updates


def wait_for_ack_with_baud_change(port, expected_class, expected_id, old_baud, target_baud, ack_timeout, settle_delay):
    old_ack = port.expect_ack_state(expected_class, expected_id, min(float(ack_timeout), DEFAULT_BAUD_SWITCH_OLD_ACK_TIMEOUT))
    if old_ack == "ack":
        if int(target_baud) != int(port.baud_rate):
            if settle_delay > 0:
                time.sleep(settle_delay)
            port.set_baud(int(target_baud))
        return "ack(old-baud)"
    if old_ack == "nak":
        return "nak(old-baud)"
    if settle_delay > 0:
        time.sleep(settle_delay)
    if int(target_baud) != int(port.baud_rate):
        port.set_baud(int(target_baud))
    new_ack = port.expect_ack_state(expected_class, expected_id, max(float(ack_timeout), DEFAULT_BAUD_SWITCH_NEW_ACK_TIMEOUT))
    if new_ack == "ack":
        return "ack(new-baud)"
    if new_ack == "nak":
        return "nak(new-baud)"
    return "timeout(new-baud)"


def resolve_cli_input_path(base_dir, maybe_relative):
    if not maybe_relative:
        return ""
    if os.path.isabs(maybe_relative):
        return maybe_relative
    cwd_candidate = os.path.abspath(maybe_relative)
    if os.path.exists(cwd_candidate):
        return cwd_candidate
    return os.path.abspath(os.path.join(base_dir, maybe_relative))


def apply_config(port_path, baud_rate, config_path, layer_name, ack_timeout, inter_packet_delay, ack_retries):
    entries = parse_cfg_dump(config_path)
    transactions = build_valset_transactions(entries, resolve_layer(layer_name))
    port = SerialPort(port_path, baud_rate)
    try:
        print("Applying %d config items from %s as %d packet(s)." % (len(entries), os.path.basename(config_path), len(transactions)))
        for index, transaction in enumerate(transactions, 1):
            packet = transaction["packet"]
            baud_updates = extract_packet_baud_change(transaction["entries"])
            print("  packet %d/%d" % (index, len(transactions)))
            target_baud = baud_updates.get(CFG_UART1_BAUD_KEY)
            final_state = "timeout"
            for attempt in range(1, int(ack_retries) + 1):
                if int(ack_retries) > 1:
                    print("    attempt %d/%d at local baud %d" % (attempt, int(ack_retries), int(port.baud_rate)))
                starting_baud = int(port.baud_rate)
                port.write(packet)
                if target_baud is not None and int(target_baud) != int(starting_baud):
                    print("    packet %d updates UART1 baud to %d; waiting across baud transition from %d" % (
                        index,
                        int(target_baud),
                        int(starting_baud),
                    ))
                    final_state = wait_for_ack_with_baud_change(
                        port,
                        UBX_CLASS_CFG,
                        UBX_ID_VALSET,
                        starting_baud,
                        int(target_baud),
                        ack_timeout,
                        DEFAULT_BAUD_SWITCH_SETTLE_DELAY,
                    )
                else:
                    final_state = port.expect_ack_state(UBX_CLASS_CFG, UBX_ID_VALSET, ack_timeout)
                if final_state.startswith("ack"):
                    break
                print("    no ACK for packet %d on attempt %d (state=%s, local_baud=%d)" % (
                    index,
                    attempt,
                    final_state,
                    int(port.baud_rate),
                ))
                if attempt < int(ack_retries):
                    time.sleep(max(float(inter_packet_delay), 0.20))
            if not final_state.startswith("ack"):
                raise RuntimeError("receiver did not ACK config packet %d (state=%s, local_baud=%d)" % (index, final_state, port.baud_rate))
            time.sleep(inter_packet_delay)
    finally:
        port.close()


def parse_args(argv):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description="Apply a u-blox config dump via CFG-VALSET.")
    parser.add_argument("config", help="Config dump path")
    parser.add_argument("-p", required=True, help="Port, e.g. /dev/ttyUSB0")
    parser.add_argument("-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--layer", choices=["ram", "bbr", "flash", "all"], default="all")
    parser.add_argument("--ack-timeout", type=float, default=2.0)
    parser.add_argument("--inter-packet-delay", type=float, default=0.1)
    parser.add_argument("--ack-retries", type=int, default=DEFAULT_ACK_RETRIES)
    args = parser.parse_args(argv)
    args.base_dir = base_dir
    args.config = resolve_cli_input_path(base_dir, args.config)
    return args


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if not os.path.exists(args.config):
        raise SystemExit("config file not found: %s" % args.config)
    apply_config(args.p, args.b, args.config, args.layer, args.ack_timeout, args.inter_packet_delay, args.ack_retries)
    print("Configuration applied successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
