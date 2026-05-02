#!/usr/bin/env python3
"""
Minimal firmware updater for the ZED-F9P.

Current updater flow:
- probe the receiver and enter ROM boot when needed
- identify the flash device and load its flash.xml profile
- set update baud and send flash geometry
- erase sectors as needed before writing each data block
- write the final file-system marker
- verify the image and request reboot

Protocol model used here:
- `UBX_CLASS_UPD + UBX_ID_UPD_LOADER_IDENTIFY` identifies the flash chip
- `UBX_CLASS_UPD + UBX_ID_UPD_FLASH_GEOMETRY` sends flash layout/timing data
- `UBX_CLASS_UPD + UBX_ID_UPD_CHIP_ERASE` starts a whole-chip erase when requested
- `UBX_CLASS_UPD + UBX_ID_UPD_SECTOR_ERASE` erases one sector at a given offset
- `UBX_CLASS_UPD + UBX_ID_UPD_WRITE_DATA` writes one data block at a given offset
- the last `UBX_ID_UPD_WRITE_DATA` is a 4-byte file-system marker, not firmware
- `UBX_CLASS_UPD + UBX_ID_UPD_FLASH_VERIFY` verifies the flashed image
- `UBX_CLASS_UPD + UBX_ID_UPD_COMPLETE_REBOOT` completes the session

Mini example:
- assume sector size `512`
- data block 0: offset `0`, length `512`
- data block 1: offset `512`, length `512`
- final file-system marker: offset `1024`, length `4`
- one simple sequence is:
  1. `UBX_CLASS_UPD + UBX_ID_UPD_SECTOR_ERASE` offset `0`
  2. `UBX_CLASS_UPD + UBX_ID_UPD_WRITE_DATA` offset `0`, len `512`
  3. `UBX_CLASS_UPD + UBX_ID_UPD_SECTOR_ERASE` offset `512`
  4. `UBX_CLASS_UPD + UBX_ID_UPD_WRITE_DATA` offset `512`, len `512`
  5. `UBX_CLASS_UPD + UBX_ID_UPD_SECTOR_ERASE` offset `1024`
  6. `UBX_CLASS_UPD + UBX_ID_UPD_WRITE_DATA` offset `1024`, len `4`
  7. `UBX_CLASS_UPD + UBX_ID_UPD_FLASH_VERIFY`
  8. `UBX_CLASS_UPD + UBX_ID_UPD_COMPLETE_REBOOT`
"""

from __future__ import print_function

import argparse
import os
import re
import select
import struct
import sys
import termios
import time
import xml.etree.ElementTree as ET


SYNC1 = 0xB5
SYNC2 = 0x62

UBX_CLASS_ACK = 0x05
UBX_ID_ACK_ACK = 0x01
UBX_ID_ACK_NAK = 0x00
UBX_CLASS_CFG = 0x06
UBX_ID_CFG_PRT = 0x00
UBX_ID_CFG_RST = 0x04
UBX_CLASS_MON = 0x0A
UBX_ID_MON_VER = 0x04
UBX_ID_MON_HW3 = 0x37
UBX_CLASS_UPD = 0x09
UBX_ID_UPD_PROBE = 0x25
UBX_ID_UPD_UPLOAD = 0x07
UBX_ID_UPD_LOADER_VERSION = 0x06
UBX_ID_UPD_LOADER_IDENTIFY = 0x08
UBX_ID_UPD_SECTOR_ERASE = 0x0B
UBX_ID_UPD_WRITE_DATA = 0x0C
UBX_ID_UPD_FLASH_VERIFY = 0x0D
UBX_ID_UPD_CHIP_ERASE = 0x16
UBX_ID_UPD_COMPLETE_REBOOT = 0x0E
UBX_ID_UPD_FLASH_GEOMETRY = 0x19

DEFAULT_SERIAL_WRITE_TIMEOUT = 5.0
DEFAULT_UPD_DATA_BLOCK_SIZE = 512
DEFAULT_FLASH_SECTOR_SIZE = 4096
DEFAULT_INITIAL_ROM_TRAINING_TIMEOUT = 6.0
DEFAULT_INITIAL_ROM_POLL_WINDOW = 0.60
DEFAULT_INITIAL_ROM_TRAINING_SETTLE_DELAY = 0.15
DEFAULT_BAUD_SWITCH_MONVER_DELAY = 0.20
DEFAULT_UPLOAD_RECONNECT_MIN_DELAY = 0.70
DEFAULT_ROM_RECONNECT_SETTLE_DELAY = 1.05
DEFAULT_SECTOR_ERASE_DELAY = 0.0061
DEFAULT_WRITE_DATA_DELAY = 0.0042
DEFAULT_VERIFY_QUIET_DELAY = 14.401979000016581
DEFAULT_VERIFY_REPLY_DELAY = 1.161656000011135
DEFAULT_FINAL_STATUS_TIMEOUT = 0.75
DEFAULT_CHIP_ERASE_STATUS_TIMEOUT = 120.0
DEFAULT_PACKET_FEEDBACK_TIMEOUT = 0.75
DEFAULT_REQUEST_RETRIES = 3
DEFAULT_PIPELINE_WINDOW = 64
DEFAULT_BAUD_SWEEP = [115200, 9600, 38400, 57600, 19200, 4800, 230400]
DEFAULT_BAUD_SWEEP_QUICK_TIMEOUT = 0.90

DEFAULT_FLASH_MAN_ID = 0x00EF
DEFAULT_FLASH_DEV_ID = 0x4015
DEFAULT_FLASH_XML_REVISION = 200054
VERIFY_PREFIX = bytes(bytearray([0x01, 0x01, 0x00, 0x00, 0x00, 0x00]))
CAPTURED_FINAL_FS_MARKER = bytes(bytearray([0x0D, 0x05, 0x00, 0x00]))
STOP_GPS_CFG_RST_PAYLOAD = bytes(bytearray([0x04, 0x00, 0x00, 0x00, 0x08, 0x00]))

DEFAULT_FLASH_DEVICE_PROFILE = {
    "tool_signature": 0x08BC1B02,
    "sector_size": 4096,
    "sector_count": 512,
    "supply_code": 0x00000002,
    "cfg": 0x000000CC,
    "min_erase_suspend_us": 12000,
    "write_page_units": 16,
    "timing_a": 137,
    "timing_b": 489,
    "timing_c": 629,
    "timing_d": 1293,
    "timing_e": 769,
    "timing_f": 1769,
    "cmd_signature": 0xEB000000,
    "xml_revision": DEFAULT_FLASH_XML_REVISION,
    "feature_flags": 0x50007A75,
    "reserved": 0,
}

BUILTIN_FLASH_CMDSET_DATABASE = {
    "ADESTO": {"tool_signature": 0x08B81F02, "sector_size": 4096, "sector_count": 512, "supply_code": 0x0000000A, "cfg": 0x000010CC, "min_erase_suspend_us": 4800, "write_page_units": 16, "timing_a": 137, "timing_b": 485, "timing_c": 625, "timing_d": 1289, "timing_e": 765, "timing_f": 1765, "cmd_signature": 0xEB00ABB9, "feature_flags": 0x50007A75, "reserved": 0},
    "EON": {"tool_signature": 0x06241E02, "sector_size": 4096, "sector_count": 2048, "supply_code": 0x0000005A, "cfg": 0x000002CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 293, "timing_c": 433, "timing_d": 1097, "timing_e": 573, "timing_f": 0, "cmd_signature": 0xEB03ABB9, "feature_flags": 0x50FF0000, "reserved": 0},
    "FIDELIX": {"tool_signature": 0x085C1E02, "sector_size": 4096, "sector_count": 128, "supply_code": 0x0000005A, "cfg": 0x00000ACC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 393, "timing_c": 533, "timing_d": 1197, "timing_e": 673, "timing_f": 1673, "cmd_signature": 0xEB03ABB9, "feature_flags": 0x50007A75, "reserved": 0},
    "GIGADEVICE": {"tool_signature": 0x085C1A02, "sector_size": 4096, "sector_count": 128, "supply_code": 0x0000000A, "cfg": 0x000002CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 393, "timing_c": 533, "timing_d": 1197, "timing_e": 673, "timing_f": 1673, "cmd_signature": 0xEB000000, "feature_flags": 0x50007A75, "reserved": 0},
    "GIGADEVICE2": {"tool_signature": 0x08501B02, "sector_size": 4096, "sector_count": 128, "supply_code": 0x0000000A, "cfg": 0x000002CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 381, "timing_c": 521, "timing_d": 1185, "timing_e": 661, "timing_f": 1661, "cmd_signature": 0xEB000000, "feature_flags": 0x50007A75, "reserved": 0},
    "ISSI": {"tool_signature": 0x08501A02, "sector_size": 4096, "sector_count": 512, "supply_code": 0x0000000A, "cfg": 0x00000ACC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 381, "timing_c": 521, "timing_d": 1185, "timing_e": 661, "timing_f": 1661, "cmd_signature": 0xEB000000, "feature_flags": 0x50000000, "reserved": 0},
    "MACRONIX": {"tool_signature": 0x08B41E02, "sector_size": 4096, "sector_count": 256, "supply_code": 0x000000A5, "cfg": 0x000002CC, "min_erase_suspend_us": 48000, "write_page_units": 16, "timing_a": 137, "timing_b": 481, "timing_c": 621, "timing_d": 1285, "timing_e": 761, "timing_f": 1761, "cmd_signature": 0xEB14ABB9, "feature_flags": 0x500030B0, "reserved": 0},
    "MACRONIX2": {"tool_signature": 0x08B41E02, "sector_size": 4096, "sector_count": 256, "supply_code": 0x000000A5, "cfg": 0x000002CC, "min_erase_suspend_us": 48000, "write_page_units": 16, "timing_a": 137, "timing_b": 481, "timing_c": 621, "timing_d": 1285, "timing_e": 761, "timing_f": 1761, "cmd_signature": 0xEB14ABB9, "feature_flags": 0x50007A75, "reserved": 0},
    "MICRON": {"tool_signature": 0x05481A02, "sector_size": 4096, "sector_count": 1024, "supply_code": 0x00000000, "cfg": 0x000002CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 275, "timing_c": 1213, "timing_d": 507, "timing_e": 779, "timing_f": 0, "cmd_signature": 0xEB000000, "feature_flags": 0x50FF0000, "reserved": 0},
    "SPANSION": {"tool_signature": 0x08BC1B02, "sector_size": 4096, "sector_count": 128, "supply_code": 0x00000002, "cfg": 0x000000CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 489, "timing_c": 629, "timing_d": 1293, "timing_e": 769, "timing_f": 1769, "cmd_signature": 0xEB06ABB9, "feature_flags": 0x50007A75, "reserved": 0},
    "SST": {"tool_signature": 0x04681B02, "sector_size": 4096, "sector_count": 1024, "supply_code": 0x0B000000, "cfg": 0x000000C5, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 271, "timing_b": 137, "timing_c": 0, "timing_d": 941, "timing_e": 463, "timing_f": 741, "cmd_signature": 0x00000000, "feature_flags": 0x50000000, "reserved": 0},
    "SST2": {"tool_signature": 0x08741E02, "sector_size": 4096, "sector_count": 2048, "supply_code": 0x0000005A, "cfg": 0x000000CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 417, "timing_c": 557, "timing_d": 1221, "timing_e": 697, "timing_f": 1697, "cmd_signature": 0xEB00ABB9, "feature_flags": 0x500030B0, "reserved": 0},
    "WINBOND": {"tool_signature": 0x08BC1B02, "sector_size": 4096, "sector_count": 128, "supply_code": 0x00000002, "cfg": 0x000000CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 489, "timing_c": 629, "timing_d": 1293, "timing_e": 769, "timing_f": 1769, "cmd_signature": 0xEB000000, "feature_flags": 0x50007A75, "reserved": 0},
    "XTX": {"tool_signature": 0x08BC1A02, "sector_size": 4096, "sector_count": 512, "supply_code": 0x00000002, "cfg": 0x000002CC, "min_erase_suspend_us": 12000, "write_page_units": 16, "timing_a": 137, "timing_b": 489, "timing_c": 629, "timing_d": 1293, "timing_e": 769, "timing_f": 1769, "cmd_signature": 0xEB000000, "feature_flags": 0x50007A75, "reserved": 0},
}

BUILTIN_FLASH_DEVICE_DATABASE = {
    0xBF2601: {"cmdset_name": "SST", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512},
    0xBF2602: {"cmdset_name": "SST", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0xBF2643: {"cmdset_name": "SST2", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0xBF2641: {"cmdset_name": "SST2", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512},
    0x014016: {"cmdset_name": "SPANSION", "supply_label": "3V", "supply_code": 0x00000002, "sector_count": 1024},
    0x014015: {"cmdset_name": "SPANSION", "supply_label": "3V", "supply_code": 0x00000002, "sector_count": 512},
    0xEF4013: {"cmdset_name": "WINBOND", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 128},
    0xEF4014: {"cmdset_name": "WINBOND", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 256},
    0xEF4015: {"cmdset_name": "WINBOND", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512},
    0xEF4016: {"cmdset_name": "WINBOND", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0xEF4017: {"cmdset_name": "WINBOND", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0xEF5014: {"cmdset_name": "WINBOND", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 256, "write_suspend": 0},
    0xEF6013: {"cmdset_name": "WINBOND", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 128},
    0xEF6014: {"cmdset_name": "WINBOND", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 256},
    0xEF6015: {"cmdset_name": "WINBOND", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 512},
    0xEF6016: {"cmdset_name": "WINBOND", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 1024},
    0xEF7015: {"cmdset_name": "WINBOND", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512},
    0xC22014: {"cmdset_name": "MACRONIX", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 256},
    0xC22016: {"cmdset_name": "MACRONIX", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024, "min_erase_suspend_us": 12000, "write_suspend": 1},
    0xC22017: {"cmdset_name": "MACRONIX", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0xC22515: {"cmdset_name": "MACRONIX", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512},
    0xC22415: {"cmdset_name": "MACRONIX", "sector_size": 4096, "sector_count": 512},
    0xC22534: {"cmdset_name": "MACRONIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 256, "min_erase_suspend_us": 12000, "write_suspend": 1},
    0xC22535: {"cmdset_name": "MACRONIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 512, "min_erase_suspend_us": 12000, "write_suspend": 1},
    0xC22536: {"cmdset_name": "MACRONIX2", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024, "min_erase_suspend_us": 9600, "write_suspend": 1},
    0xC22314: {"cmdset_name": "MACRONIX2", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 256, "min_erase_suspend_us": 9600, "write_suspend": 1},
    0xC22315: {"cmdset_name": "MACRONIX2", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512, "min_erase_suspend_us": 9600, "write_suspend": 1},
    0xC22814: {"cmdset_name": "MACRONIX2", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 256, "min_erase_suspend_us": 9600, "write_suspend": 1},
    0xC22815: {"cmdset_name": "MACRONIX2", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_size": 4096, "sector_count": 512, "min_erase_suspend_us": 9600, "write_suspend": 1},
    0xC84013: {"cmdset_name": "GIGADEVICE", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 128},
    0xC84014: {"cmdset_name": "GIGADEVICE", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 256},
    0xC84015: {"cmdset_name": "GIGADEVICE", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 512},
    0xC84016: {"cmdset_name": "GIGADEVICE2", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0x20BA16: {"cmdset_name": "MICRON", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0x20BA18: {"cmdset_name": "MICRON", "supply_label": "3V", "supply_code": 0x00000002, "sector_size": 4096, "sector_count": 1024},
    0x1C3013: {"cmdset_name": "EON", "sector_count": 128, "cfg": 0x000010CC},
    0x1C3014: {"cmdset_name": "EON", "sector_count": 256},
    0x1C7015: {"cmdset_name": "EON", "sector_count": 512},
    0x1C3814: {"cmdset_name": "EON", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 256, "cfg": 0x000004CC},
    0x1C3815: {"cmdset_name": "EON", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 512, "cfg": 0x000004CC},
    0x1C3816: {"cmdset_name": "EON", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024, "cfg": 0x000004CC},
    0x1C3817: {"cmdset_name": "EON", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024, "cfg": 0x000004CC},
    0xF83214: {"cmdset_name": "FIDELIX", "sector_count": 256},
    0xF83215: {"cmdset_name": "FIDELIX", "sector_count": 512},
    0xF84214: {"cmdset_name": "FIDELIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 256},
    0xF84215: {"cmdset_name": "FIDELIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 512},
    0xF84216: {"cmdset_name": "FIDELIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024},
    0xF84217: {"cmdset_name": "FIDELIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024},
    0xF84317: {"cmdset_name": "FIDELIX", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024},
    0x9D6015: {"cmdset_name": "ISSI", "sector_count": 512},
    0x9D6016: {"cmdset_name": "ISSI", "sector_count": 1024},
    0x9D7015: {"cmdset_name": "ISSI", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 512},
    0x9D7016: {"cmdset_name": "ISSI", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024},
    0x9D7017: {"cmdset_name": "ISSI", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024},
    0x1F4317: {"cmdset_name": "ADESTO", "supply_label": "1.8V", "supply_code": 0x00000001, "sector_count": 1024},
    0x0B4015: {"cmdset_name": "XTX", "sector_count": 512},
}

UBFL_MAGIC = b"UBFL"
UBFL_TRAILER_WORDS = 9
UBFL_TRAILER_SIZE = UBFL_TRAILER_WORDS * 4

def verbose_print(enabled, message):
    if enabled:
        print(message)


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


def parse_baud_triplet(spec):
    parts = spec.split(":")
    if len(parts) == 1:
        value = int(parts[0])
        return value, value, value
    if len(parts) == 2:
        return int(parts[0]), int(parts[1]), int(parts[1])
    if len(parts) == 3:
        return int(parts[0]), int(parts[1]), int(parts[2])
    raise ValueError("invalid baud specification: %s" % spec)


def combine_jedec_id(man_id, dev_id):
    man_id = int(man_id) & 0xFF
    dev_id = int(dev_id) & 0xFFFF
    return struct.unpack("<I", bytes(bytearray([man_id, (dev_id >> 8) & 0xFF, dev_id & 0xFF, 0x00])))[0]


def apply_builtin_flash_profile(profile, man_id, dev_id):
    lookup_key = ((int(man_id) & 0xFF) << 16) | (int(dev_id) & 0xFFFF)
    device_overrides = BUILTIN_FLASH_DEVICE_DATABASE.get(lookup_key)
    if device_overrides and device_overrides.get("cmdset_name"):
        cmdset_profile = BUILTIN_FLASH_CMDSET_DATABASE.get(device_overrides["cmdset_name"])
        if cmdset_profile:
            profile.update(cmdset_profile)
    if device_overrides:
        profile.update(device_overrides)
    return profile


def load_flash_device_profile(flash_xml_path, man_id, dev_id):
    profile = dict(DEFAULT_FLASH_DEVICE_PROFILE)
    profile["man_id"] = int(man_id)
    profile["dev_id"] = int(dev_id)
    profile["jedec"] = combine_jedec_id(man_id, dev_id)
    apply_builtin_flash_profile(profile, man_id, dev_id)
    if not flash_xml_path or not os.path.exists(flash_xml_path):
        return profile
    tree = ET.parse(flash_xml_path)
    root = tree.getroot()
    revision = root.attrib.get("revision", "")
    match = re.search(r"(\d+)", revision)
    if match:
        profile["xml_revision"] = int(match.group(1))
    wanted = "x%02X%04X" % (int(man_id) & 0xFF, int(dev_id) & 0xFFFF)
    for category in root.findall("category"):
        category_sector_size = category.findtext("sectorSize")
        category_cfg = category.findtext("cfg")
        category_suspend = category.findtext("minEraseSuspend")
        for device in category.findall("device"):
            if device.attrib.get("jedec", "").lower() != wanted.lower():
                continue
            sector_size = device.findtext("sectorSize") or category_sector_size
            sector_count = device.findtext("sectorCount")
            cfg = device.findtext("cfg") or category_cfg
            suspend = device.findtext("minEraseSuspend") or category_suspend
            if sector_size:
                profile["sector_size"] = int(sector_size.strip().rstrip("/"), 0)
            if sector_count:
                profile["sector_count"] = int(sector_count.strip().rstrip("/"), 0)
            if cfg:
                profile["cfg"] = int(cfg.strip(), 0)
            if suspend:
                profile["min_erase_suspend_us"] = int(suspend.strip(), 0)
            return profile
    return profile


def build_flashprot_payload_from_profile(profile):
    words = [
        profile["tool_signature"],
        profile.get("jedec", combine_jedec_id(DEFAULT_FLASH_MAN_ID, DEFAULT_FLASH_DEV_ID)),
        profile["sector_size"],
        profile["sector_count"],
        profile["supply_code"],
        profile["cfg"],
        profile["min_erase_suspend_us"],
        profile["write_page_units"],
        profile["timing_a"],
        profile["timing_b"],
        profile["timing_c"],
        profile["timing_d"],
        profile["timing_e"],
        profile["timing_f"],
        profile["cmd_signature"],
        profile["xml_revision"],
        profile["feature_flags"],
        profile["reserved"],
    ]
    return b"".join(struct.pack("<I", word) for word in words)


CAPTURED_UPD_FLASHPROT_PAYLOAD = build_flashprot_payload_from_profile(
    load_flash_device_profile("", DEFAULT_FLASH_MAN_ID, DEFAULT_FLASH_DEV_ID)
)


def build_upd_verify_payload(image_path, flashprot_payload, include_fis_in_flash):
    image_bytes = load_firmware_payload(image_path)["raw_bytes"]
    verify_words = (flashprot_payload + image_bytes) if include_fis_in_flash else image_bytes
    if len(verify_words) % 4:
        raise ValueError("verify input length is not DWORD-aligned: %d" % len(verify_words))
    sum1 = 0
    sum2 = 0
    for offset in range(0, len(verify_words), 4):
        word = struct.unpack("<I", verify_words[offset:offset + 4])[0]
        sum1 = (sum1 + word) & 0xFFFFFFFF
        sum2 = (sum2 + sum1) & 0xFFFFFFFF
    total_size = len(verify_words)
    return VERIFY_PREFIX + struct.pack("<III", total_size, sum1, sum2)


def build_upd_data_payload(block_units, chunk):
    return struct.pack("<II", int(block_units), len(chunk)) + chunk


def iter_upd_data_chunks(image_bytes, flashprot_payload, block_size, merge_fis, fis_separate=False):
    if fis_separate:
        byte_offset = 0
        while byte_offset < len(flashprot_payload):
            chunk = flashprot_payload[byte_offset:byte_offset + block_size]
            yield byte_offset, chunk
            byte_offset += len(chunk)
        image_offset = len(flashprot_payload)
        byte_offset = 0
        while byte_offset < len(image_bytes):
            chunk = image_bytes[byte_offset:byte_offset + block_size]
            yield image_offset + byte_offset, chunk
            byte_offset += len(chunk)
        return
    combined = (flashprot_payload + image_bytes) if merge_fis else image_bytes
    byte_offset = 0
    while byte_offset < len(combined):
        chunk = combined[byte_offset:byte_offset + block_size]
        yield byte_offset, chunk
        byte_offset += len(chunk)


def build_final_fs_marker_payload(image_path, flashprot_payload, sector_size, include_fis_in_flash):
    image_bytes = load_firmware_payload(image_path)["raw_bytes"]
    total_bytes = (len(flashprot_payload) + len(image_bytes)) if include_fis_in_flash else len(image_bytes)
    final_offset = (total_bytes + (int(sector_size) - 1)) & ~(int(sector_size) - 1)
    return struct.pack("<II", int(final_offset), len(CAPTURED_FINAL_FS_MARKER)) + CAPTURED_FINAL_FS_MARKER


def build_upd_data_packets(image_path, flashprot_payload, block_size, sector_size, merge_fis, fis_separate=False):
    image_bytes = load_firmware_payload(image_path)["raw_bytes"]
    packets = []
    include_fis_in_flash = merge_fis or fis_separate
    for block_units, chunk in iter_upd_data_chunks(image_bytes, flashprot_payload, block_size, merge_fis, fis_separate):
        packets.append(make_ubx_packet(UBX_CLASS_UPD, UBX_ID_UPD_WRITE_DATA, build_upd_data_payload(block_units, chunk)))
    packets.append(
        make_ubx_packet(
            UBX_CLASS_UPD,
            UBX_ID_UPD_WRITE_DATA,
            build_final_fs_marker_payload(image_path, flashprot_payload, sector_size, include_fis_in_flash),
        )
    )
    return packets


def parse_ubfl_trailer(image_bytes):
    if len(image_bytes) < UBFL_TRAILER_SIZE:
        return None
    trailer = image_bytes[-UBFL_TRAILER_SIZE:]
    if trailer[-4:] != UBFL_MAGIC:
        return None
    words = struct.unpack("<9I", trailer)
    image_sizes = []
    for size in words[3:3 + words[5]]:
        if size:
            image_sizes.append(size)
    if not image_sizes:
        return None
    payload_size = sum(image_sizes)
    if payload_size > len(image_bytes):
        raise ValueError("UBFL trailer payload size exceeds file length")
    return {
        "raw": trailer,
        "image_sizes": image_sizes,
        "payload_size": payload_size,
    }


def load_firmware_payload(image_path):
    with open(image_path, "rb") as handle:
        image_bytes = handle.read()
    return {
        "raw_bytes": image_bytes,
        "ubfl": parse_ubfl_trailer(image_bytes),
    }


def build_sector_erase_payload(address):
    return struct.pack("<I", int(address))


def build_linear_sector_erase_payloads(count, sector_size):
    return [build_sector_erase_payload(index * sector_size) for index in range(count)]


def sector_index_for_offset(offset, sector_size):
    return int(offset) // int(sector_size)


class SerialPort(object):
    def __init__(self, path, baud_rate):
        self.path = path
        self.fd = None
        self.baud_rate = int(baud_rate)
        self.pending_frames = []
        self.open(self.baud_rate)

    def is_open(self):
        return self.fd is not None

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
        self.pending_frames = []

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def reopen(self, baud_rate=None):
        self.open(self.baud_rate if baud_rate is None else baud_rate)

    def set_baud(self, baud_rate):
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

    def write_raw(self, data):
        self.write(data)

    def unread_frame(self, frame):
        self.pending_frames.insert(0, frame)

    def read_ubx_frame(self, timeout_seconds):
        if self.pending_frames:
            return self.pending_frames.pop(0)
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

    def poll(self, msg_class, msg_id, timeout_seconds):
        self.write(make_ubx_packet(msg_class, msg_id, b""))
        while True:
            frame = self.read_ubx_frame(timeout_seconds)
            if frame is None:
                return None
            got_class, got_id, payload = frame
            if got_class == msg_class and got_id == msg_id:
                return payload

    def send_ubx(self, msg_class, msg_id, payload):
        self.write(make_ubx_packet(msg_class, msg_id, payload))

    def drain_input(self, quiet_seconds, overall_timeout):
        deadline = time.time() + overall_timeout
        last_rx = time.time()
        collected = bytearray()
        while time.time() < deadline:
            chunk = os.read(self.fd, 4096)
            if chunk:
                collected.extend(chunk)
                last_rx = time.time()
                continue
            if time.time() - last_rx >= quiet_seconds:
                break
        return bytes(collected)

    def expect_ack(self, expected_class, expected_id, timeout_seconds):
        deadline = time.time() + timeout_seconds
        while time.time() < deadline:
            frame = self.read_ubx_frame(timeout_seconds)
            if frame is None:
                return False
            msg_class, msg_id, payload = frame
            if msg_class != UBX_CLASS_ACK or len(payload) < 2:
                continue
            if payload[0] != expected_class or payload[1] != expected_id:
                continue
            return msg_id == UBX_ID_ACK_ACK
        return False


def summarize_drain_bytes(data):
    if not data:
        return "no data"
    preview = data[:24].hex()
    if len(data) > 24:
        preview += "..."
    return "%d byte(s) [%s]" % (len(data), preview)


def drain_phase(port, verbose, label, quiet_seconds, overall_timeout):
    data = port.drain_input(quiet_seconds, overall_timeout)
    verbose_print(verbose, "  %s: %s" % (label, summarize_drain_bytes(data)))
    return data


def get_path_identity(path):
    try:
        st = os.stat(path)
    except OSError:
        return None
    return (st.st_dev, st.st_ino, st.st_rdev)


def reconnect_port(port, baud_rate, timeout_seconds, verbose, label, require_disconnect=False, min_reopen_delay=0.0):
    verbose_print(verbose, "  waiting for reconnect: %s" % label)
    original_realpath = os.path.realpath(port.path) if os.path.exists(port.path) else ""
    original_identity = get_path_identity(original_realpath or port.path)
    port.close()
    disconnect_start = time.time()
    deadline = time.time() + timeout_seconds
    last_error = ""
    saw_disconnect = False
    while time.time() < deadline:
        exists = os.path.exists(port.path)
        current_realpath = os.path.realpath(port.path) if exists else ""
        elapsed = time.time() - disconnect_start
        if not exists:
            saw_disconnect = True
            time.sleep(0.10)
            continue
        if elapsed < min_reopen_delay:
            time.sleep(0.10)
            continue
        if require_disconnect and not saw_disconnect:
            current_identity = get_path_identity(current_realpath or port.path)
            if current_realpath == original_realpath and current_identity == original_identity:
                time.sleep(0.10)
                continue
        try:
            port.reopen(baud_rate)
            drain_phase(port, verbose, "%s initial drain" % label, 0.05, 0.20)
            verbose_print(verbose, "  reconnected: %s at %d baud%s" % (label, baud_rate, " after disconnect" if require_disconnect else ""))
            return True
        except OSError as exc:
            last_error = str(exc)
            time.sleep(0.10)
    if last_error:
        verbose_print(verbose, "  reconnect error: %s" % last_error)
    return False


def decode_nul_strings(payload, chunk_sizes):
    items = []
    offset = 0
    for chunk_size in chunk_sizes:
        chunk = payload[offset:offset + chunk_size]
        offset += chunk_size
        text = chunk.split(b"\x00", 1)[0].decode("ascii", "replace").strip()
        items.append(text)
    return items


def parse_mon_ver(payload):
    if len(payload) < 40:
        raise ValueError("MON-VER payload too short")
    sw_version, hw_version = decode_nul_strings(payload[:40], [30, 10])
    extensions = []
    offset = 40
    while offset + 30 <= len(payload):
        text = payload[offset:offset + 30].split(b"\x00", 1)[0].decode("ascii", "replace").strip()
        if text:
            extensions.append(text)
        offset += 30
    return {
        "sw_version": sw_version,
        "hw_version": hw_version,
        "extensions": extensions,
    }


def extract_specific_ubx_payload_from_bytes(raw_bytes, expected_class, expected_id):
    if not raw_bytes:
        return None
    index = 0
    raw_len = len(raw_bytes)
    while index + 8 <= raw_len:
        if raw_bytes[index] != SYNC1 or raw_bytes[index + 1] != SYNC2:
            index += 1
            continue
        msg_class = raw_bytes[index + 2]
        msg_id = raw_bytes[index + 3]
        payload_len = struct.unpack("<H", raw_bytes[index + 4:index + 6])[0]
        frame_end = index + 6 + payload_len + 2
        if frame_end > raw_len:
            index += 1
            continue
        payload = raw_bytes[index + 6:index + 6 + payload_len]
        ck_a, ck_b = checksum(raw_bytes[index + 2:index + 6 + payload_len])
        if raw_bytes[index + 6 + payload_len] == ck_a and raw_bytes[index + 6 + payload_len + 1] == ck_b:
            if msg_class == expected_class and msg_id == expected_id:
                return payload
            index = frame_end
            continue
        index += 1
    return None


def extract_mon_ver_payload_from_text_bytes(raw_bytes):
    if not raw_bytes:
        return None
    text = raw_bytes.decode("ascii", "ignore")

    sw_match = re.search(r"ROM BOOT\s+\d+\.\d+\s+\([^)]+\)", text)
    if not sw_match:
        return None
    sw_version = sw_match.group(0).strip()

    hw_match = re.search(r"0{2,}\d{4,}0{2,}", text)
    hw_version = ""
    if hw_match:
        hw_candidate = hw_match.group(0).strip("\x00")
        hw_digits = re.sub(r"[^0-9A-Fa-f]", "", hw_candidate)
        if len(hw_digits) >= 8:
            hw_version = hw_digits[-8:]
    if not hw_version:
        alt_hw_match = re.search(r"\b\d{8}\b", text)
        if alt_hw_match:
            hw_version = alt_hw_match.group(0)

    extensions = []
    protver_match = re.search(r"PROTVER=\d+\.\d+", text)
    if protver_match:
        extensions.append(protver_match.group(0))
    mod_match = re.search(r"MOD=[A-Za-z0-9\-]+", text)
    if mod_match:
        extensions.append(mod_match.group(0))

    if not hw_version:
        hw_version = "00190000"

    payload = bytearray(40 + 30 * len(extensions))
    payload[:30] = sw_version.encode("ascii", "replace")[:30].ljust(30, b"\x00")
    payload[30:40] = hw_version.encode("ascii", "replace")[:10].ljust(10, b"\x00")
    offset = 40
    for item in extensions:
        payload[offset:offset + 30] = item.encode("ascii", "replace")[:30].ljust(30, b"\x00")
        offset += 30
    return bytes(payload)


def extract_mon_ver_payload_from_probe_bytes(raw_bytes):
    payload = extract_specific_ubx_payload_from_bytes(raw_bytes, UBX_CLASS_MON, UBX_ID_MON_VER)
    if payload is not None:
        return payload
    return extract_mon_ver_payload_from_text_bytes(raw_bytes)


def parse_mon_hw3(payload):
    if len(payload) < 22:
        raise ValueError("MON-HW3 payload too short")
    version = payload[0]
    flags = payload[2]
    hw_version = payload[3:13].split(b"\x00", 1)[0].decode("ascii", "replace").strip()
    return {
        "version": version,
        "safe_boot": bool(flags & 0x02),
        "hw_version": hw_version,
    }


def print_probe(mon_ver, mon_hw3):
    print("Receiver probe:")
    print("  SW:        %s" % mon_ver["sw_version"])
    print("  HW:        %s" % mon_ver["hw_version"])
    print("  SafeBoot:  %s" % ("yes" if mon_hw3["safe_boot"] else "no"))
    for item in mon_ver["extensions"]:
        print("  EXT:       %s" % item)


def wait_for_specific_ubx(port, expected_class, expected_id, timeout_seconds):
    deadline = time.time() + timeout_seconds
    deferred_frames = []
    while time.time() < deadline:
        frame = port.read_ubx_frame(max(0.05, min(0.25, deadline - time.time())))
        if frame is None:
            continue
        msg_class, msg_id, payload = frame
        if msg_class == expected_class and msg_id == expected_id:
            while deferred_frames:
                port.unread_frame(deferred_frames.pop())
            return payload
        deferred_frames.append(frame)
    while deferred_frames:
        port.unread_frame(deferred_frames.pop())
    return None


def poll_mon_ver_until_reply(port, verbose, label, timeout_seconds, poll_interval=0.20):
    deadline = time.time() + timeout_seconds
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        verbose_print(verbose, "  MON-VER poll: %s attempt %d" % (label, attempt))
        port.send_ubx(UBX_CLASS_MON, UBX_ID_MON_VER, b"")
        payload = wait_for_specific_ubx(port, UBX_CLASS_MON, UBX_ID_MON_VER, min(poll_interval, max(0.05, deadline - time.time())))
        if payload is not None:
            verbose_print(verbose, "  MON-VER reply: %s %d byte(s)" % (label, len(payload)))
            return payload
    verbose_print(verbose, "  MON-VER reply: %s none" % label)
    return None


def probe_mon_ver_with_rom_training(port, verbose, ack_timeout, training_enabled):
    mon_ver_payload = port.poll(UBX_CLASS_MON, UBX_ID_MON_VER, ack_timeout)
    if mon_ver_payload is not None:
        return mon_ver_payload
    if not training_enabled:
        return None
    verbose_print(verbose, "  initial MON-VER poll failed; trying ROM training sequence 55 55")
    drain_phase(port, verbose, "pre-training drain", 0.05, 0.20)
    deadline = time.time() + max(ack_timeout, DEFAULT_INITIAL_ROM_TRAINING_TIMEOUT)
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        port.write_raw(bytes(bytearray([0x55, 0x55])))
        time.sleep(DEFAULT_INITIAL_ROM_TRAINING_SETTLE_DELAY)
        port.write_raw(make_ubx_packet(UBX_CLASS_MON, UBX_ID_MON_VER, b""))
        raw_window = port.drain_input(0.05, min(DEFAULT_INITIAL_ROM_POLL_WINDOW, max(0.05, deadline - time.time())))
        payload = extract_mon_ver_payload_from_probe_bytes(raw_window)
        if payload is not None:
            verbose_print(verbose, "  MON-VER reply: after ROM training attempt %d, %d byte(s)" % (attempt, len(payload)))
            return payload
        if time.time() < deadline:
            drained = port.drain_input(0.02, 0.05)
            salvaged = extract_mon_ver_payload_from_probe_bytes(drained)
            if salvaged is not None:
                verbose_print(verbose, "  salvaged MON-VER from fragmented ROM text after attempt %d, %d byte(s)" % (attempt, len(salvaged)))
                return salvaged
    verbose_print(verbose, "  ROM training attempts completed: %d" % attempt)
    drained = drain_phase(port, verbose, "post-training drain", 0.05, 0.20)
    salvaged = extract_mon_ver_payload_from_probe_bytes(drained)
    if salvaged is not None:
        verbose_print(verbose, "  salvaged MON-VER from fragmented ROM text after %d attempts, %d byte(s)" % (attempt, len(salvaged)))
        return salvaged
    verbose_print(verbose, "  MON-VER reply: after initial ROM training none")
    return None


def exchange_ubx_expect_payload(port, verbose, label, msg_class, msg_id, payload, reply_class, reply_id, timeout_seconds):
    verbose_print(verbose, "  %s" % label)
    port.send_ubx(msg_class, msg_id, payload)
    reply_payload = wait_for_specific_ubx(port, reply_class, reply_id, timeout_seconds)
    ack_ok = port.expect_ack(msg_class, msg_id, timeout_seconds)
    if reply_payload is None:
        verbose_print(verbose, "  %s reply payload: none" % label)
    else:
        verbose_print(verbose, "  %s reply payload: %d byte(s) [%s]" % (label, len(reply_payload), reply_payload[:24].hex() + ("..." if len(reply_payload) > 24 else "")))
    verbose_print(verbose, "  %s ACK=%s" % (label, "yes" if ack_ok else "no"))
    return reply_payload, ack_ok


def build_cfg_prt_uart1(baud_rate):
    return struct.pack("<BBHIIHHHH", 0x01, 0x00, 0x0000, 0x000008C0, int(baud_rate), 0x0001, 0x0001, 0x0000, 0x0000)


def parse_upd_verify_reply(payload):
    if len(payload) != 5:
        return {"ok": False, "status": None, "payload_hex": payload.hex(), "reason": "unexpected payload length %d" % len(payload)}
    return {"ok": payload[4] == 0x01, "status": payload[4], "payload_hex": payload.hex(), "reason": ""}


def parse_upd_loader_identify_reply(payload):
    if len(payload) != 8:
        return {
            "ok": False,
            "status": None,
            "man_id": None,
            "dev_id": None,
            "payload_hex": payload.hex(),
            "reason": "unexpected payload length %d" % len(payload),
        }
    status = struct.unpack("<I", payload[:4])[0]
    man_id, dev_id = struct.unpack("<HH", payload[4:8])
    return {
        "ok": status == 0,
        "status": status,
        "man_id": man_id,
        "dev_id": dev_id,
        "payload_hex": payload.hex(),
        "reason": "",
    }


def parse_upd_loader_version_reply(payload):
    if len(payload) != 1:
        return {
            "ok": False,
            "version_raw": None,
            "version_text": "",
            "payload_hex": payload.hex(),
            "reason": "unexpected payload length %d" % len(payload),
        }
    version_raw = payload[0]
    return {
        "ok": True,
        "version_raw": version_raw,
        "version_text": "%d.%d" % ((version_raw >> 4) & 0x0F, version_raw & 0x0F),
        "payload_hex": payload.hex(),
        "reason": "",
    }


def parse_upd_status_reply(payload):
    if len(payload) != 5:
        return {"ok": False, "status": None, "value": None, "payload_hex": payload.hex(), "reason": "unexpected payload length %d" % len(payload)}
    return {"ok": payload[4] == 0x01, "status": payload[4], "value": struct.unpack("<I", payload[:4])[0], "payload_hex": payload.hex(), "reason": ""}


def wait_for_upd_status_reply(port, expected_id, timeout_seconds):
    deadline = time.time() + timeout_seconds
    ack_seen = False
    deferred_frames = []
    seen_upd_frames = []
    while time.time() < deadline:
        frame = port.read_ubx_frame(max(0.05, min(0.25, deadline - time.time())))
        if frame is None:
            continue
        msg_class, msg_id, payload = frame
        if msg_class == UBX_CLASS_UPD and msg_id == expected_id:
            while deferred_frames:
                port.unread_frame(deferred_frames.pop())
            reply = parse_upd_status_reply(payload)
            reply["ack_seen"] = ack_seen
            reply["seen_upd_frames"] = list(seen_upd_frames)
            return reply
        if msg_class == UBX_CLASS_UPD:
            seen_upd_frames.append({"msg_id": msg_id, "payload_hex": payload.hex()})
            deferred_frames.append(frame)
            continue
        if msg_class == UBX_CLASS_ACK and msg_id == UBX_ID_ACK_ACK and len(payload) >= 2 and payload[0] == UBX_CLASS_UPD and payload[1] == expected_id:
            ack_seen = True
    while deferred_frames:
        port.unread_frame(deferred_frames.pop())
    return None


def wait_for_chip_erase_complete(port, timeout_seconds):
    deadline = time.time() + timeout_seconds
    ack_seen = False
    deferred_frames = []
    seen_upd_frames = []
    while time.time() < deadline:
        frame = port.read_ubx_frame(max(0.05, min(0.25, deadline - time.time())))
        if frame is None:
            continue
        msg_class, msg_id, payload = frame
        if msg_class == UBX_CLASS_UPD and msg_id == UBX_ID_UPD_CHIP_ERASE:
            while deferred_frames:
                port.unread_frame(deferred_frames.pop())
            return {
                "ok": len(payload) == 1 and payload[0] == 0x01,
                "status": payload[0] if payload else None,
                "payload_hex": payload.hex(),
                "reason": "" if len(payload) == 1 else "unexpected payload length %d" % len(payload),
                "ack_seen": ack_seen,
                "seen_upd_frames": list(seen_upd_frames),
            }
        if msg_class == UBX_CLASS_UPD:
            seen_upd_frames.append({"msg_id": msg_id, "payload_hex": payload.hex()})
            deferred_frames.append(frame)
            continue
        if msg_class == UBX_CLASS_ACK and msg_id == UBX_ID_ACK_ACK and len(payload) >= 2 and payload[0] == UBX_CLASS_UPD and payload[1] == UBX_ID_UPD_CHIP_ERASE:
            ack_seen = True
    while deferred_frames:
        port.unread_frame(deferred_frames.pop())
    return None


def wait_for_upd_verify_reply(port, timeout_seconds):
    deadline = time.time() + timeout_seconds
    ack_seen = False
    while time.time() < deadline:
        frame = port.read_ubx_frame(max(0.05, min(0.25, deadline - time.time())))
        if frame is None:
            continue
        msg_class, msg_id, payload = frame
        if msg_class == UBX_CLASS_UPD and msg_id == UBX_ID_UPD_FLASH_VERIFY:
            reply = parse_upd_verify_reply(payload)
            reply["ack_seen"] = ack_seen
            return reply
        if msg_class == UBX_CLASS_ACK and msg_id == UBX_ID_ACK_ACK and len(payload) >= 2 and payload[0] == UBX_CLASS_UPD and payload[1] == UBX_ID_UPD_FLASH_VERIFY:
            ack_seen = True
    return None


def wait_for_ack_state(port, expected_class, expected_id, timeout_seconds):
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        frame = port.read_ubx_frame(max(0.05, min(0.25, deadline - time.time())))
        if frame is None:
            continue
        msg_class, msg_id, payload = frame
        if msg_class != UBX_CLASS_ACK or len(payload) < 2:
            continue
        if payload[0] != expected_class or payload[1] != expected_id:
            continue
        if msg_id == UBX_ID_ACK_ACK:
            return "ack"
        if msg_id == UBX_ID_ACK_NAK:
            return "nak"
    return None


def wait_for_upd_transfer_feedback(port, expected_id, expected_value, timeout_seconds):
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        frame = port.read_ubx_frame(max(0.05, min(0.25, deadline - time.time())))
        if frame is None:
            continue
        msg_class, msg_id, payload = frame
        if msg_class == UBX_CLASS_ACK and len(payload) >= 2 and payload[0] == UBX_CLASS_UPD and payload[1] == expected_id:
            if msg_id == UBX_ID_ACK_ACK:
                return {"feedback": "ack", "ok": True, "value": None, "payload_hex": ""}
            if msg_id == UBX_ID_ACK_NAK:
                return {"feedback": "nak", "ok": False, "value": None, "payload_hex": ""}
            continue
        if msg_class == UBX_CLASS_UPD and msg_id == expected_id:
            reply = parse_upd_status_reply(payload)
            if reply.get("value") == expected_value:
                reply["feedback"] = "status"
                return reply
    return None


def wait_for_upd_quiet_window(port, verbose, label, quiet_seconds, overall_timeout):
    deadline = time.time() + overall_timeout
    quiet_start = time.time()
    seen_frames = []
    deferred_frames = []
    while time.time() < deadline:
        remaining = deadline - time.time()
        frame = port.read_ubx_frame(min(0.25, max(0.05, remaining)))
        if frame is None:
            if time.time() - quiet_start >= quiet_seconds:
                while deferred_frames:
                    port.unread_frame(deferred_frames.pop())
                verbose_print(verbose, "  %s: quiet %.3fs after %d trailing frame(s)" % (label, quiet_seconds, len(seen_frames)))
                return seen_frames
            continue
        msg_class, msg_id, payload = frame
        seen_frames.append({"msg_class": msg_class, "msg_id": msg_id, "payload_hex": payload.hex()})
        quiet_start = time.time()
        if msg_class != UBX_CLASS_UPD:
            deferred_frames.append(frame)
    while deferred_frames:
        port.unread_frame(deferred_frames.pop())
    verbose_print(verbose, "  %s: timeout waiting for %.3fs quiet after %d trailing frame(s)" % (label, quiet_seconds, len(seen_frames)))
    return seen_frames


def retry_operation(verbose, label, attempts, operation):
    last_result = None
    for attempt in range(1, attempts + 1):
        if attempts > 1:
            verbose_print(verbose, "  %s attempt %d/%d" % (label, attempt, attempts))
        last_result = operation(attempt)
        if last_result:
            return last_result
    return last_result


def require_feedback_result(result, label, allow_nak):
    if result is None:
        raise RuntimeError("%s produced no feedback" % label)
    if result.get("feedback") == "nak" and not allow_nak:
        raise RuntimeError("%s received ACK-NAK" % label)
    if result.get("feedback") == "status" and not result.get("ok", False):
        raise RuntimeError("%s failed: payload=%s%s" % (
            label,
            result.get("payload_hex", ""),
            " (%s)" % result["reason"] if result.get("reason") else "",
        ))
    return result


def find_outstanding_by_value(items, expected_value):
    for index, item in enumerate(items):
        if item["expected_value"] == expected_value:
            return index
    return -1


def consume_transfer_feedback_frame(frame, outstanding_by_id):
    msg_class, msg_id, payload = frame
    if msg_class == UBX_CLASS_ACK and len(payload) >= 2 and payload[0] == UBX_CLASS_UPD and payload[1] in (UBX_ID_UPD_SECTOR_ERASE, UBX_ID_UPD_WRITE_DATA):
        queue = outstanding_by_id[payload[1]]
        if not queue:
            return None
        item = queue[0]
        if msg_id == UBX_ID_ACK_ACK:
            queue.pop(0)
            return {"item": item, "feedback": "ack", "ok": True}
        if msg_id == UBX_ID_ACK_NAK:
            if item.get("retry_on_nak"):
                return {"item": item, "feedback": "nak", "ok": False, "retry_on_nak": True}
            queue.pop(0)
            return {"item": item, "feedback": "nak", "ok": False}
        return None
    if msg_class == UBX_CLASS_UPD and msg_id in (UBX_ID_UPD_SECTOR_ERASE, UBX_ID_UPD_WRITE_DATA):
        reply = parse_upd_status_reply(payload)
        queue = outstanding_by_id[msg_id]
        if reply.get("value") is None:
            return None
        index = find_outstanding_by_value(queue, reply["value"])
        if index < 0:
            return None
        item = queue[index]
        if item.get("defer_status"):
            queue.pop(index)
            return {"defer_frame": frame, "item": item}
        queue.pop(index)
        reply["item"] = item
        reply["feedback"] = "status"
        return reply
    return None


def oldest_outstanding_item(outstanding_by_id):
    oldest = None
    for queue in outstanding_by_id.values():
        if queue and (oldest is None or queue[0]["sent_at"] < oldest["sent_at"]):
            oldest = queue[0]
    return oldest


def pump_transfer_feedback(port, outstanding_by_id, deferred_frames, until_time):
    events = []
    while time.time() < until_time:
        frame = port.read_ubx_frame(min(0.01, max(0.001, until_time - time.time())))
        if frame is None:
            break
        event = consume_transfer_feedback_frame(frame, outstanding_by_id)
        if event is not None:
            if event.get("defer_frame") is not None:
                deferred_frames.append(event["defer_frame"])
                continue
            events.append(event)
    return events


def validate_transfer_events(events):
    for event in events:
        item = event.get("item")
        if item is None:
            continue
        if event.get("feedback") == "nak" and event.get("retry_on_nak"):
            continue
        if event.get("feedback") == "nak" and not item.get("allow_nak"):
            raise RuntimeError("%s received ACK-NAK" % item["label"])
        if event.get("feedback") == "status" and not event.get("ok", False):
            raise RuntimeError("%s failed: payload=%s%s" % (
                item["label"],
                event.get("payload_hex", ""),
                " (%s)" % event["reason"] if event.get("reason") else "",
            ))


def send_with_pipeline_feedback(port, verbose, outstanding_by_id, deferred_frames, item, pipeline_window, packet_feedback_timeout, request_retries):
    while True:
        while sum(len(queue) for queue in outstanding_by_id.values()) >= pipeline_window:
            oldest = oldest_outstanding_item(outstanding_by_id)
            if oldest is None:
                break
            validate_transfer_events(
                pump_transfer_feedback(port, outstanding_by_id, deferred_frames, min(time.time() + 0.01, oldest["sent_at"] + packet_feedback_timeout))
            )
            oldest = oldest_outstanding_item(outstanding_by_id)
            if oldest is None:
                break
            if time.time() - oldest["sent_at"] < packet_feedback_timeout:
                continue
            if oldest["attempts"] >= request_retries:
                raise RuntimeError("%s produced no feedback after %d attempts" % (oldest["label"], oldest["attempts"]))
            oldest["attempts"] += 1
            oldest["sent_at"] = time.time()
            verbose_print(verbose, "  %s retry %d/%d" % (oldest["label"], oldest["attempts"], request_retries))
            port.write_raw(oldest["packet"])
        item["attempts"] = 1
        item["sent_at"] = time.time()
        port.write_raw(item["packet"])
        outstanding_by_id[item["expected_id"]].append(item)
        validate_transfer_events(pump_transfer_feedback(port, outstanding_by_id, deferred_frames, time.time() + 0.002))
        return


def drain_pipeline_feedback(port, verbose, outstanding_by_id, deferred_frames, packet_feedback_timeout, request_retries):
    while True:
        oldest = oldest_outstanding_item(outstanding_by_id)
        if oldest is None:
            return
        validate_transfer_events(
            pump_transfer_feedback(port, outstanding_by_id, deferred_frames, min(time.time() + 0.01, oldest["sent_at"] + packet_feedback_timeout))
        )
        oldest = oldest_outstanding_item(outstanding_by_id)
        if oldest is None:
            return
        if time.time() - oldest["sent_at"] < packet_feedback_timeout:
            continue
        if oldest["attempts"] >= request_retries:
            raise RuntimeError("%s produced no feedback after %d attempts" % (oldest["label"], oldest["attempts"]))
        oldest["attempts"] += 1
        oldest["sent_at"] = time.time()
        verbose_print(verbose, "  %s retry %d/%d" % (oldest["label"], oldest["attempts"], request_retries))
        port.write_raw(oldest["packet"])


def pop_deferred_upd_status_reply(deferred_frames, expected_id, expected_value=None):
    for index, frame in enumerate(deferred_frames):
        msg_class, msg_id, payload = frame
        if msg_class != UBX_CLASS_UPD or msg_id != expected_id:
            continue
        reply = parse_upd_status_reply(payload)
        if expected_value is not None and reply.get("value") != expected_value:
            continue
        deferred_frames.pop(index)
        return reply
    return None


def collect_boot_text(port, quiet_seconds, overall_timeout):
    raw = port.drain_input(quiet_seconds, overall_timeout)
    text = raw.decode("ascii", "replace")
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    return raw, lines


def run_loader_setup(port, baud_cur, baud_safe, verbose, ack_timeout, wait_reconnect, request_retries, probe_info=None):
    verbose_print(verbose, "Loader setup:")
    drain_phase(port, verbose, "pre-sequence drain", 0.05, 0.25)
    if probe_info is not None and probe_info.get("sw_version", "").startswith("ROM BOOT"):
        verbose_print(verbose, "  receiver already in ROM boot; skipping UPD-UPLOAD handoff")
        return
    probe_result = retry_operation(
        verbose,
        "UPD probe state",
        request_retries,
        lambda _attempt: exchange_ubx_expect_payload(
            port,
            verbose,
            "UPD probe state",
            UBX_CLASS_UPD,
            UBX_ID_UPD_PROBE,
            b"",
            UBX_CLASS_UPD,
            UBX_ID_UPD_PROBE,
            ack_timeout,
        ),
    )
    if not probe_result or probe_result[0] is None:
        raise RuntimeError("UPD-PROBE produced no reply")
    cfg_prt_result = retry_operation(
        verbose,
        "CFG-PRT poll",
        request_retries,
        lambda _attempt: exchange_ubx_expect_payload(
            port,
            verbose,
            "CFG-PRT poll",
            UBX_CLASS_CFG,
            UBX_ID_CFG_PRT,
            b"",
            UBX_CLASS_CFG,
            UBX_ID_CFG_PRT,
            ack_timeout,
        ),
    )
    if not cfg_prt_result or cfg_prt_result[0] is None:
        raise RuntimeError("CFG-PRT poll produced no reply")

    def send_upload(_attempt):
        if not port.is_open():
            verbose_print(verbose, "  reopening port before UPD start loader task retry")
            port.reopen(baud_cur)
            drain_phase(port, verbose, "pre-UPD-UPLOAD retry drain", 0.05, 0.20)
        verbose_print(verbose, "  UPD start loader task")
        port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_UPLOAD, b"")
        drain_phase(port, verbose, "UPD start loader task response window", 0.03, ack_timeout)
        return reconnect_port(
            port,
            baud_safe,
            wait_reconnect,
            verbose,
            "post-UPD-UPLOAD",
            min_reopen_delay=DEFAULT_UPLOAD_RECONNECT_MIN_DELAY,
        )

    if not retry_operation(verbose, "UPD-UPLOAD reconnect", request_retries, send_upload):
        raise RuntimeError("post-UPD-UPLOAD reconnect did not complete")
    if DEFAULT_ROM_RECONNECT_SETTLE_DELAY > 0:
        verbose_print(verbose, "  waiting %.2fs for ROM boot after reconnect" % DEFAULT_ROM_RECONNECT_SETTLE_DELAY)
        time.sleep(DEFAULT_ROM_RECONNECT_SETTLE_DELAY)
    verbose_print(verbose, "  raw training sequence 55 55 after ROM reconnect")
    port.write_raw(bytes(bytearray([0x55, 0x55])))
    time.sleep(0.01)
    rom_mon_ver = retry_operation(
        verbose,
        "ROM MON-VER after reconnect",
        request_retries,
        lambda _attempt: poll_mon_ver_until_reply(port, verbose, "after ROM reconnect", max(ack_timeout, 2.5)),
    )
    if rom_mon_ver is None:
        raise RuntimeError("ROM MON-VER poll failed after UPD-UPLOAD reconnect")


def run_loader_prep(port, verbose, ack_timeout, request_retries):
    verbose_print(verbose, "Loader prep:")
    drain_phase(port, verbose, "pre-loader-prep drain", 0.05, 0.25)
    probe_result = retry_operation(
        verbose,
        "UPD probe state",
        request_retries,
        lambda _attempt: exchange_ubx_expect_payload(
            port,
            verbose,
            "UPD probe state",
            UBX_CLASS_UPD,
            UBX_ID_UPD_PROBE,
            b"",
            UBX_CLASS_UPD,
            UBX_ID_UPD_PROBE,
            ack_timeout,
        ),
    )
    if not probe_result or probe_result[0] is None:
        raise RuntimeError("UPD-PROBE produced no reply")
    cfg_prt_result = retry_operation(
        verbose,
        "CFG-PRT poll",
        request_retries,
        lambda _attempt: exchange_ubx_expect_payload(
            port,
            verbose,
            "CFG-PRT poll",
            UBX_CLASS_CFG,
            UBX_ID_CFG_PRT,
            b"",
            UBX_CLASS_CFG,
            UBX_ID_CFG_PRT,
            ack_timeout,
        ),
    )
    if not cfg_prt_result or cfg_prt_result[0] is None:
        raise RuntimeError("CFG-PRT poll produced no reply")

    require_feedback_result(
        retry_operation(
            verbose,
            "UPD-START-LOADER",
            request_retries,
            lambda _attempt: (
                lambda ack_state: {"feedback": ack_state, "ok": ack_state == "ack"} if ack_state else None
            )(
                (
                    port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_UPLOAD, b"\x01"),
                    wait_for_ack_state(port, UBX_CLASS_UPD, UBX_ID_UPD_UPLOAD, ack_timeout),
                )[-1]
            ),
        ),
        "UPD-START-LOADER",
        False,
    )

    def send_loader_version(_attempt):
        port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_LOADER_VERSION, b"")
        reply_payload = wait_for_specific_ubx(port, UBX_CLASS_UPD, UBX_ID_UPD_LOADER_VERSION, ack_timeout)
        ack_state = wait_for_ack_state(port, UBX_CLASS_UPD, UBX_ID_UPD_LOADER_VERSION, ack_timeout)
        if reply_payload is None:
            return None
        reply = parse_upd_loader_version_reply(reply_payload)
        reply["ack_state"] = ack_state
        return reply

    loader_version = retry_operation(verbose, "UPD-LOADER-VERSION", request_retries, send_loader_version)
    if loader_version is None or not loader_version["ok"]:
        raise RuntimeError("UPD-LOADER-VERSION produced no usable reply")
    verbose_print(verbose, "  uploader version %s detected" % loader_version["version_text"])

    verbose_print(verbose, "  CFG-RST stop GPS")
    port.send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_RST, STOP_GPS_CFG_RST_PAYLOAD)
    drain_phase(port, verbose, "CFG-RST stop GPS response window", 0.03, ack_timeout)


def run_flash(port, image_path, baud_cur, baud_upd, verbose, ack_timeout, wait_reconnect, request_retries, packet_feedback_timeout, pipeline_window, flash_xml_path, flash_profile, reset_after_flash, no_fis, chip_erase):
    firmware = load_firmware_payload(image_path)
    if firmware["ubfl"] is not None:
        verbose_print(verbose, "  firmware container images: %s (payload sum %d, file bytes %d)" % (
            ",".join(str(size) for size in firmware["ubfl"]["image_sizes"]),
            firmware["ubfl"]["payload_size"],
            len(firmware["raw_bytes"]),
        ))
    def send_exec(_attempt):
        port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_LOADER_IDENTIFY, struct.pack("<I", 0x48))
        reply_payload = wait_for_specific_ubx(port, UBX_CLASS_UPD, UBX_ID_UPD_LOADER_IDENTIFY, 0.75)
        ack_state = wait_for_ack_state(port, UBX_CLASS_UPD, UBX_ID_UPD_LOADER_IDENTIFY, 0.25)
        if reply_payload is None:
            return None
        reply = parse_upd_loader_identify_reply(reply_payload)
        reply["ack_state"] = ack_state
        return reply

    exec_reply = retry_operation(verbose, "UPD-LOADER-IDENTIFY", request_retries, send_exec)
    if exec_reply is None or not exec_reply["ok"]:
        raise RuntimeError("UPD-LOADER-IDENTIFY produced no usable reply")

    detected_profile = load_flash_device_profile(
        flash_xml_path,
        exec_reply["man_id"] if exec_reply["man_id"] is not None else flash_profile["man_id"],
        exec_reply["dev_id"] if exec_reply["dev_id"] is not None else flash_profile["dev_id"],
    )
    flash_profile = detected_profile
    flashprot_payload = build_flashprot_payload_from_profile(flash_profile)
    merge_fis = not no_fis
    fis_separate = bool(no_fis)
    include_fis_in_flash = merge_fis or fis_separate
    verify_payload = build_upd_verify_payload(image_path, flashprot_payload, include_fis_in_flash)
    data_packets = build_upd_data_packets(
        image_path,
        flashprot_payload,
        DEFAULT_UPD_DATA_BLOCK_SIZE,
        flash_profile["sector_size"],
        merge_fis,
        fis_separate,
    )
    sector_erase_packets = []
    if not chip_erase:
        sector_erase_packets = [
            make_ubx_packet(UBX_CLASS_UPD, UBX_ID_UPD_SECTOR_ERASE, payload)
            for payload in build_linear_sector_erase_payloads(flash_profile["sector_count"], flash_profile["sector_size"])
        ]
    normal_data_packets = data_packets[:-1]
    final_fs_marker_packet = data_packets[-1]
    active_sector_count = 0
    if normal_data_packets:
        last_normal_offset = struct.unpack("<I", normal_data_packets[-1][6:10])[0]
        last_normal_length = struct.unpack("<I", normal_data_packets[-1][10:14])[0]
        active_sector_count = sector_index_for_offset(last_normal_offset + max(0, last_normal_length - 1), flash_profile["sector_size"]) + 1
    final_fs_marker_offset = struct.unpack("<I", final_fs_marker_packet[6:10])[0]
    final_fs_marker_sector = sector_index_for_offset(final_fs_marker_offset, flash_profile["sector_size"])

    verbose_print(verbose, "Flash stage:")
    verbose_print(verbose, "  UPD-LOADER-IDENTIFY detected flash: man_id=0x%04X dev_id=0x%04X ack=%s" % (
        exec_reply["man_id"],
        exec_reply["dev_id"],
        exec_reply["ack_state"] or "none",
    ))
    verbose_print(verbose, "  flash profile: JEDEC=0x%06X sector_size=%d sector_count=%d cfg=0x%08X min_erase_suspend_us=%d xml_revision=%d" % (
        flash_profile["jedec"] & 0xFFFFFF,
        flash_profile["sector_size"],
        flash_profile["sector_count"],
        flash_profile["cfg"],
        flash_profile["min_erase_suspend_us"],
        flash_profile["xml_revision"],
    ))
    metadata_bits = []
    if flash_profile.get("cmdset_name"):
        metadata_bits.append("cmdset=%s" % flash_profile["cmdset_name"])
    if flash_profile.get("supply_label"):
        metadata_bits.append("supply=%s" % flash_profile["supply_label"])
    if "write_suspend" in flash_profile:
        metadata_bits.append("write_suspend=%d" % flash_profile["write_suspend"])
    if metadata_bits:
        verbose_print(verbose, "  flash metadata: %s" % " ".join(metadata_bits))
    verbose_print(verbose, "  FIS handling: %s" % ("merged into image stream" if merge_fis else "sent separately"))
    verbose_print(verbose, "  dynamic scheduler: normal-data=%d active-sectors=%d final-fs-marker-sector=%d" % (
        len(normal_data_packets),
        active_sector_count,
        final_fs_marker_sector,
    ))
    verbose_print(verbose, "  chip erase: %s" % ("yes" if chip_erase else "no"))
    verbose_print(verbose, "  sector-erase packets: %d" % len(sector_erase_packets))
    verbose_print(verbose, "  data packets: %d" % len(data_packets))
    progress_interval = max(1, len(normal_data_packets) // 20) if normal_data_packets else 1

    def send_baud_switch(_attempt):
        verbose_print(verbose, "  CFG-PRT set UART1 baud=%d" % baud_upd)
        port.send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_PRT, build_cfg_prt_uart1(baud_upd))
        drain_phase(port, verbose, "CFG-PRT baud-switch response window", 0.05, ack_timeout)
        port.set_baud(baud_upd)
        return reconnect_port(port, baud_upd, wait_reconnect, verbose, "post-baud-switch")

    if not retry_operation(verbose, "CFG-PRT baud switch", request_retries, send_baud_switch):
        raise RuntimeError("post-baud-switch reconnect did not complete")
    if DEFAULT_BAUD_SWITCH_MONVER_DELAY > 0:
        verbose_print(verbose, "  waiting %.2fs after baud switch" % DEFAULT_BAUD_SWITCH_MONVER_DELAY)
        time.sleep(DEFAULT_BAUD_SWITCH_MONVER_DELAY)

    def send_post_baud_training(_attempt):
        verbose_print(verbose, "  sending captured 0xFF training block + MON-VER")
        port.write_raw((b"\xFF" * 1024) + make_ubx_packet(UBX_CLASS_MON, UBX_ID_MON_VER, b""))
        return wait_for_specific_ubx(port, UBX_CLASS_MON, UBX_ID_MON_VER, 0.80)

    post_baud_mon_ver = retry_operation(verbose, "post-baud MON-VER", request_retries, send_post_baud_training)
    if post_baud_mon_ver is None:
        raise RuntimeError("ROM MON-VER reply missing after 0xFF block + MON-VER")
    verbose_print(verbose, "  0xFF block + MON-VER reply: %d byte(s) [%s]" % (
        len(post_baud_mon_ver),
        post_baud_mon_ver[:24].hex() + ("..." if len(post_baud_mon_ver) > 24 else ""),
    ))

    require_feedback_result(
        retry_operation(
        verbose,
        "UPD-FLASH-GEOMETRY",
        request_retries,
        lambda _attempt: (
            lambda ack_state: {"feedback": ack_state, "ok": ack_state == "ack"} if ack_state else None
        )(
            (
                port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_FLASH_GEOMETRY, flashprot_payload),
                wait_for_ack_state(port, UBX_CLASS_UPD, UBX_ID_UPD_FLASH_GEOMETRY, packet_feedback_timeout),
            )[-1]
        ),
        ),
        "UPD-FLASH-GEOMETRY",
        True,
    )
    time.sleep(DEFAULT_SECTOR_ERASE_DELAY)

    if chip_erase:
        require_feedback_result(
            retry_operation(
                verbose,
                "UPD-CHIP-ERASE",
                request_retries,
                lambda _attempt: (
                    lambda ack_state: {"feedback": ack_state, "ok": ack_state == "ack"} if ack_state else None
                )(
                        (
                            port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_CHIP_ERASE, b""),
                            wait_for_ack_state(port, UBX_CLASS_UPD, UBX_ID_UPD_CHIP_ERASE, packet_feedback_timeout),
                        )[-1]
                ),
            ),
            "UPD-CHIP-ERASE",
            True,
        )
        verbose_print(verbose, "  waiting for chip erase completion")
        chip_erase_reply = wait_for_chip_erase_complete(port, DEFAULT_CHIP_ERASE_STATUS_TIMEOUT)
        if chip_erase_reply is None:
            raise RuntimeError("UPD-CHIP-ERASE produced no completion reply")
        if not chip_erase_reply["ok"]:
            raise RuntimeError("UPD-CHIP-ERASE failed: payload=%s%s" % (
                chip_erase_reply["payload_hex"],
                " (%s)" % chip_erase_reply["reason"] if chip_erase_reply["reason"] else "",
            ))
        verbose_print(verbose, "  chip erase complete")

    outstanding_by_id = {
        UBX_ID_UPD_SECTOR_ERASE: [],
        UBX_ID_UPD_WRITE_DATA: [],
    }
    deferred_frames = []
    erased_sector_count = 0
    data_index = 0

    def send_sector_erase(sector_index):
        packet = sector_erase_packets[sector_index]
        time.sleep(DEFAULT_SECTOR_ERASE_DELAY)
        sector_offset = struct.unpack("<I", packet[6:10])[0]
        send_with_pipeline_feedback(
            port,
            verbose,
            outstanding_by_id,
            deferred_frames,
            {
                "label": "sector-erase %d/%d" % (sector_index + 1, len(sector_erase_packets)),
                "packet": packet,
                "expected_id": UBX_ID_UPD_SECTOR_ERASE,
                "expected_value": sector_offset,
                "allow_nak": False,
            },
            pipeline_window,
            packet_feedback_timeout,
            request_retries,
        )
        if verbose and (sector_index < 4 or (sector_index + 1) % 64 == 0 or (sector_index + 1) == len(sector_erase_packets)):
            verbose_print(verbose, "  sector-erase %d/%d" % (sector_index + 1, len(sector_erase_packets)))

    while data_index < len(normal_data_packets):
        data_packet = normal_data_packets[data_index]
        data_value = struct.unpack("<I", data_packet[6:10])[0]
        required_sector_count = sector_index_for_offset(data_value, flash_profile["sector_size"]) + 1
        if not chip_erase:
            while erased_sector_count < required_sector_count:
                send_sector_erase(erased_sector_count)
                erased_sector_count += 1
        time.sleep(DEFAULT_WRITE_DATA_DELAY)
        send_with_pipeline_feedback(
            port,
            verbose,
            outstanding_by_id,
            deferred_frames,
            {
                "label": "data %d/%d" % (data_index + 1, len(data_packets)),
                "packet": data_packet,
                "expected_id": UBX_ID_UPD_WRITE_DATA,
                "expected_value": data_value,
                "allow_nak": False,
                "retry_on_nak": chip_erase,
            },
            pipeline_window,
            packet_feedback_timeout,
            request_retries,
        )
        data_index += 1
        if verbose and (
            data_index <= 3
            or data_index == len(normal_data_packets)
            or (data_index % progress_interval) == 0
        ):
            verbose_print(
                verbose,
                "  write progress: %d/%d (%.1f%%)" % (
                    data_index,
                    len(normal_data_packets),
                    (100.0 * float(data_index) / float(len(normal_data_packets))) if normal_data_packets else 100.0,
                ),
            )

    if not chip_erase:
        required_sector_count = final_fs_marker_sector + 1
        while erased_sector_count < required_sector_count:
            send_sector_erase(erased_sector_count)
            erased_sector_count += 1

    time.sleep(DEFAULT_WRITE_DATA_DELAY)
    send_with_pipeline_feedback(
        port,
        verbose,
        outstanding_by_id,
        deferred_frames,
        {
            "label": "data %d/%d" % (len(data_packets), len(data_packets)),
            "packet": final_fs_marker_packet,
            "expected_id": UBX_ID_UPD_WRITE_DATA,
            "expected_value": final_fs_marker_offset,
            "allow_nak": True,
            "retry_on_nak": chip_erase,
            "defer_status": True,
        },
        pipeline_window,
        packet_feedback_timeout,
        request_retries,
    )
    drain_pipeline_feedback(port, verbose, outstanding_by_id, deferred_frames, packet_feedback_timeout, request_retries)
    final_data_reply = pop_deferred_upd_status_reply(deferred_frames, UBX_ID_UPD_WRITE_DATA, final_fs_marker_offset)
    while deferred_frames:
        port.unread_frame(deferred_frames.pop())
    if final_data_reply is None:
        final_data_reply = wait_for_upd_status_reply(port, UBX_ID_UPD_WRITE_DATA, DEFAULT_FINAL_STATUS_TIMEOUT)
    if final_data_reply is None:
        verbose_print(verbose, "  final file-system marker: no explicit UPD-WRITE-DATA status reply; continuing after ACK-only completion")
    else:
        verbose_print(verbose, "  final file-system marker status: payload=%s status=%s" % (
            final_data_reply["payload_hex"],
            "0x%02X" % final_data_reply["status"] if final_data_reply["status"] is not None else "<none>",
        ))
    if final_data_reply is not None and not final_data_reply["ok"]:
        raise RuntimeError("final file-system marker UPD-WRITE-DATA failed: payload=%s%s" % (
            final_data_reply["payload_hex"],
            " (%s)" % final_data_reply["reason"] if final_data_reply["reason"] else "",
        ))

    wait_for_upd_quiet_window(
        port,
        verbose,
        "pre-verify quiet window",
        DEFAULT_VERIFY_QUIET_DELAY,
        DEFAULT_VERIFY_QUIET_DELAY + 5.0,
    )

    def send_verify(_attempt):
        port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_FLASH_VERIFY, verify_payload)
        return wait_for_upd_verify_reply(port, ack_timeout + 1.0 + DEFAULT_VERIFY_REPLY_DELAY)

    verify_reply = retry_operation(verbose, "UPD-FLASH-VERIFY", request_retries, send_verify)
    if verify_reply is None:
        raise RuntimeError("UPD-FLASH-VERIFY produced no reply")
    verbose_print(verbose, "  verify reply: payload=%s status=%s ack=%s" % (
        verify_reply["payload_hex"],
        "0x%02X" % verify_reply["status"] if verify_reply["status"] is not None else "<none>",
        "yes" if verify_reply.get("ack_seen") else "no",
    ))
    if not verify_reply["ok"]:
        raise RuntimeError("hardware verify failed: payload=%s%s" % (
            verify_reply["payload_hex"],
            " (%s)" % verify_reply["reason"] if verify_reply["reason"] else "",
        ))

    if int(reset_after_flash):
        time.sleep(0.05)
        retry_operation(
            verbose,
            "UPD-COMPLETE-REBOOT",
            request_retries,
            lambda _attempt: (
                port.send_ubx(UBX_CLASS_UPD, UBX_ID_UPD_COMPLETE_REBOOT, b""),
                drain_phase(port, verbose, "UPD-COMPLETE-REBOOT response window", 0.05, packet_feedback_timeout),
            )[-1],
        )


def resolve_cli_input_path(base_dir, maybe_relative):
    if not maybe_relative:
        return ""
    if os.path.isabs(maybe_relative):
        return maybe_relative
    cwd_candidate = os.path.abspath(maybe_relative)
    if os.path.exists(cwd_candidate):
        return cwd_candidate
    return os.path.abspath(os.path.join(base_dir, maybe_relative))


def parse_baud_sweep(spec):
    if not spec:
        return list(DEFAULT_BAUD_SWEEP)
    values = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        values.append(int(part))
    if not values:
        raise ValueError("baud sweep list is empty")
    return values


def probe_baud_sweep(port_path, baud_rates, verbose, ack_timeout, training_enabled):
    print("Baud sweep on %s:" % port_path)
    for baud_rate in baud_rates:
        print("  trying %d..." % baud_rate)
        port = SerialPort(port_path, baud_rate)
        try:
            mon_ver_payload = probe_mon_ver_with_rom_training(port, verbose, ack_timeout, training_enabled)
            if mon_ver_payload is None:
                continue
            probe_info = parse_mon_ver(mon_ver_payload)
            mon_hw3_payload = port.poll(UBX_CLASS_MON, UBX_ID_MON_HW3, ack_timeout)
            mon_hw3 = parse_mon_hw3(mon_hw3_payload) if mon_hw3_payload is not None else {"safe_boot": False, "hw_version": probe_info["hw_version"]}
            print("  detected baud: %d" % baud_rate)
            return baud_rate
        finally:
            port.close()
    print("  no reply on tested bauds: %s" % ",".join(str(item) for item in baud_rates))
    return None


def parse_args(argv):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description="Minimal Linux firmware updater for the u-blox ZED-F9P.")
    parser.add_argument("image", nargs="?", help="Firmware image path")
    parser.add_argument("-p", required=False, default="", help="Port, e.g. /dev/ttyUSB0")
    parser.add_argument("-F", default="", help="Optional flash XML definition file")
    parser.add_argument("-b", default="115200", help="baudcur[:baudsafe[:baudupd]]")
    parser.add_argument("-s", default="1", help="Enter safeboot before updating")
    parser.add_argument("-v", default="1", help="Verbose mode")
    parser.add_argument("-a", default="1", help="Perform autobauding (1) or use the provided baud directly (0)")
    parser.add_argument("-E", default="0", help="Erase only")
    parser.add_argument("-R", default="1", help="Reset after update completion")
    parser.add_argument("-t", default="1", help="Send the ROM training sequence when needed")
    parser.add_argument("-C", default="0", help="Do chip erase instead of sector erases")
    parser.add_argument("--no-fis", default="0", help="Do not merge the FIS into the image")
    parser.add_argument("--up-ram", default="0", help="Download image to CODE-RAM instead of flash")
    parser.add_argument("--usb-alt", default="0", help="Use USB alternative mode for firmware update")
    parser.add_argument("--probe", action="store_true", help="Probe the receiver and print MON-VER / MON-HW3")
    parser.add_argument("--probe-baud-sweep", action="store_true", help="Probe common baud rates and report the first working one")
    parser.add_argument("--baud-sweep-list", default="", help="Comma-separated baud list for --probe-baud-sweep")
    parser.add_argument("--wait-reconnect", type=float, default=10.0)
    parser.add_argument("--ack-timeout", type=float, default=2.0)
    parser.add_argument("--request-retries", type=int, default=DEFAULT_REQUEST_RETRIES)
    parser.add_argument("--packet-feedback-timeout", type=float, default=DEFAULT_PACKET_FEEDBACK_TIMEOUT)
    parser.add_argument("--pipeline-window", type=int, default=DEFAULT_PIPELINE_WINDOW)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    args.base_dir = base_dir
    args.image = resolve_cli_input_path(base_dir, args.image) if args.image else ""
    args.flash_xml = resolve_cli_input_path(base_dir, args.F) if args.F else ""
    args.verbose = int(args.v)
    args.safeboot = int(args.s)
    args.autobaud = int(args.a)
    args.erase_only = int(args.E)
    args.reset = int(args.R)
    args.training_sequence = int(args.t)
    args.chip_erase = int(args.C)
    args.no_fis = int(args.no_fis)
    args.up_ram = int(args.up_ram)
    args.usb_alt = int(args.usb_alt)
    args.baud_cur, args.baud_safe, args.baud_upd = parse_baud_triplet(args.b)
    args.baud_sweep_list = parse_baud_sweep(args.baud_sweep_list)
    return args


def validate_native_mode_args(args):
    unsupported = []
    if args.erase_only != 0:
        unsupported.append("-E")
    if args.up_ram != 0:
        unsupported.append("--up-ram")
    if args.usb_alt != 0:
        unsupported.append("--usb-alt")
    if unsupported:
        raise SystemExit("native flasher does not support changing %s yet" % ", ".join(unsupported))


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    validate_native_mode_args(args)
    if not args.p:
        raise SystemExit("a port is required (-p /dev/ttyUSB0)")
    if args.image and not os.path.exists(args.image):
        raise SystemExit("image file not found: %s" % args.image)
    if args.flash_xml and not os.path.exists(args.flash_xml):
        raise SystemExit("flash XML file not found: %s" % args.flash_xml)

    flash_profile = load_flash_device_profile(args.flash_xml, DEFAULT_FLASH_MAN_ID, DEFAULT_FLASH_DEV_ID)

    print("----------CMD line arguments-----------")
    print("Image file:        %s" % (args.image or "<none>"))
    print("Flash:             %s" % (args.flash_xml or "<built-in>"))
    print("Fis:               <compiled-in>")
    print("Port:              %s" % args.p)
    print("Baudrates:         %d/%d/%d" % (args.baud_cur, args.baud_safe, args.baud_upd))
    print("Safeboot:          %d" % args.safeboot)
    print("Reset:             %d" % args.reset)
    print("AutoBaud:          %d" % args.autobaud)
    print("Verbose:           %d" % args.verbose)
    print("Erase only:        %d" % args.erase_only)
    print("Training sequence: %d" % args.training_sequence)
    print("Chip erase:        %d" % args.chip_erase)
    print("Merging FIS:       %d" % (0 if args.no_fis else 1))
    print("Update RAM:        %d" % args.up_ram)
    print("Use USB alt:       %d" % args.usb_alt)
    print("Flash mode:        updater")
    print("---------------------------------------")

    if args.dry_run:
        print("Dry run requested.")
        return 0

    if args.probe_baud_sweep:
        detected_baud = probe_baud_sweep(args.p, args.baud_sweep_list, args.verbose, args.ack_timeout, args.training_sequence != 0)
        return 0 if detected_baud is not None else 1

    if args.autobaud:
        detected_baud = probe_baud_sweep(args.p, args.baud_sweep_list, args.verbose, args.ack_timeout, args.training_sequence != 0)
        if detected_baud is None:
            return 1
        args.baud_cur = detected_baud

    port = SerialPort(args.p, args.baud_cur)
    try:
        mon_ver_payload = probe_mon_ver_with_rom_training(port, args.verbose, args.ack_timeout, args.training_sequence != 0)
        if mon_ver_payload is None:
            raise SystemExit("MON-VER poll failed on %s at %d baud" % (args.p, args.baud_cur))
        probe_info = parse_mon_ver(mon_ver_payload)
        mon_hw3_payload = port.poll(UBX_CLASS_MON, UBX_ID_MON_HW3, args.ack_timeout)
        mon_hw3 = parse_mon_hw3(mon_hw3_payload) if mon_hw3_payload is not None else {"safe_boot": False, "hw_version": probe_info["hw_version"]}
        if args.probe or args.verbose:
            print_probe(probe_info, mon_hw3)
        if args.image:
            already_in_rom_boot = probe_info.get("sw_version", "").startswith("ROM BOOT")
            if args.safeboot or not already_in_rom_boot:
                if not args.safeboot and not already_in_rom_boot:
                    verbose_print(args.verbose, "Safeboot disabled, but receiver is not in ROM boot; using UPD-UPLOAD handoff")
                run_loader_setup(port, args.baud_cur, args.baud_safe, args.verbose, args.ack_timeout, args.wait_reconnect, args.request_retries, probe_info)
            else:
                verbose_print(args.verbose, "Safeboot disabled; skipping UPD-UPLOAD handoff")
            run_loader_prep(port, args.verbose, args.ack_timeout, args.request_retries)
            run_flash(
                port,
                args.image,
                args.baud_cur,
                args.baud_upd,
                args.verbose,
                args.ack_timeout,
                args.wait_reconnect,
                args.request_retries,
                args.packet_feedback_timeout,
                args.pipeline_window,
                args.flash_xml,
                flash_profile,
                args.reset,
                args.no_fis,
                args.chip_erase,
            )
    finally:
        port.close()

    if args.image:
        print("Flash completed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
