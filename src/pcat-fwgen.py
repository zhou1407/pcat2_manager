#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import sys
import zlib
import time

def main():
    if len(sys.argv) < 3:
        print('Usage: {0} [FW file] [FW version string]'.format(sys.argv[0]))
        return 0

    fw_ver = sys.argv[2].encode('ascii')
    if len(fw_ver) != 14:
        print('Invalid firmware version string length!')
        return 1

    if not fw_ver.startswith(b'RA2E1'):
        print('Invalid firmware version string!')
        return 1

    fw_raw_data = None
    with open(sys.argv[1], 'rb') as f:
        fw_raw_data = f.read()

    if len(fw_raw_data) == 0:
        print('Empty firmware file!')
        return 2

    fw_crc = zlib.crc32(fw_raw_data)
    fw_raw_len = len(fw_raw_data)
    fw_date = int(time.time())

    fw_size = 0

    with open('pcat2-firmware.bin', 'wb') as f:
        fw_size += f.write(b'ARBDPHC2') #8b
        fw_size += f.write(fw_ver) #14b
        fw_size += f.write(fw_raw_len.to_bytes(4, byteorder='little', signed=False)) #4b
        fw_size += f.write(fw_crc.to_bytes(4, byteorder='little', signed=False)) #4b
        fw_size += f.write(fw_date.to_bytes(8, byteorder='little', signed=False)) #8b

        while fw_size < 512:
            fw_size += f.write(b'\x00')

        fw_size += f.write(fw_raw_data)

    print('Firmware packed successfully, size: {0} bytes.'.format(fw_size))

if __name__ == '__main__':
    main()
