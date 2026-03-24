#!/usr/bin/env python3
import serial
import time

PORT = '/dev/ttyUSB0'

# WitMotion unlock + baud change sequence
CMD_UNLOCK    = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])  # must send first
CMD_SET_BAUD  = bytes([0xFF, 0xAA, 0x04, 0x06, 0x00])  # reg 0x04 = baud, 0x06 = 115200
CMD_SAVE      = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])  # save to flash

print("Opening at 9600...")
ser = serial.Serial(PORT, 9600, timeout=1)
time.sleep(0.5)

print("Sending unlock...")
ser.write(CMD_UNLOCK)
time.sleep(0.1)

print("Sending baud change -> 115200...")
ser.write(CMD_SET_BAUD)
time.sleep(0.1)

print("Saving to flash...")
ser.write(CMD_SAVE)
time.sleep(0.3)

ser.close()
print("Done! Unplug USB for 5 seconds, then replug.")

