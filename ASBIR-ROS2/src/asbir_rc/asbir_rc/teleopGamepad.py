#!/usr/bin/env python3
# Receives input from controller (/dev/input/event17), 
# converts data into servo motor control range, 
# sends to arduino serial port (/dev/ttyACM0)
import evdev
import serial

def connectBluetooth():
    while True:
        try:
            gamepad = evdev.InputDevice('/dev/input/event260')
        except (FileNotFoundError, PermissionError):
            continue

        print(gamepad)
        return gamepad

def connectSerial():
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', 9600)
            # ser = serial.Serial('/dev/ttyACM1', 9600)
        except FileNotFoundError:
            continue
        else:
            return ser

gamepad = connectBluetooth()
# ser = connectSerial()

# ser.write_timeout=10
fs = 1500
bs = 1500
fw = 1600
bw = 1600

while True:
    try:
        event = gamepad.read_one()
        if event.type==3:
            if event.code==4:
                if event.value > 36000:
                    fw = 2400
                    bw = 2400
                elif event.value < 28000:
                    fw = 800
                    bw = 800
                else:
                    fw = 1600
                    bw = 1600
            elif event.code==3:
                if event.value < 28000:
                    fs = 1500-((32000-event.value)/32000)*(400)
                    bs = 1500+((32000-event.value)/32000)*(400)
                elif event.value > 36000:
                    fs = 1500+((event.value-32000)/32000)*(400) 
                    bs = 1500-((event.value-32000)/32000)*(400)
                else:
                    fs=1500
                    bs=1500
            # print(ser.read())
    except (AttributeError, serial.SerialTimeoutException):
        continue
    control = ("%d,%d,%d,%df\n" %(fs,bs,fw,bw)) 
    print(fs, bs, fw, bw)
    # ser.write(control.encode('utf-8'))