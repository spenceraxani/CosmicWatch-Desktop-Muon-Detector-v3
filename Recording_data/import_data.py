# This script requires one library:
# pyserial
# to install, type: >> pip install pyserial

from __future__ import print_function
import serial 
import time
import glob
import sys
import os
import os.path
import signal
from datetime import datetime
from multiprocessing import Process
import numpy as np
import math
import random

def print_help1():
    print('\n===================== HELP =======================')
    print('This can be used to read data from one or more CW detectors. ')
    print('If you would like to read multiple detectors to a single file,')
    print('separate the desired "Available serial ports" by commas below.\n')
    print('If you have problems, check the following:')
    print('1. Is your detector connected to the serial USB port?\n')
    print('2. Check that you have the correct drivers installed:\n')
    print('\tMacOS: CH340g driver (try: https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver)')
    print('\tWindows: no dirver needed, might need admin permissions')
    print('\tLinux: no driver needed, might need sudo permissions')


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    ComPort.close()     
    file.close() 
    sys.exit(0)

def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
        sys.exit(0)
    result = []
    for port in ports:
        try: 
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

print('\n             Welcome to:   ')
print('CosmicWatch: The Desktop Muon Detector (v3)\n')
print("What would you like to do?")
print("  [1] Record data on the computer")
print("  [h] Help")

if sys.version_info[:3] > (3,0):
    mode = str(input('Select operation: '))
elif sys.version_info[:3] > (2,5,2):
    mode = str(raw_input('Select operation: '))

if mode == 'h':
    print_help1()
    sys.exit()

else:
    mode = int(mode)
    if mode not in [1]:
        print('-- Error --')
        print('Invalid selection')
        print('Exiting...')
        sys.exit()

t1 = time.time()
port_list = serial_ports()
if (time.time()-t1)>2:
    print('Listing ports is taking unusually long. Try disabling your Bluetooth.')

print('\nWhich ports do you want to read from?')
for i in range(len(port_list)):
    print('  ['+str(i+1)+'] ' + str(port_list[i]))
print('  [h] help')

if sys.version_info[:3] > (3,0):
    ArduinoPort = input("Select port: ")
    ArduinoPort = ArduinoPort.split(',')

elif sys.version_info[:3] > (2,5,2):
    ArduinoPort = raw_input("Select port(s): ")

nDetectors = len(ArduinoPort)
port_name_list = []

for i in range(len(ArduinoPort)):
	port_name_list.append(str(port_list[int(ArduinoPort[i])-1]))

if ArduinoPort == 'h':
    print_help1()
    sys.exit()


if mode == 1:
    # Ask for file name:
    cwd = os.getcwd()
    print('')
    if sys.version_info[:3] > (3,0):
        fname = input("Enter file name (default: "+cwd+"/CW_data.txt):")
    elif sys.version_info[:3] > (2,5,2):
        fname = raw_input("Enter file name (default: "+cwd+"/CW_data.txt):")
    detector_name_list = []
    if fname == '':
        fname = cwd+"/CW_data.txt"
    print(' -- Saving data to: '+fname)

    # Make a dictionary for each connected detector
    for i in range(nDetectors):
        time.sleep(0.1)
        globals()['Det%s' % str(i)] = serial.Serial(str(port_name_list[i]),115200)
        time.sleep(0.1)
    file = open(fname, "w")
    
    # Get list of names, using 5 seconds of data.
    det_names = []
    t1 = time.time()
    while (time.time()-t1) < 5:
        for i in range(nDetectors):
            if globals()['Det%s' % str(i)].inWaiting():
                data = globals()['Det%s' % str(i)].readline().decode().replace('\r\n','')    # Wait and read data 
                data = data.split("\t")
                det_names.append(data[-2])
                
    print("\nHere is a list of the detectors I see:")
    det_names = list(set(det_names))
    for i in range(len(det_names)):
        print("  "+str(i+1)+') '+det_names[i])

    # Start recording data to file.
    print("\nTaking data ...")
    print("Press ctl+c to terminate process")
    while True:
        for i in range(nDetectors):
            if globals()['Det%s' % str(i)].inWaiting():
                data = globals()['Det%s' % str(i)].readline().decode().replace('\r\n','')    # Wait and read data 
                data = data.split("\t")
                ti = str(datetime.now()).split(" ")
                comp_time = ti[-1]
                data[1] = comp_time
                comp_date = ti[0].split('-')
                data[2] = comp_date[2] + '/' +comp_date[1] + '/' + comp_date[0] #ti[0].replace('-','/')
                for j in range(len(data)-1):
                    file.write(data[j]+'\t')
                file.write("\n")

    for i in range(nDetectors):
            globals()['Det%s' % str(i)].close()     
    file.close()  



