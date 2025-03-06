################################################################################
# Author: Greg Campbell
#
# Including code from 2017 ROBOTIS CO., LTD. - Author: Ryu Woon Jung (Leon)
#
# *********     Experimental Test Procedure      *********
#
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Current data from an Adafruit INA260 breakout board via ESP32 Pico D4

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from Serial_Comms import *

import time                                    # Use some timers
import serial                                  # Import serial library
import numpy as np                             # Import numpy
import csv                                     # Import csv 
import datetime                                # Import datetime
import pickle                                  # Import pickle

# ESP32 Pico D4 features
esp_serial = '/dev/ttyUSB1'
esp_baud = 115200

# Dictionary details
dict_file = 'test_dict.pkl'
in_num = 2

#############################################################################
# Dynamixel Features
#############################################################################

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38
ADDR_MX_PRESENT_LOAD       = 40
ADDR_MX_LED                = 25
CW_ANGLE_LIMIT             = 6
CCW_ANGLE_LIMIT            = 8


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 35                # Dynamixel ID : 1
BAUDRATE                    = 3000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
SPEED = [100, 1124]
STOP = 0


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

#############################################################################
# Dynamixel Control Functions
#############################################################################
# Toggle LED
def dxl_set_LED(on_off_bool,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_LED = ADDR_MX_LED):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_LED, on_off_bool)
    if dxl_comm_result != COMM_SUCCESS:
        print("LED ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("LED ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Velocity Control - wheel mode only
def dxl_move_speed(speed,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_MOVING_SPEED = ADDR_MX_MOVING_SPEED):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("Speed ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Speed ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Position Control - joint or multiturn mode only
def dxl_move_position(position,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_GOAL_POSITION = ADDR_MX_GOAL_POSITION):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print("Position ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Position ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Set control type - joint/wheel/multiturn
def dxl_set_Control(control,portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, CW_ANGLE_LIMIT = CW_ANGLE_LIMIT, CCW_ANGLE_LIMIT = CCW_ANGLE_LIMIT):
    # Set CW & CCW Limit to 0 (for continuous rotation)
    if control == 'joint':
        CW_VAL = 1
        CCW_VAL = 4095
    elif control == 'wheel':
        CW_VAL = 0
        CCW_VAL = 0
    elif control == 'multiturn':
        CW_VAL = 4095
        CCW_VAL = 4095
    else:
        print("Invalid control type")
        return

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, CW_ANGLE_LIMIT, CW_VAL)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, CCW_ANGLE_LIMIT, CCW_VAL)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return

# Enable Dynamixel Torque
def dxl_set_Torque_enable(on_off_bool, portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, on_off_bool)
    if dxl_comm_result != COMM_SUCCESS:
        print("Torque ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Torque ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return

# Read Dynamixel at given adress (acceptable addresses are 2-byte only)
# Currently works for ADDR_MX_PRESENT_POSITION, ADDR_MX_PRESENT_SPEED, ADDR_MX_PRESENT_LOAD
def dxl_read_address(ADDR, portHandler = portHandler, packetHandler = packetHandler, DXL_ID = DXL_ID):
    dxl_address_value, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR)
    if dxl_comm_result != COMM_SUCCESS:
        print("Position ERROR %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("Position ERROR %s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_address_value


"""
Add data line to .csv file.
param filename: [string] The name of the .csv file (including .csv)
param data: [string] The data line to be added to the .csv file
"""
def append_to_csv(filename, data):
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([data])

"""
Format .csv file (remove double quotes, empty rows, and header row).
param filename: [string] The name of the .csv file (including .csv)
"""
def clean_csv(filename):
    lines = []
    with open(filename, 'r') as file:
        for line in file:
            # Remove double quotes from the line
            line = line.replace('"', '')
            # Check if the line is empty (we accept 'ovf' deal with it in post-processing)
            if line.strip() != '':
                lines.append(line)

    with open(filename, 'w') as file:
        file.writelines(lines)

    with open(filename, 'r', newline='') as file:
        reader = csv.reader(file)
        rows = list(reader)

    for i in range(1, len(rows)):
        rows[i][0] = int(rows[i][0]) - int(rows[0][0])
    
    rows[0][0] = 0

    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(rows)

"""
Main function to run an experiment (inflate and deflate at a specific height, write data to a .csv)
param height_number: The height of the test rig in mm.
param test_number: The number of the current trial (generally 0-2)
param data_path: The path to the directory where data should be saved (os.path.join(parent_dir, sample_string))
param Burst_Pressure: The pressure at which the test rig should stop inflating (PSI), see initial test set for material values
param MEASURE_DEFLATION: Whether or not to measure deflation (True/False)
"""
def run_experiment_sec(test_number,data_path,time_sec=5,serial_port = esp_serial,baud_rate=esp_baud):
    serialString = ""  # Used to hold data coming over UART
    
    this_experiment = str(test_number)
    this_trial_path = os.path.join(data_path,this_experiment)
    file_name = this_trial_path + ".csv"

    # Open serial port (we will hold it open for the duration of the experiment)
    serialPort = serial.Serial(
        port=serial_port, baudrate=baud_rate, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
    )
    time.sleep(0.1)

    start_time = time.time()

    # Record data throughout inflation
    try:
        while (time.time()-start_time < time_sec):
            try:
                # Wait until there is data waiting in the serial buffer
                if serialPort.in_waiting > 0:
            
                    # Read data out of the buffer until a carraige return / new line is found
                    serialString = serialPort.readline()
                    
                    # Print the contents of the serial data
                    if(not serialString == b'' ):
                        try:
                            append_to_csv(file_name,serialString.decode("Ascii"))
                        except:
                            pass
                    values = parse_string(str(serialString))
                    # Check for contact
                    if(0 == float(values[-1])):
                        break
            except KeyboardInterrupt:
                print('Stopped by user')
    except KeyboardInterrupt:
        serialPort.close()
        print('Stopped by user')
    
    # Stop inflation and release air
    serialPort.write(AIR_PUMP_OFF)
    # print("Air pump off.")
    time.sleep(0.25)
    serialPort.write(AIR_RELEASE_ON)
    
    # Deflate time equal to inflate time
    start_time = time.time()
    double_tap = False
    Stop_Record = False
    stopDeflate = 0
    manualOverride = 0
    try:
        while (time.time()-start_time < time_sec):
            try:
                if serialPort.in_waiting > 0:
                    line = serialPort.readline()
                    values = parse_string(line)
                    
                    if len(values) >= 6:
                        P_PSI = float(values[2])
                        if(P_PSI <= 0.001):
                            if stopDeflate >= 5: #avoid a noise-based reading. -GMC 8/29/24
                                break
                            stopDeflate += 1
                        #Stop recording on contact break:
                        F_up = float(values[1])
                        if F_up>=-0.04 and not Stop_Record: 
                            Stop_Record = True

                        # Manual Override for Deflation
                        if (np.logical_and(float(values[6]) == 0, P_PSI < 0.3)):
                            if manualOverride == 15: #Doesnt usually press the limit switch longer than 15 reads
                                break
                            manualOverride += 1
                    if time.time()-start_time >= 1 and values[0]==0 and not double_tap:
                        serialPort.write(AIR_RELEASE_ON)
                        double_tap = True
            except Exception as e:
                print("Error reading data:", e)
            except KeyboardInterrupt:
                print('Stopped by user')
    except KeyboardInterrupt:
        print('Stopped by user')

    #send_character(serial_port, baud_rate, AIR_RELEASE_OFF)   
    time.sleep(1)
    serialPort.write(AIR_RELEASE_OFF)
    serialPort.close()
        
    # Note - the following only works if you strap pin 15 of ESP32 to GND (supressing init message).
    clean_csv(file_name)


#################################################
# Main Loop
#################################################

# Set control type
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

dxl_set_Control('wheel')
dxl_set_Torque_enable(True)

# flush serial buffer
flush(esp_serial, baud_rate=esp_baud)

# create an empty array
dxl_inputs = np.zeros([0,2]).astype(int)
esp_inputs = np.zeros([0,1]).astype(int)

while(1):
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # # Write goal speed
    dxl_set_LED(True)
    dxl_move_speed(SPEED[index])
    start_time = time.time()
    while(time.time()-start_time < 2):
        # Too slow to read speed & position as well (~15ms per read (!!))
        # speed = dxl_read_address(ADDR_MX_PRESENT_SPEED)
        # pos = dxl_read_address(ADDR_MX_PRESENT_POSITION)

        # Read load
        load = int(dxl_read_address(ADDR_MX_PRESENT_LOAD))

        # Read current
        in_state = read_state(esp_serial, baud_rate=esp_baud, expected_len=2)
        try:
            in_current = float(in_state[1][:-3])
        except:
            in_current = 0
        # append
        dxl_inputs = np.vstack((dxl_inputs,[int((time.time()-start_time)*1000),load]))
        esp_inputs = np.vstack((esp_inputs,[in_current]))
        # print("Speed: %d, Position: %d, Load: %d" % (speed, pos, load))

    dxl_set_LED(False)
    dxl_move_speed(STOP)

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

dxl_set_Torque_enable(False)

portHandler.closePort()

all_inputs = np.hstack((dxl_inputs,esp_inputs))

# add to existing dictionary
key_name = 'test' + str(in_num)

os.chdir('Mod_DOE')
# dict_set = {}
dict_set = pickle.load(open(dict_file, "rb"))
# see if key exists
if key_name in dict_set.keys():
    print("Key exists, appending")
    dict_set[key_name].append(all_inputs)
else:
    dict_set[key_name] = [all_inputs] # save as a list
pickle.dump(dict_set, open(dict_file, "wb"))


# shape : [[nx1,nx1],nx1,nx1] - n is number of samples
# shape : [mxn,mxn,mxn,mxn ... ] - m is number of TRIALS (... is number of reported values)
    # concatenate over m trials - > [nx1,nx1,nx1,nx1] (samples)
        # report single value: [1,0] - cost (i.e. max(current))