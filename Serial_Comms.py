"""
Created on April 28, 2024

Define variables for custom communication protocol.
Define functions for serial communications.
"""
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# imports
import serial
import time
import re

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Commands
AIR_PUMP_ON = 'w\n'.encode()
AIR_PUMP_OFF = 's\n'.encode()
AIR_PUMP_LOW = 'e\n'.encode()

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Function definitions

"""
Send a command (character) to the test setup via serial communications.
param serial_port: The serial port to which the test rig controller is connected.
param baud_rate: The baud rate of the serial connection.
param character: The command to be sent to the test setup.
"""
def send_character(serial_port, baud_rate=115200, character='\n'.encode(), return_error = False):
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        
        # Write the specified character to the serial line
        ser.write(character)
        # print(f"Sent {character}")
        
        # Close the serial port
        ser.close()
        
        time.sleep(0.02)

        if return_error:
            return False
        
    except serial.SerialException as e:
        print(f"An error occurred: {e}")
        if return_error:
            return True

"""
Take in a string and return a list of strings
param string: The string to be parsed.
return: A list of strings.
Note: unclear if the 'str_line[2:-3]' is necessary, but it is used in the original code. Altered for leg lifts 9/17/24.
Re Note: the -3 is to deal with '0\\n' - easier to just use values[-1] = values[-1][0]
"""
def parse_string(string, float_only = False):
    if float_only:
        matches = re.findall(r"[-+]?\d*\.\d+|\d+", string)
        float_vals = [float(match) for match in matches]
        return float_vals
    else:
        str_line = str(string.strip())
        # str_line = str_line[2:-3]
        # values = list(map(str.strip, str_line.split(',')))
        values = list(map(str.strip, str_line.split(',')))
        return values

"""
Flush the serial port buffer.
param serial_port: The serial port to which the test rig controller is connected.
param baud_rate: The baud rate of the serial connection.
"""
def flush(serial_port, baud_rate=115200):
    ser = serial.Serial(serial_port, baud_rate, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    time.sleep(0.001)
    while(ser.in_waiting > 0):
        ser.read(ser.in_waiting)
    ser.close()
    

"""
Read a string from the serial port and parse it into a list of values.
param serial_port: The serial port to which the test rig controller is connected.
param baud_rate: The baud rate of the serial connection.
return: A list of [string] values read from the serial port.
"""
def read_state(serial_port, baud_rate=115200, expected_len = 7):
    # # state = pd.DataFrame(columns=['Time','Force','Pressure','L_Height','R_Height','Flow','Limit_Switch'])
    # state = pd.DataFrame(columns=['Time','Current'])
    serialPort = serial.Serial(
        port=serial_port, baudrate=baud_rate, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
    )

    start_time = time.time()

    # read for at least 5ms
    while (time.time()-start_time < 0.005):

        if serialPort.in_waiting > 0:
            # Read data out of the buffer until a carraige return / new line is found
            serialString = serialPort.readline()
            # Print the contents of the serial data
            if(not serialString == b''):
                try:
                    # str_line = str(serialString)
                    values = parse_string(serialString, float_only=False)
                    if len(values) >= expected_len:
                        serialPort.close()
                        # flush(serial_port, baud_rate)
                        return values
                    else:
                        #try 1 more time
                        serialString = serialPort.readline()
                        # str_line = str(serialString)
                        values = parse_string(serialString)
                        # flush(serial_port, baud_rate)
                        serialPort.close()
                        return values
                except:
                    pass
    return 0 # if failed.


"""
Read the height from the serial port for a specified amount of time.
param read_time: The amount of time to read the height from the serial port (seconds)
param serial_port: The serial port to which the test rig controller is connected.
param baud_rate: The baud rate of the serial connection.
param max_recordable_height: The maximum height that can be recorded by the ToF sensor (must be less than error val of 999).
return: [float] The average height of the left and right ToF sensors over the specified time period (transformed via affine transform).
"""
def read_state_for(read_time, serial_port, baud_rate=115200, expected_len = 7):
    # print('Reading height for ' + str(read_time) + ' seconds')
    start_time = time.time()
    inputs = []
    serialPort = serial.Serial(
        port=serial_port, baudrate=baud_rate, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
    )
    time.sleep(0.005)

    # read the serial port for x seconds
    try:
        while (time.time()-start_time < read_time):
            # Wait until there is data waiting in the serial buffer
            if serialPort.in_waiting > 0:
                # Read data out of the buffer until a carraige return / new line is found
                serialString = serialPort.readline()
                # Print the contents of the serial data
                if(not serialString == b''):
                    try:
                        str_line = str(serialString)
                        values = parse_string(str_line)
                        if len(values) >= expected_len:
                            inputs.append(values)
                    except:
                        pass
    except KeyboardInterrupt:
        print('Stopped by user')

    serialPort.close()

    return inputs

"""
Reset the ESP32 microcontroller by sending a reset command.
param serial_port: The serial port to which the test rig controller is connected.
param baud_rate: The baud rate of the serial connection.
No return.
"""
def ESP_Hard_Reset(serial_port, baud_rate=115200, loop_count=0):
    # Reset ESP
    send_character(serial_port, baud_rate, RESET_ESP)

    # Wait for ESP to reset, look for available character
    serialPort = serial.Serial(
        port=serial_port, baudrate=baud_rate, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
    )
    start_time = time.time()
    try:
        while(time.time()-start_time < 15): # give max 15 seconds for reset
            if serialPort.in_waiting > 0: # break once we have contact
                break
    except KeyboardInterrupt:
        print('Stopped by user')  
    serialPort.close()  
    time.sleep(0.5)

    L,R = read_current_height(serial_port, baud_rate)
    # R as primary
    if (R > 800) and loop_count < 2:
        loop_count += 1
        ESP_Hard_Reset(serial_port, baud_rate, loop_count)