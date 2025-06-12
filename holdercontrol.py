#!/usr/bin/python3

import serial
import time

# Commands
# 1: drop / pull
# 2: 
# 3: set cord height

# ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
arduino  = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
# print(ser.name)
# ser.write('1'.encode())
# time.sleep(0.1)  # Small delay to allow response
# result = ser.readall()
# print(result)
# ser.close()

# Read and print the startup menu
time.sleep(1)  # Wait briefly for the menu to be transmitted
# while arduino.in_waiting > 0:  # Check for available data
#     menu_line = arduino.readline().decode().strip()  # Read and decode a line
#     print(menu_line)  # Display the menu output

# Wait until there is input available
while True:
    if arduino.in_waiting > 0:  # Check if there is data to read
        break  # Exit loop once data is detected

while True:
    while arduino.in_waiting > 0:  # Check for available data
        menu_line = arduino.readline().decode().strip()  # Read and decode a line
        print(menu_line)  # Display the menu output
    # Get user input for command
    command = input("Enter a command to send to Arduino: ").strip()

    # Now you can send a command
    # command = '1'  # Example single-character command
    arduino.write(command.encode())

    # time.sleep(0.1)  # Small delay to allow response
    print('waiting for result')
    while True:
        if arduino.in_waiting > 0:  # Check if there is data to read
            break  # Exit loop once data is detected

    # result = arduino.readline().decode()
    # print(result)

# Close connection
arduino.close()