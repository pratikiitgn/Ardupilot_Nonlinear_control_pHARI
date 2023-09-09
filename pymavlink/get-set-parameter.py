#!/usr/bin/python3
"""
    Source: https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
            https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ
            https://mavlink.io/en/messages/common.html#PARAM_VALUE
            https://mavlink.io/en/messages/common.html#PARAM_SET

    PARAM_REQUEST_LIST: Get all the on-board parameters, vehicle will emit PARAM_VALUE messages back
    PARAM_REQUEST_READ: Request a value of specific parameter
    PARAM_VALUE: Contains a parameter value, only sent when requested or a parameter change
    PARAM_SET: Used to set a specific parameter value
"""

import time
from pymavlink import mavutil

# Connect to Pixhawk over serial port
master = mavutil.mavlink_connection('/dev/ttyACM0')

# Wait for the heartbeat message to confirm the connection
master.wait_heartbeat()

print("pratik")

# Receive messages from Pixhawk
while True:
    msg = master.recv_msg()
    if msg and msg.get_type() == "STATUSTEXT":
        text = msg.text
        if text.startswith("commandGCS:"):
            # Process the message
            print("Received command:", text)
            # You can add your own logic here to handle the received message