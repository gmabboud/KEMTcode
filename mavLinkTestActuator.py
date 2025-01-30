from pymavlink import mavutil

# Connect to pixhawk
connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B003D000F51383130333132-if00', baud=115200)

# Wait for a heartbeat from the Pixhawk to confirm communication
connection.wait_heartbeat()
print("Heartbeat received! System ready.")

# Request the ACTUATOR_OUTPUT_STATUS message
def request_actuator_output_status():
    print("Requesting ACTUATOR_OUTPUT_STATUS message...")
    # Send a message interval request to the Pixhawk
    connection.mav.command_long_send(
        connection.target_system,    # Target system ID
        connection.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # Command ID
        0,                           # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, # Message ID
        1000000,                     # Interval in microseconds (1 second)
        0, 0, 0, 0, 0                # Unused parameters
    )

# Function to display actuator output values
def display_actuator_output_status(message):
    print(f"Timestamp: {message.time_usec} µs")
    print(f"Active Outputs: {message.active}")

    for i, value in enumerate(message.actuator):
        if value != 0:  # Ignore unused channels (zero values)
            print(f"Actuator {i + 1} Output: {value}")

    print("-" * 40)

# Main loop to handle incoming messages
request_actuator_output_status()
while True:
    # Wait for an ACTUATOR_OUTPUT_STATUS message
    message = connection.recv_match(type='ACTUATOR_OUTPUT_STATUS', blocking=True)
    if message:
        display_actuator_output_status(message)
