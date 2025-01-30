from pymavlink import mavutil

# Connect to pixhawk
connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B003D000F51383130333132-if00', baud=115200)
# Wait for a heartbeat from the Pixhawk
connection.wait_heartbeat()
print("Heartbeat received! System ready.")

# Request the SERVO_OUTPUT_RAW message
def request_servo_output():
    print("Requesting SERVO_OUTPUT_RAW message...")
    # Send a message interval request to the Pixhawk
    connection.mav.command_long_send(
        connection.target_system,    # Target system ID
        connection.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, # Command ID
        0,                           # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, # Message ID
        1000000,                     # Interval in microseconds (1 second)
        0, 0, 0, 0, 0                # Unused parameters
    )

# Function to display the servo output values
def display_servo_output(message):
    print(f"Timestamp: {message.time_usec} µs")
    print(f"Port: {message.port}")
    for i in range(1, 17):  # Loop over servo outputs
        servo_field = f"servo{i}_raw"
        if hasattr(message, servo_field):
            servo_value = getattr(message, servo_field)
            print(f"Servo {i} Output: {servo_value} µs")
    print("-" * 40)

# Main loop to handle incoming messages
request_servo_output()
while True:
    # Wait for a SERVO_OUTPUT_RAW message
    message = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
    if message:
        display_servo_output(message)
