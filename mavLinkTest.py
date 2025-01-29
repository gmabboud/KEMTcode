from pymavlink import mavutil

# Connect to pixhawk
connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B003D000F51383130333132-if00', baud=115200)

# Confirm heartbeat
connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Print our MAVLink messgaes
while True:
    message = connection.recv_match()
    if message:
        print(message)
