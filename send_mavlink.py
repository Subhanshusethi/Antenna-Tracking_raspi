import time
from pymavlink import mavutil

# Create a MAVLink connection
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

while True:
    # Send a heartbeat message
    connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    time.sleep(1)
