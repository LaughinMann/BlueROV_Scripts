import random
import time
from pymavlink import mavutil

# TODO: Modify code to only send it when we get something from the Modem

# Default ip and port for QGroundControl
qgc_ip = "localhost"
qgc_port = 14550

# Establish a connection to QGroundControl
qgc_conn = mavutil.mavlink_connection("udpout:{}:{}".format(qgc_ip, qgc_port), source_system=1, source_component=1)
print("QGC_Modem_Link: Sending Data for QGroundControl on {}:{}".format(qgc_ip, qgc_port))

# Continuously send a HEARTBEAT and SYSTEM_STATUS message to QGroundControl
while True:
    # Create a MAVLink heartbeat message
    msg_heartbeat = mavutil.mavlink.MAVLink_heartbeat_message(
        custom_mode=19,
        system_status=5,
        base_mode=81,
        autopilot=3,
        type=12,
        mavlink_version=3
    )

    print("Sending HEARTBEAT Message to QGroundControl: ", msg_heartbeat)
    # Send the message over the UDP connection to QGroundControl
    qgc_conn.mav.send(msg_heartbeat)

    # Create a MAVLink system status message
    msg_system_status = mavutil.mavlink.MAVLink_sys_status_message(
        onboard_control_sensors_present=321977359,
        onboard_control_sensors_enabled=304126991,
        onboard_control_sensors_health=321952783,
        load=434,
        voltage_battery=random.randint(10000, 15000),
        current_battery=46,
        battery_remaining=99,
        drop_rate_comm=0,
        errors_comm=0,
        errors_count1=0,
        errors_count2=0,
        errors_count3=0,
        errors_count4=0)

    print("Sending SYS_STATUS Message to QGroundControl: ", msg_system_status)
    # Send the message over the UDP connection to QgroundControl
    qgc_conn.mav.send(msg_system_status)

    # Wait for a short period before sending the next set of messages to QGroundControl
    time.sleep(1)
