import time
# Import mavutil
from pymavlink import mavutil


def wait_conn():
    """
    Sends a ping to establish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6),  # Unix time in microseconds
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)


def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency

    Args:
        message_id (int): MAVLink message ID, use mavutil.mavlink.MAVLINK_MSG_ID_***.
        frequency_hz (float): Desired frequency in Hz as a float
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,  # The MAVLink message ID
        1e6 / frequency_hz,  # The interval between two messages in microseconds. Set to -1 to disable and 0 to
        # request default rate.
        0, 0, 0, 0,  # Unused parameters
        0,  # Target address of message stream (if message has target address fields). 0: Flight-stack default (
        # recommended), 1: address of requestor, 2: broadcast.
    )


# Create the connection
# Companion is already configured to allow script connections under the port 9000
# Note: The connection is done with 'udpout' and not 'udpin'.
# You can check in http:192.168.1.2:2770/mavproxy that the communication made for 9000
# uses a 'udp' (server) and not 'udpout' (client).
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')

# Send a ping to start connection and wait for any reply.
# This function is necessary when using 'udpout',
# as described before, 'udpout' connects to 'udpin',
# and needs to send something to allow 'udpin' to start
# sending data.
wait_conn()

# Configure HEARTBEAT message to be sent at 1Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)

# Configure SYS_STATUS message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)

# Loop to collect information
while True:
    msg = master.recv_match(type="HEARTBEAT")
    
    if not msg:
        continue

    output_file = open("message_data.txt", "w")
    print(msg)
    
    output_file.write(msg)
    output_file.close()
    
    master.mav.heartbeat_send(
        12,
        3,
        81,
        0, 0)
    
    time.sleep(0.1)
