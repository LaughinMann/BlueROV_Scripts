import time
import serial
import gzip
import hashlib
from pymavlink import mavutil


def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        send_ping()
        msg = master.recv_match()
    time.sleep(0.5)


def request_message_interval(master, message_input, frequency_hz):
    """
    Set request interval for specific messages
    """
    message_name = "MAVLINK_MSG_ID_" + message_input
    message_id = getattr(mavutil.mavlink, message_name)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,
        1e6 / frequency_hz,
        0,
        0, 0, 0, 0)


def send_ping():
    """
    Send a ping to the BlueROV to make sure it knows we're still here
    """
    master.mav.ping_send(
        int(time.time() * 1e6),
        0,
        0,
        0
    )

def send_file_to_modem():
    print("TEST: Send data to modem")

def generate_md5():
    pass

def compress_file():
    pass

# Create the connection
#  Companion is already configured to allow script connections under the port 9000
# Note: The connection is done with 'udpout' and not 'udpin'.
#  You can check in http:192.168.1.2:2770/mavproxy that the communication made for 9000
#  uses a 'udp' (server) and not 'udpout' (client).
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')

# Send a ping to start connection and wait for any reply.
#  This function is necessary when using 'udpout',
#  as described before, 'udpout' connects to 'udpin',
#  and needs to send something to allow 'udpin' to start
#  sending data.
wait_conn()

# Set frequency of the messages to receive
request_message_interval(master, "HEARTBEAT", 1)
request_message_interval(master, "SYS_STATUS", 2)

# open output file, or create one if it somehow does not exist
heartbeat_wrote_to_file = False
sys_status_wrote_to_file = False

# Get some information!
while True:
    # open file
    output_file = open("message_data.txt", "w")

    while heartbeat_wrote_to_file == False or sys_status_wrote_to_file == False:
        # filter out the messages we want
        heartbeat_msg = master.recv_match(type="HEARTBEAT")
        sys_status_msg = master.recv_match(type="SYS_STATUS")

        # if the message is not a heartbeat or sys_status ignore and continue loop
        if not heartbeat_msg and not sys_status_msg:
            continue

            # Make sure no None types are printed
            if (heartbeat_msg != None and not heartbeat_wrote_to_file):
                print(heartbeat_msg.to_dict())
                output_file.writelines("{}".format(heartbeat_msg.to_dict()))
                heartbeat_wrote_to_file = True
            if (sys_status_msg != None and not sys_status_wrote_to_file):
                print(sys_status_msg.to_dict())
                output_file.writelines("{}".format(sys_status_msg.to_dict()))
                sys_status_wrote_to_file = True

        # Send a ping to the BlueROV to make sure it knows we're still here
        send_ping()

    # close file
    output_file.close()

    # send file to modem
    send_file_to_modem()
    # example reset to send another message
    heartbeat_wrote_to_file = False
    sys_status_wrote_to_file = False

    # Send a ping to the BlueROV to make sure it knows we're still here
    send_ping()
    # Sleep for 0.1 seconds
    time.sleep(1)
