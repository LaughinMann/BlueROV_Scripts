import shutil
import time
import serial
import gzip
import hashlib
from pymavlink import mavutil


def wait_conn():
    """
    Sends a ping to establish the UDP communication and awaits for a response
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
    """
    Package and send the message_data.txt file
    """
    # Compress the data file first
    compress_file()
    # generate a hash for the file
    gzip_data_file_hash = get_data_file_hash()
    # send file to modem
    try:
        # open the gzip file
        with gzip.open("message_data.txt.gz", "rb") as data_file:
            data = data_file.read()
        # establish connection to modem
        modem_serial = serial.Serial("CHANGE_ME_TO_MODEM_PORT", baudrate=115200)
        # first send the hash of the file that we will soon receive
        modem_serial.write("<<TRANSFER_START>>")
        time.sleep(0.01);
        modem_serial.write(gzip_data_file_hash)
        time.sleep(0.01);
        # write the message_data.txt.gz file to the modem
        modem_serial.write(data)
        time.sleep(0.01);
        # close the modem serial connection
        modem_serial.write("<<TRANSFER END>>")
        time.sleep(0.01);
        modem_serial.close()
    except serial.serialutil.SerialException:
        print("ERROR: The Modem Serial port not found. Ensure modem is connected and powered on.")


def get_data_file_hash():
    """
    Gets the MD5 hash of the message_data.txt file
    """
    try:
        return hashlib.md5(open("message_data.txt.gz", "rb").read()).hexdigest()
    except FileNotFoundError:
        print("ERROR: Message_data.txt.gz does not exist. File hash reading failed.")


def compress_file():
    """
    Compresses the message_data.txt file to gzip format.
    """
    try:
        data_file = open("message_data.txt", "rb")
        with gzip.open("message_data.txt.gz", "wb") as data_file_compressed:
            shutil.copyfileobj(data_file, data_file_compressed)
    except FileNotFoundError:
        print("ERROR: Message_data.txt file does not exist. Cannot create a compressed file.")

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
            if heartbeat_msg != None and not heartbeat_wrote_to_file:
                print(heartbeat_msg.to_dict())
                output_file.writelines("{}".format(heartbeat_msg.to_dict()))
                heartbeat_wrote_to_file = True
            if sys_status_msg != None and not sys_status_wrote_to_file:
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
    # Sleep for 1 second
    time.sleep(1)
