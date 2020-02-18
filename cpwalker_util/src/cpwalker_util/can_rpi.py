#!/usr/bin/python
import rospy
import time
import can

class CANbus(object):
    def __init__(self, channel, bustype='socketcan'):        
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1000000)
        self.buffer = can.BufferedReader()
        self.verbose = rospy.get_param('can_comm/verbose', False)

    def __del__(self):
        self.bus.shutdown()

    """ Send the following data frame to receive a data package from the sensor acquisition board:
        Message type: Standard (11-bit identifier)
        Message identifier (ID): 68
        Pack data: 0
    """
    def send_command(self):
        id = 68
        msg = can.Message(arbitration_id=id, extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])   # 11-bit identifier (not-extended)
        if self.verbose: rospy.loginfo_throttle(0.5,"Sending initialization message (ID: {})!".format(id))
        self.bus.send(msg)
        if self.verbose: rospy.loginfo_throttle(0.5,msg)

    """ Keep hearing CAN port until timeout is reached. This function should be
        used after sending the commanding data frame (previous function)
    """
    def receive_data(self):
        msg = self.bus.recv(0.01) # Timeout in seconds. None: Wait until data is received.
        if msg is None: 
            rospy.logerr('Timeout occurred, no message.')
        elif self.verbose:
            rospy.loginfo_throttle(0.5,msg)
        return msg
