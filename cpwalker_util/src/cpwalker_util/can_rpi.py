#!/usr/bin/python
import rospy
import time
import can

class CANbus(object):
    def __init__(self, id, channel='can1', bustype='socketcan'):
        self.id = id
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        self.verbose = rospy.get_param('can_comm/verbose', False)

    """ Send the following data frame to receive a data package from the sensor acquisition board:
        Message type: Standard (11-bit identifier)
        Message identifier (ID): 68
        Pack data: 0
    """
    def send_command(self):
        id = 68
        msg = can.Message(arbitration_id=id, extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])   # 11-bit identifier (not-extended)
        if self.verbose: rospy.logwarn("Sending initialization message (ID: {})!".format(id))
        self.bus.send(msg)
        if self.verbose: rospy.info(msg)

    """ Keep hearing CAN port until timeout is reached. This function should be
        used after sending the commanding data frame (previous function)
    """
    def receive_data(self):
        msg = self.bus.recv(1) # Timeout in seconds. None: Wait until data is received.
        if msg is None:
            rospy.logerr('Timeout occurred, no message.')
        elif self.verbose:
            rospy.loginfo(msg)
        return msg
