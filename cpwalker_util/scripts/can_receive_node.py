#!/usr/bin/python
import rospy
import time
import can

class CANbus(object):
    def __init__(self, id, node_name, channel='can1', bustype='socketcan'):
        self.id = id
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)
        rospy.init_node('{}_can_node'.format(node_name), anonymous=True)

    """Send initialization bus data to start acquiring sensor data
        Message type: Standard (11-bit identifier)
        Message identifier (ID): 68
        Pack data: 0
    """
    def send_init(self):
        id = 68  
	# for i in range(10):      
	msg = can.Message(arbitration_id=id, extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])   # 11-bit identifier
	#print msg
	#rospy.logwarn("Sending initialization message (ID: {})!".format(id))
	self.bus.send(msg)

    def receive_data(self):
        message = self.bus.recv() # Timeout in seconds.
        if message is None:
                rospy.logerr('Timeout occurred, no message.')
	else:
		rospy.loginfo(message)

def main():
	# module = CANbus(80, 'left_hip')
	# module = CANbus(85, 'right_hip')
	# module = CANbus(90, 'left_knee')
	module = CANbus(95, 'right_knee')
	r = rospy.Rate(2)      # Hz
	while not rospy.is_shutdown():
		#module.send_init()
		#r.sleep()
		module.receive_data()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
