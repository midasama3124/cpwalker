#!/usr/bin/python
import rospy
import time
import can

class CANbus(object):
    def __init__(self, id, node_name, channel='can0', bustype='socketcan_native'):
        self.id = id
        self.bustype = bustype
        self.channel = channel
        self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        rospy.init_node('{}_can_node', anonymous=True)

    """Send initialization bus data to start acquiring sensor data
        Message type: Standard (11-bit identifier)
        Message identifier (ID): 68
        Pack data: 0
    """
    def send_init(self):
        # [id, i, 0, 1, 3, 1, 4, 1]
        id = 68
        msg = can.Message(arbitration_id=0xc0ffee, data=[id, 0], extended_id=False)
        self.bus.send(msg)
        # Issue #3: Need to keep running to ensure the writing threads stay alive. ?
        # time.sleep(1)

    def receive_data(self):
        message = self.bus.recv(1.0) # Timeout in seconds.
        if message is None:
                print('Timeout occurred, no message.')
        print(message)

def main():
    # left_hip = CANbus(80)
    # right_hip = CANbus(85)
    # left_knee = CANbus(90)
    RK_module = CANbus(95, 'right_knee')    
    RK_module.send_init()
    while not rospy.is_shutdown():
        RK_module.receive_data()
        rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
