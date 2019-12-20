#!/usr/bin/python
import rospy
import struct

# import acq lib
from std_msgs.msg import Float64

class PotAcq(object):
    def __init__(self, joint_name):

        self.init_parameters()
        self.init_pubs()

    def init_parameters(self):
        self.elevation_topic = self.rospy.get_param("elevation_control/position_topic", "/elevation_signal")

    def init_pubs(self):
        self.elevation_pub = rospy.Publisher(elevation_topic, Float64, queue_size=100)
    
    def update_sensor_data(self):
        return True
    
    def get_position(self):
        return True

    def publish_data(self):
        return True

def main():
    rospy.init_node('encoders_acq_node', anonymous=True)

    rate_param = rospy.get_param("elevation_hw/sampling_frequency", False)
    rate = rospy.Rate(rate_param)
    
    elevation_acq = PotAcq()
    
    rospy.logwarn("[Elevation] Reading sensor data...")
    while not rospy.is_shutdown():
        # Update and publish elevation acquisition data
        elevation_acq.update_sensor_data()
        elevation_acq.publish_data()
    	rate.sleep()
    
    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("[Elevation] Node closed\n")
        sys.stdout.close()
        os.system('clear')
        raise
