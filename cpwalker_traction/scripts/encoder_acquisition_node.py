#!/usr/bin/python
import rospy
import struct
from cpwalker_util.spi_rpi import SPIbus
from std_msgs.msg import Int32

"""Global variables"""
joints = ["right_wheel", "left_wheel"]

class EncoderAcq(object):
    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.verbose = rospy.get_param("traction_hw/verbose", False)
        self.device = rospy.get_param("traction_hw/joints/{}/spi_dev".format(self.joint_name))
        print "Joint: {}, Device: {}".format(self.joint_name,self.device)
        n_bytes = 4
        clk_speed = 1000000
        self.spi_bus = SPIbus(self.device, clk_speed, n_bytes)      # SPI bus
        """ROS initialization"""
        self.init_pubs_()

    def init_pubs_(self):
        self.enc_pub = rospy.Publisher('{}_encoder'.format(self.joint_name), Int32, queue_size=100)

    def get_sensor_data(self):
        enc_count = self.spi_bus.readCounter()
        if enc_count is not None:
            if self.verbose: rospy.loginfo("Encoder data: {}".format(enc_count))
            self.enc_pub.publish(enc_count)
        else:
            rospy.logerr("No message received")

def main():
    rospy.init_node('encoders_acq_node', anonymous=True)
    """Module initialization"""
    for joint_name in joints:
        if rospy.has_param("traction_hw/joints/{}".format(joint_name)):
            joint_hw = rospy.get_param("traction_hw/joints/{}".format(joint_name))
            exec("{} = EncoderAcq('{}')".format(joint_name, joint_name))

    rate = rospy.Rate(100)      # TODO add this as parameter
    rospy.loginfo("[Encoders] Reading sensor data...")
    while not rospy.is_shutdown():
        if 'right_wheel' in locals(): right_wheel.get_sensor_data()
    	if 'left_wheel' in locals(): left_wheel.get_sensor_data()
    	rate.sleep()
    right_wheel.spi_bus.close()
    left_wheel.spi_bus.close()

    """Clean up ROS parameter server"""
    try:
        rospy.delete_param("exo_traction")
        rospy.delete_param("spi_comm")
    except KeyError:
        pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
