#!/usr/bin/python
import rospy
import struct
from cpwalker_util.spi_rpi import SPIbus
from std_msgs.msg import UInt16

"""Global variables"""
joints = ["right_wheel", "left_wheel"]

class EncoderAcq(object):
    def __init__(self, node_name, has_hw):
        self.node_name = node_name
        self.verbose = rospy.get_param("traction_hw/verbose", False)
        self.device = rospy.get_param("spi_comm/{}/spidev".format(self.node_name), 1)
        self.spi_bus = SPIbus(dev=self.device)      # SPI bus
        """ROS initialization"""
        rospy.init_node('{}_acq_node'.format(node_name), anonymous=True)
        self.init_pubs_()

    def init_pubs_(self):
        self.enc_pub = rospy.Publisher('{}_encoder'.format(self.node_name), UInt16, queue_size=100)

    def get_sensor_data(self):
        msg = self.spi_bus.receive_data()
        if msg is not None:
            ''' Bytearray decoding returns tuple. Meaning of arguments:
            >: big endian, <: little endian, H: unsigned short (2 bytes), B: unsigned char'''
            enc_msg = struct.unpack('>H', msg[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
            self.enc_pub.publish(enc_msg)

def main():
    """Module initialization"""
    for joint_name in joints:
        if rospy.has_param("traction_hw/joints/{}".format(joint_name)):
            joint_hw = rospy.get_param("exo_hw/joints/{}".format(joint_name))
            has_hw = []
            for hw_component in hw_names:
                has_hw.append(joint_hw[hw_component])
            if True in has_hw:
                exec("{} = SensorAcq('{}', has_hw)".format(joint_name, joint_name))

    rate = rospy.Rate(100)      # Hz
    rospy.logwarn("Reading sensor data...")
    while not rospy.is_shutdown():
        right_wheel.get_sensor_data()
    	rate.sleep()

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
