#!/usr/bin/python
import rospy
import struct
from scipy import signal

from cpwalker_util.can_rpi import CANbus
from std_msgs.msg import Float64

'''
This code is directly adapted from the exo code, as the communication scheme should be the same.

Also, I'm assuming:
    - all of the sensors are connected to a single CAN module (i.e., a single DAQ);
    - this CAN bus is used exclusively for the elevation acquisition

Right now, the potentiometer is not used at all.

TODO (and this is an important TODO):
Put the filter and equation coeficients into the config files and import them as parameters.
Those values will not be the same for the CPWalker and SWalker and might change with time.
'''

"""Global variables"""
hw_names = ["Potentiometer", "Gauge"]

class SensorAcq(object):
    def __init__(self, joint_name, has_hw):
        self.joint_name = joint_name
        self.verbose = rospy.get_param("elevation_hw/verbose", False)
        self.has_pot = has_hw[0]
        self.has_gauge = has_hw[1]
        """ROS initialization"""
        self.init_params()
        self.init_pubs_()
        self.init_var()

    def init_params(self):
        self.pot_msg = Float64(0.0)
        self.gauge_msg = Float64(0.0)

        ## Gauge
        # Low pass FIR Filter // Considering 100 Hz sampling frequency
        self.filter_gauge_den = 1.0
        self.filter_gauge_num = [-0.000726704529603941, -0.00306014837932883, -0.00727193824711213,	-0.0112533700301156,
                            	-0.00959215560618704, 0.00504821352674820, 0.0378627066672568, 0.0870857819528269,
                            	0.141857621447510, 0.185268649416829, 0.201828298578163, 0.185268649416829, 0.141857621447510,
                            	0.0870857819528269, 0.0378627066672568, 0.00504821352674820, -0.00959215560618704, -0.0112533700301156,
                                -0.00727193824711213, -0.00306014837932883, -0.000726704529603941]
        self.gauge_data_array = [0] * int(len(self.filter_gauge_num))   # past data array initialization

        ## Potentiometer
        # FIR Filter
        # From CPWalker Simulink Model // Note: I'm not sure which sampling frequency is implied here. 
        # Honestly, the filter order seems rather big. Havent tested it as the sensor is not connected
        self.filter_pot_den = 1.0
        self.filter_pot_num = [0.00024042439949025545, 0.00049836390813927627, 0.00097417229437224417, 0.0017000443550953693, 
            0.0027373631842420856, 0.0041421027122268989, 0.0059580488476079062, 0.0082097899804267037, 0.010896842583805376,
            0.013988542477833011, 0.017421366592009706, 0.021099314003761898, 0.024896741653447137, 0.028664469167552876, 
            0.032238878049440665, 0.035452434826228466, 0.038145665650585241, 0.040179625609237338, 0.04144581448006561, 
            0.04187577805775819, 0.04144581448006561, 0.040179625609237338, 0.038145665650585241, 0.035452434826228466,  
            0.032238878049440665, 0.028664469167552876, 0.024896741653447137, 0.021099314003761898, 0.017421366592009706,  
            0.013988542477833011, 0.010896842583805376, 0.0082097899804267037, 0.0059580488476079062, 0.0041421027122268989,
            0.0027373631842420856, 0.0017000443550953693, 0.00097417229437224417, 0.00049836390813927627, 0.00024042439949025545]
        self.pot_data_array = [0] * int(len(self.filter_pot_num))   # past data array initialization

    def init_pubs_(self):
        if self.has_pot:
            self.pot_pub = rospy.Publisher('{}_pot'.format(self.joint_name), Float64, queue_size=100)
        if self.has_gauge:
            self.gauge_raw_pub = rospy.Publisher('{}_gauge_raw'.format(self.joint_name), Float64, queue_size=100)
            self.gauge_force_pub = rospy.Publisher('{}_gauge_weight'.format(self.joint_name), Float64, queue_size=100)

    def init_var(self):
        self.window_size_gauge_filter = 50.0                                 # Running mean filter - window size # Currently unused
        self.gauge_array_filter = [0] * int(self.window_size_gauge_filter)   # Running mean filter               # Currently unused

    def get_sensor_data(self,msg):
        ''' Bytearray decoding returns tuple. Meaning of arguments:
        >: big endian, <: little endian, H: unsigned short (2 bytes), B: unsigned char'''
        if self.has_pot:
            pot_data = struct.unpack('<H', msg.data[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
            self.pot_msg.data = self.pot_processing(pot_data)
            self.pot_pub.publish(self.pot_msg)
            # To debug: print side by side raw and filtered values
            rospy.loginfo_throttle(1,"[Elev Acq] Pot:    {}  {}".format(pot_data,self.pot_msg.data))
        if self.has_gauge:
            gauge_data = struct.unpack('<H', msg.data[2:4])[0]   # 3-4 bytes correspond to strain_gauge reading
            self.gauge_msg.data = self.gauge_processing(gauge_data)
            self.gauge_force_pub.publish(self.gauge_msg)
            self.gauge_raw_pub.publish(gauge_data)
            # To debug: print side by side raw and filtered values
            rospy.loginfo_throttle(1,"[Elev Acq] Gauge:  {}  {}".format(gauge_data,self.gauge_msg.data))

    def pot_processing(self,pot_data):
        pot_processed = self.pot_fir_filter(pot_data)
        return pot_processed

    def gauge_processing(self,gauge_data):
        gauge_filtered = self.gauge_fir_filter(gauge_data)
        gauge_processed = self.gauge_to_force(gauge_filtered)
        return gauge_processed
    
    def pot_fir_filter(self,data):
        # Rolling window
        self.pot_data_array.pop()
        self.pot_data_array.insert(0,data)
        # Filter
        #data_filtered = signal.lfilter(self.filter_pot_num, self.filter_pot_den, self.pot_data_array)
        data_filtered = data
        return data_filtered
    
    def gauge_fir_filter(self,data):
        # Rolling window
        self.gauge_data_array.pop()
        self.gauge_data_array.insert(0,data)
        # Filter
        gauge_filtered_array = signal.lfilter(self.filter_gauge_num, self.filter_gauge_den, self.gauge_data_array)
        gauge_filtered = gauge_filtered_array[-1]
        ''' Rolling average # To delete, use the FIR filter instead
        self.gauge_array_filter.pop()
        self.gauge_array_filter.insert(0,gauge_raw)
        gauge_filtered = sum(self.gauge_array_filter) / self.window_size_gauge_filter
        '''
        return gauge_filtered

    def gauge_to_force(self,data):
        '''
        First (and quick calibration) using the SWalker
        ADC reading from 0 to 1024
        Weights from 0 to 100 // the zero already offsets the supports and exo
        ADC     Weight
        25      0 Kg
        75      5 Kg
        130     10 Kg
        '''
        gauge_force = (data - 25)/10.5 
        return gauge_force

def main():
    rospy.init_node('elevation_acq_node', anonymous=True)
    """Module initialization"""
    if rospy.has_param("elevation_hw/joints/elevation"):
        joint_hw = rospy.get_param("elevation_hw/joints/elevation")
        can_id = rospy.get_param("elevation_hw/joints/elevation/can_id")
        has_hw = []
        for hw_component in hw_names:
            has_hw.append(joint_hw[hw_component])
        if True in has_hw:
            elevation = SensorAcq('elevation', has_hw)
    
    # CAN
    can_channel = rospy.get_param("can_comm/elevation_port", "can1")
    can_bus = CANbus(channel=can_channel)

    rate_param = rospy.get_param("elevation_hw/sampling_frequency", 100)
    rate = rospy.Rate(rate_param) 
    rospy.loginfo("[Elevation Acquisition] Reading sensor data...")
    while not rospy.is_shutdown():
        can_bus.send_command()
        msg = can_bus.receive_data()
        if msg is not None:
            elevation.get_sensor_data(msg)
    	rate.sleep()

    """Clean up ROS parameter server"""
    try:
        rospy.delete_param("exo_hw")
        rospy.delete_param("can_comm")
    except KeyError:
        pass    
    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("[Elevation] Node closed\n")
        sys.stdout.close()
        os.system('clear')
        raise
