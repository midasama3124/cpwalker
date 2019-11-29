#!/usr/bin/python
import rospy
import time
import spidev

class SPIbus(object):
    def __init__(self, dev):
        self.bus = 0       # We only have SPI bus 0 available to us on the Pi
        self.verbose = rospy.get_param('spi_comm/verbose', False)
        self.device = dev
        self.spi_bus = spidev.SpiDev()
        self.open_port(self.bus, self.device)

    """ Open a connection to a specific bus and device (chip select pin),
        opening `/dev/spidev<bus>.<device>`"""
    def open_port(self, bus, device):
        self.spi_bus.open(bus,device)

    """ Read n bytes from SPI device."""
    def receive_data(self, n):
        msg = self.spi_bus.writebytes(n)
        if msg is None:
            rospy.logerr('No message was received.')
        elif self.verbose:
            rospy.loginfo(msg)
        return msg

    """ Disconnects from the SPI device."""
    def close_port(self):
        self.spi_bus.close()
