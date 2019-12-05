#!/usr/bin/python
import rospy
from time import sleep
import spidev

class SPIbus():

    #-------------------------------------------
    # Constants

    #   Commands
    CLEAR_COUNTER = 0x20
    CLEAR_STATUS = 0x30
    READ_COUNTER = 0x60
    READ_STATUS = 0x70
    WRITE_MODE0 = 0x88
    WRITE_MODE1 = 0x90

    #   Modes
    FOURX_COUNT = 0x03

    FOURBYTE_COUNTER = 0x00
    THREEBYTE_COUNTER = 0x01
    TWOBYTE_COUNTER = 0x02
    ONEBYTE_COUNTER = 0x03

    BYTE_MODE = [ONEBYTE_COUNTER, TWOBYTE_COUNTER, THREEBYTE_COUNTER, FOURBYTE_COUNTER]

    #   Values
    max_val = 4294967295

    # Global Variables

    counterSize = 4 #Default 4

    #----------------------------------------------
    # Constructor

    def __init__(self, CSX, CLK, BTMD):
        self.counterSize = BTMD #Sets the byte mode that will be used


        self.spi = spidev.SpiDev() #Initialize object
        self.spi.open(0, CSX) #Which CS line will be used
        self.spi.max_speed_hz = CLK #Speed of clk (modifies speed transaction)

        #Init the Encoder
        print 'Clearing Encoder CS%s\'s Count...\t' % (str(CSX)), self.clearCounter()
        print 'Clearing Encoder CS%s\'s Status..\t' % (str(CSX)), self.clearStatus()

        self.spi.xfer2([self.WRITE_MODE0, self.FOURX_COUNT])

        sleep(.1) #Rest

        self.spi.xfer2([self.WRITE_MODE1, self.BYTE_MODE[self.counterSize-1]])

    def close(self):
        self.spi.close()

    def clearCounter(self):
        self.spi.xfer2([self.CLEAR_COUNTER])

        return '[DONE]'

    def clearStatus(self):
        self.spi.xfer2([self.CLEAR_STATUS])

        return '[DONE]'

    def readCounter(self):
        readTransaction = [self.READ_COUNTER]

        for i in range(self.counterSize):
            readTransaction.append(0)

        data = self.spi.xfer2(readTransaction)

        EncoderCount = 0
        for i in range(self.counterSize):
            EncoderCount = (EncoderCount << 8) + data[i+1]

        if data[1] != 255:
            return EncoderCount
        else:
            return (EncoderCount - (self.max_val+1))

    def readStatus(self):
        data = self.spi.xfer2([self.READ_STATUS, 0xFF])

        return data[1]
