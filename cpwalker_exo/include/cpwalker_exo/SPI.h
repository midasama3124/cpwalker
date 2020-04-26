/**
 * \file SPI.h
 *
 * \date Created mar 17, 2020
 * \author Pablo Romero <p.romero.sorozabal@gmail.com>
 */


#ifndef CPWALKER_EXO_SPI_H_
#define CPWALKER_EXO_SPI_H_

#include <iostream>
#include <bcm2835.h>
#include "ros/ros.h"
#include <string>
#include <stdlib.h>

class SPI {
 public:
   SPI();

   //Initiate bc2835 and set up spi.
   bool init();

   //Send data to the selected joint.
   void sendData(std::string joint, float voltage);

   //End bcm2835 and spi.
   void end();

   ~SPI();

 private:
   uint16_t data_;
   float last_voltage_;
};

#endif  // CPWALKER_EXO_SPI_H_
