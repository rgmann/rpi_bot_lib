// 
// Copyright (c) 2024, Robert Vaughan <<vaughan.t.robert@gmail.com>>
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 

#ifndef ADXL345_INTERFACE_H
#define ADXL345_INTERFACE_H

#include <stdint.h>
#include <cstddef>
#include <string>
#include "error_code.h"

namespace rpi_bot_lib {

class I2cInterface;

class Adxl345Controller {
public:

   static constexpr uint16_t kDeviceAddress = 0x53;

   Adxl345Controller( I2cInterface&, uint16_t address = kDeviceAddress );

   Adxl345Controller() = delete;

   /**
    * Non copyable
    */
   Adxl345Controller( const Adxl345Controller& ) = delete;
   Adxl345Controller& operator= ( const Adxl345Controller& ) = delete;

   /**
    * Initialize the interface to verify that the peripheral is addressable and
    * complete startup configuration.
    *
    * @return error
    */
   error initialize();

   enum AxisType {
      XAxis = 0,
      YAxis = 1,
      ZAxis = 2,
      NumAxis
   };

   enum RangeSetting {
      kRange2g,
      kRange4g,
      kRange8g,
      kRange16g
   };

   /**
    * Configure the sensitivity range for the accelerometer.
    *
    * @param setting Sensitivity range setting
    * @return error
    */
   error set_range_setting(RangeSetting setting);


   struct AccelerationData {
      int16_t x = 0;
      int16_t y = 0;
      int16_t z = 0;
   };

   /**
    * Read a single three-axis acceleration sample. Blocks the caller until the
    * peripheral responds or an error occurs.
    *
    * @param data Reference to acceleration data structure, updated on success
    * @return error
    */
   error read_acceleration_data(AccelerationData& data);


private:

   I2cInterface& i2c_;

   bool initialized_;

   uint16_t address_;

   double gains_[AxisType::NumAxis];
};

}

#endif // ADXL345_INTERFACE_H