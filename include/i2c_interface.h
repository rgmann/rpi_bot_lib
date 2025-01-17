// 
// Copyright (c) 2015, Robert Glissmann
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

// %% license-end-token %%
// 
// Author: Robert.Glissmann@gmail.com (Robert Glissmann)
// Date: 2015-11-15
// 

#ifndef  I2C_INTERFACE_H
#define  I2C_INTERFACE_H

#include <stdint.h>
#include <cstddef>
#include "error_code.h"

namespace rpi_bot_lib {

class I2cInterface {
public:

   /**
    * Copying is not permitted
    */
   I2cInterface( const I2cInterface& ) = delete;

   /**
    * Copying is not permitted
    */
   I2cInterface& operator= ( const I2cInterface& ) = delete;

   /**
    * Destructor
    */
   ~I2cInterface();

   /**
    * Get a reference to the interface singleton
    *
    * @return I2cInterface*  Pointer to I2C interface instance
    */
   static I2cInterface& instance( const char* device_path = NULL );

   /**
    * Open the I2C interface
    *
    * @return bool True on success; false on failure
    */
   error open( const char* device_path );

   /**
    * Close the I2C interface
    */
   void close();

   /**
    * Check whether the interface is open
    *
    * @return True if open; false otherwise
    */
   bool is_open() const;

   /**
    * Acquire must be called to start a read/write operation.
    *
    * @param device_id Peripheral I2C device ID
    */
   error acquire( uint16_t device_id );

   /**
    * Read a specified number of bytes from the device into the specified
    * memory location.
    *
    * @param buffer Pointer to pre-allocated buffer
    * @param max_bytes Maximum number of bytes to read into the buffer
    * @param bytes_read Actual number of bytes read into the buffer
    * @return error
    */
   error read( void* buffer, size_t max_bytes, size_t& bytes_recvd );

   /**
    * Read a specified number of bytes from the specified address into the
    * specified memory location.
    *
    * @param address Register address to read from
    * @param buffer Pointer to pre-allocated buffer
    * @param max_bytes Maximum number of bytes to read into the buffer
    * @param bytes_read Actual number of bytes read into the buffer
    * @return error
    */
   error read( uint8_t address, void* buffer, size_t max_bytes, size_t& bytes_recvd );

   /**
    * Write a specified number of bytes from a buffer to the device.
    *
    * @param buffer Pointer to pre-allocated buffer
    * @param buffer_size Maximum number of bytes to write
    * @return error
    */
   error write( const void* buffer, size_t buffer_size );

   /**
    * Write a specified number of bytes from a buffer to the device address.
    *
    * @param address device register address
    * @param buffer Pointer to pre-allocated buffer
    * @param buffer_size Maximum number of bytes to write
    * @return error
    */
   error write( uint8_t address, const void* buffer, size_t buffer_size );

private:

   I2cInterface();

private:

   static constexpr int kInvalidHandle = -1;
   
   int handle_;

   uint32_t capabilities_;
};

}

#endif // I2C_INTERFACE_H
