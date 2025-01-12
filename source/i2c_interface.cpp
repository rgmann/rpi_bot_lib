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

// #include <glib.h>
// #include <glib/gprintf.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sstream>

#include "i2c_interface.h"

using namespace coral;
using namespace rpi_bot_lib;

//-----------------------------------------------------------------------------
I2cInterface::I2cInterface()
   : handle_( I2cInterface::kInvalidHandle )
   , capabilities_( 0 )
{
}

//-----------------------------------------------------------------------------
I2cInterface::~I2cInterface()
{
   I2cInterface::close();
}

//-----------------------------------------------------------------------------
I2cInterface& I2cInterface::instance( const char* device_path )
{
   static I2cInterface our_instance;

   if ( ( our_instance.handle_ == I2cInterface::kInvalidHandle ) && device_path )
   {
      our_instance.open( device_path );
   }

   return our_instance;
}

//-----------------------------------------------------------------------------
bool I2cInterface::open( const char* device_path )
{
   i2c_error error;

   if ( handle_ == kInvalidHandle )
   {
      handle_ = ::open( device_path, O_RDWR );

      // Get capabilities for future reference.
      if ( is_open() )
      {
         // Retrieve the available I2C functionality.
         if ( ioctl( handle_, I2C_FUNCS, &capabilities_ ) < 0 )
         {
            capabilities_ = 0;
         }
      }
      else
      {
         std::stringstream message;
         message << "I2cInterface::open: Failed to open device - "
                 << strerror(errno)
                 << std::endl;

         error.code = kOpenError;
         error.message = message.str();
      }
   }

   return error;
}

//-----------------------------------------------------------------------------
void I2cInterface::close()
{
   if ( is_open() )
   {
      ::close( handle_ );
      handle_ = kInvalidHandle;
   }
}

//-----------------------------------------------------------------------------
bool I2cInterface::is_open() const
{
   return ( handle_ > kInvalidHandle );
}

//-----------------------------------------------------------------------------
I2cInterface::i2c_error I2cInterface::acquire( uint16_t device_id )
{
   i2c_error error;

   if ( is_open() )
   {
      uint32_t device_id_ul = static_cast<uint32_t>(device_id);

      if ( ( device_id_ul & 0xFFFFFF00 ) &&
           ( ( capabilities_ & I2C_FUNC_10BIT_ADDR ) == 0 ) )
      {
         error.code = kInvalidAddessMode;
         error.message = "I2cInterface::acquire: 10-bit addresses not supported.\n";
      }
      else if ( ioctl( handle_, I2C_SLAVE, device_id_ul ) < 0 )
      {
         error.code = kInvalidDeviceID;
         error.message = "I2cInterface::acquire: Invalid device ID\n";
      }
   }
   else
   {
      error.code = kInvalidDeviceHandle;
      error.message = "I2cInterface::acquire: Invalid device handle\n";
   }

   return error;
}

//-----------------------------------------------------------------------------
I2cInterface::i2c_error I2cInterface::read(
   void*    buffer,
   size_t   max_bytes,
   size_t&  bytes_recvd
)
{
   i2c_error error;

   bytes_recvd = 0;

   if ( is_open() )
   {
      int byte_rcvd_count = ::read( handle_, buffer, max_bytes );

      if ( byte_rcvd_count > 0 )
      {
         bytes_recvd = byte_rcvd_count;
      }
      else
      {
         error.code = kReadError;
         error.message = "I2cInterface::acquire: Invalid device handle\n";
      }
   }
   else
   {
      error.code = kInvalidDeviceHandle;
      error.message = "I2cInterface::acquire: Invalid device handle\n";
   }

   return error;
}

//-----------------------------------------------------------------------------
I2cInterface::i2c_error I2cInterface::read(
   uint8_t  address,
   void*    buffer,
   size_t   max_bytes,
   size_t&  bytes_recvd
)
{
   bytes_recvd = 0;

   i2c_error error = I2cInterface::write( &address, sizeof(address) );

   if ( error == I2cInterface::kSuccess )
   {
      error = I2cInterface::read( buffer, max_bytes, bytes_recvd );
   }

   return error;
}

//-----------------------------------------------------------------------------
I2cInterface::i2c_error I2cInterface::write(
   const void*    buffer,
   size_t         buffer_size )
{
   i2c_error error;

   if ( is_open() )
   {
      // log::mem_dump("I2cInterface::write: ", (const char*)buffer, buffer_size );
      int bytes_written = ::write( handle_, buffer, buffer_size );

      if ( bytes_written == buffer_size )
      {
         status = kSuccess;
      }
      else
      {
         std::stringstream message;
         message << "I2cInterface::write: Error(ret="
                 << bytes_written << ") - "
                 << strerror(errno) << std::endl;

         error.code = kWriteError;
         error.message = message.str();
      }
   }
   else
   {
      error.code = kInvalidDeviceHandle;
      error.message = "I2cInterface::write: Invalid device handle\n";
   }

   return error;
}

//-----------------------------------------------------------------------------
I2cInterface::i2c_error I2cInterface::write(
   uint8_t        address,
   const void*    buffer,
   size_t         buffer_size )
{
   i2c_error error;

   // Create a number buffer that contains the register address and the user-
   // supplied buffer. The data must be sent as one buffer that a repeated
   // start is performed rather than a stop.
   size_t temp_buffer_size = sizeof( address ) + buffer_size;
   uint8_t* temp_buffer = new uint8_t[ temp_buffer_size ];

   memcpy( temp_buffer, &address, sizeof(address) );
   memcpy( temp_buffer + sizeof(address), buffer, buffer_size );

   error = I2cInterface::write( temp_buffer, temp_buffer_size );

   delete[] temp_buffer;

   return error;
}
