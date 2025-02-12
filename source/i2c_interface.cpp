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
#include <limits>

#include "i2c_interface.h"

using namespace rpi_bot_lib;

static const std::string kInvalidDeviceHandleMessage = "Invalid device handle";

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
error I2cInterface::open( const char* device_path )
{
   error status;

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
         status.code = error::kError;
         status.message_begin() << "Failed to open device - "
                                << strerror(errno);
         status.message_end();
      }
   }

   return status;
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
error I2cInterface::acquire( uint16_t device_id )
{
   error status;

   if ( is_open() )
   {
      uint32_t device_id_ul = static_cast<uint32_t>(device_id);

      if ( ( device_id_ul & 0xFFFFFF00 ) &&
           ( ( capabilities_ & I2C_FUNC_10BIT_ADDR ) == 0 ) )
      {
         status = error::make_error("10-bit addresses not supported");
      }
      else if ( ioctl( handle_, I2C_SLAVE, device_id_ul ) < 0 )
      {
         status = error::make_error("Invalid device ID");
      }
   }
   else
   {
      status = error::make_error(kInvalidDeviceHandleMessage);
   }

   return status;
}

//-----------------------------------------------------------------------------
error I2cInterface::read(
   void*    buffer,
   size_t   max_bytes,
   size_t&  bytes_recvd
)
{
   error status;

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
         status = error::make_error(kInvalidDeviceHandleMessage);
      }
   }
   else
   {
      status = error::make_error(kInvalidDeviceHandleMessage);
   }

   return status;
}

//-----------------------------------------------------------------------------
error I2cInterface::read(
   uint8_t  address,
   void*    buffer,
   size_t   max_bytes,
   size_t&  bytes_recvd
)
{
   bytes_recvd = 0;

   error status = I2cInterface::write( &address, sizeof(address) );

   if ( status )
   {
      status = I2cInterface::read( buffer, max_bytes, bytes_recvd );
   }

   return status;
}

//-----------------------------------------------------------------------------
error I2cInterface::write(
   const void*    buffer,
   size_t         buffer_size )
{
   error status;

   if ( is_open() && (buffer_size <= static_cast<size_t>(std::numeric_limits<int>::max())))
   {
      int bytes_written = ::write( handle_, buffer, buffer_size );

      if ( bytes_written != static_cast<int>(buffer_size) )
      {
         status.code = error::kError;
         status.message_begin() << "Error(ret="
                 << bytes_written << ") - "
                 << strerror(errno);
         status.message_end();
      }
   }
   else
   {
      status = error::make_error(kInvalidDeviceHandleMessage);
   }

   return status;
}

//-----------------------------------------------------------------------------
error I2cInterface::write(
   uint8_t        address,
   const void*    buffer,
   size_t         buffer_size )
{
   error status;

   // Create a number buffer that contains the register address and the user-
   // supplied buffer. The data must be sent as one buffer that a repeated
   // start is performed rather than a stop.
   size_t temp_buffer_size = sizeof( address ) + buffer_size;
   uint8_t* temp_buffer = new uint8_t[ temp_buffer_size ];

   memcpy( temp_buffer, &address, sizeof(address) );
   memcpy( temp_buffer + sizeof(address), buffer, buffer_size );

   status = I2cInterface::write( temp_buffer, temp_buffer_size );

   delete[] temp_buffer;

   return status;
}
