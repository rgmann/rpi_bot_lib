// 
// Copyright (c) 2025, Robert Vaughan <<vaughan.t.robert@gmail.com>>
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

#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

#include <string>
#include <sstream>
#include <utility>

namespace rpi_bot_lib {

/**
 * Generic error class use by all interfaces within the library
 */
class error {
public:

   enum error_code {
      kSuccess = 0,
      kError,
      kNumError
   };

   error_code  code = kSuccess;
   std::string message;

   /**
    * Clears the stream so that a new message can be built.
    *
    * @return Reference to stream
    */
   std::stringstream& message_begin()
   {
      message_stream_.str(std::string());
      return message_stream_;
   }

   /**
    * End the message by appending endl and updating message with formatted string.
    *
    * @return void
    */
   void message_end()
   {
      message_stream_ << std::endl;
      message = message_stream_.str();
   }

   /**
    * Convenience method to check whether the status code indicates success.
    *
    * @return bool
    */
   bool ok() const { return code == kSuccess; }

   /**
    * Convenience operator to evaluate the status code as a boolean (success == true).
    *
    * @return bool
    */
   operator bool() const { return ok(); }

   /**
    * Static factory to create an error instace with the specified message.
    *
    * @param error_message Error message string
    * @return error
    */
   static error make_error(const std::string& error_meesage)
   {
      error status;
      status.code = kError;
      status.message = error_meesage;
      return status;
   }


private:

   // Stream should not be directly accessed by callers
   std::stringstream message_stream_;
};

}

#endif // __ERROR_CODE_H__