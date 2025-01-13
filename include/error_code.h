#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

#include <string>
#include <sstream>
#include <utility>

namespace rpi_bot_lib {

class error {
public:

   enum error_code {
      kSuccess = 0,
      kError,
      kNumError
   };

   error_code  code = kSuccess;
   std::string message;


   std::stringstream& message_begin()
   {
      message_stream_.str(std::string());
      return message_stream_;
   }

   void message_end()
   {
      message_stream_ << std::endl;
      message = message_stream_.str();
   }

   bool ok() const { return code == kSuccess; }
   operator bool() const { return ok(); }

   static error make_error(const std::string& error_meesage)
   {
      error status;
      status.code = kError;
      status.message = error_meesage;
      return status;
   }

private:

   std::stringstream message_stream_;
};

}

#endif // __ERROR_CODE_H__