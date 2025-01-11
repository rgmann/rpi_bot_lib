#include <vector>
#include "Log.h"
#include "i2c_interface.h"
#include "adxl345_controller.h"

/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID			0x00		// Device ID
#define ADXL345_RESERVED1		0x01		// Reserved. Do Not Access. 
#define ADXL345_THRESH_TAP		0x1D		// Tap Threshold. 
#define ADXL345_OFSX			0x1E		// X-Axis Offset. 
#define ADXL345_OFSY			0x1F		// Y-Axis Offset.
#define ADXL345_OFSZ			0x20		// Z- Axis Offset.
#define ADXL345_DUR				0x21		// Tap Duration.
#define ADXL345_LATENT			0x22		// Tap Latency.
#define ADXL345_WINDOW			0x23		// Tap Window.
#define ADXL345_THRESH_ACT		0x24		// Activity Threshold
#define ADXL345_THRESH_INACT	0x25		// Inactivity Threshold
#define ADXL345_TIME_INACT		0x26		// Inactivity Time
#define ADXL345_ACT_INACT_CTL	0x27		// Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF		0x28		// Free-Fall Threshold.
#define ADXL345_TIME_FF			0x29		// Free-Fall Time.
#define ADXL345_TAP_AXES		0x2A		// Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS	0x2B		// Source of Tap/Double Tap
#define ADXL345_BW_RATE			0x2C		// Data Rate and Power mode Control
#define ADXL345_POWER_CTL		0x2D		// Power-Saving Features Control
#define ADXL345_INT_ENABLE		0x2E		// Interrupt Enable Control
#define ADXL345_INT_MAP			0x2F		// Interrupt Mapping Control
#define ADXL345_INT_SOURCE		0x30		// Source of Interrupts
#define ADXL345_DATA_FORMAT		0x31		// Data Format Control
#define ADXL345_DATAX0			0x32		// X-Axis Data 0
#define ADXL345_DATAX1			0x33		// X-Axis Data 1
#define ADXL345_DATAY0			0x34		// Y-Axis Data 0
#define ADXL345_DATAY1			0x35		// Y-Axis Data 1
#define ADXL345_DATAZ0			0x36		// Z-Axis Data 0
#define ADXL345_DATAZ1			0x37		// Z-Axis Data 1
#define ADXL345_FIFO_CTL		0x38		// FIFO Control
#define ADXL345_FIFO_STATUS		0x39		// FIFO Status

#define ADXL345_BW_1600			0xF			// 1111		IDD = 40uA
#define ADXL345_BW_800			0xE			// 1110		IDD = 90uA
#define ADXL345_BW_400			0xD			// 1101		IDD = 140uA
#define ADXL345_BW_200			0xC			// 1100		IDD = 140uA
#define ADXL345_BW_100			0xB			// 1011		IDD = 140uA 
#define ADXL345_BW_50			0xA			// 1010		IDD = 140uA
#define ADXL345_BW_25			0x9			// 1001		IDD = 90uA
#define ADXL345_BW_12_5		    0x8			// 1000		IDD = 60uA 
#define ADXL345_BW_6_25			0x7			// 0111		IDD = 50uA
#define ADXL345_BW_3_13			0x6			// 0110		IDD = 45uA
#define ADXL345_BW_1_56			0x5			// 0101		IDD = 40uA
#define ADXL345_BW_0_78			0x4			// 0100		IDD = 34uA
#define ADXL345_BW_0_39			0x3			// 0011		IDD = 23uA
#define ADXL345_BW_0_20			0x2			// 0010		IDD = 23uA
#define ADXL345_BW_0_10			0x1			// 0001		IDD = 23uA
#define ADXL345_BW_0_05			0x0			// 0000		IDD = 23uA


 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN		0x00		//INT1: 0
#define ADXL345_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT		0x07
#define ADXL345_INT_SINGLE_TAP_BIT		0x06
#define ADXL345_INT_DOUBLE_TAP_BIT		0x05
#define ADXL345_INT_ACTIVITY_BIT		0x04
#define ADXL345_INT_INACTIVITY_BIT		0x03
#define ADXL345_INT_FREE_FALL_BIT		0x02
#define ADXL345_INT_WATERMARK_BIT		0x01
#define ADXL345_INT_OVERRUNY_BIT		0x00

#define ADXL345_DATA_READY				0x07
#define ADXL345_SINGLE_TAP				0x06
#define ADXL345_DOUBLE_TAP				0x05
#define ADXL345_ACTIVITY				0x04
#define ADXL345_INACTIVITY				0x03
#define ADXL345_FREE_FALL				0x02
#define ADXL345_WATERMARK				0x01
#define ADXL345_OVERRUNY				0x00


 /****************************** ERRORS ******************************/
#define ADXL345_OK			1		// No Error
#define ADXL345_ERROR		0		// Error Exists

#define ADXL345_NO_ERROR	0		// Initial State
#define ADXL345_READ_ERROR	1		// Accelerometer Reading Error
#define ADXL345_BAD_ARG		2		// Bad Argument

using namespace rpi_bot_lib;

//-----------------------------------------------------------------------------
Adxl345Controller::Adxl345Controller(I2cInterface& i2c, uint16_t address)
   : i2c_( i2c )
   , initialized_( false )
   , address_( address )
   , last_error_( 0 )
{
   gains_[Adxl345Controller::XAxis] = 0.00376390;
   gains_[Adxl345Controller::YAxis] = 0.00376009;
   gains_[Adxl345Controller::ZAxis] = 0.00349265;
}

//-----------------------------------------------------------------------------
bool Adxl345Controller::initialize()
{
   bool success = false;
   if ( i2c_.acquire( address_ ) == I2cInterface::kSuccess )
   {
      success = true;

      // Wakeup
      uint8_t power_ctrl = 0;
      if (i2c_.write(ADXL345_POWER_CTL, &power_ctrl, sizeof(power_ctrl)) != I2cInterface::kSuccess)
      {
         success = false;
         coral::log::error("Adxl345Controller::initialize: Failed attempting to wakeup.\n");
      }

      // Auto_Sleep
      power_ctrl = 0x10;
      if (success && i2c_.write(ADXL345_POWER_CTL, &power_ctrl, sizeof(power_ctrl)) != I2cInterface::kSuccess)
      {
         success = false;
         coral::log::error("Adxl345Controller::initialize: Failed attempting to set Auto_Sleep.\n");
      }

      // Measure
      power_ctrl = 0x08;
      if (success && i2c_.write(ADXL345_POWER_CTL, &power_ctrl, sizeof(power_ctrl)) != I2cInterface::kSuccess)
      {
         success = false;
         coral::log::error("Adxl345Controller::initialize: Failed attempting to set Measure.\n");
      }

      initialized_ = success;
   }
   else
   {
      coral::log::error("Adxl345Controller::read_acceleration_data: Failed to acquire bus.\n");
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Adxl345Controller::set_range_setting(RangeSetting setting)
{
   uint8_t temp = 0;
   uint8_t new_setting = 0;
   bool success = false;

   if ( setting == Adxl345Controller::kRange2g )
   {
      new_setting = 0x00;
   }
   else if ( setting == Adxl345Controller::kRange4g )
   {
      new_setting = 0x01;
   }
   else if ( setting == Adxl345Controller::kRange8g )
   {
      new_setting = 0x02;
   }
   else if ( setting == Adxl345Controller::kRange16g )
   {
      new_setting = 0x03;
   }
   else
   {
      coral::log::error("Adxl345Controller::set_range_setting: Invalid setting.\n");
      return false;
   }

   if ( i2c_.acquire( address_ ) == I2cInterface::kSuccess )
   {
      size_t bytes_rcvd = 0;
      success = true;

      if ( i2c_.read(ADXL345_DATA_FORMAT, &temp, sizeof(temp), bytes_rcvd) != I2cInterface::kSuccess )
      {
         coral::log::error("Adxl345Controller::set_range_setting: Error reading ADXL345_DATA_FORMAT setting.\n");
         success = false;
      }
      new_setting |= (temp & 0xEC);

      if ( success && i2c_.write(ADXL345_DATA_FORMAT, &new_setting, sizeof(new_setting))  != I2cInterface::kSuccess )
      {
         coral::log::error("Adxl345Controller::set_range_setting: Error writing ADXL345_DATA_FORMAT setting.\n");
         success = false;
      }
   }
   else
   {
      coral::log::error("Adxl345Controller::read_acceleration_data: Failed to acquire bus.\n");
   }

   return success;
}

//-----------------------------------------------------------------------------
bool Adxl345Controller::read_acceleration_data(AccelerationData& data)
{
   bool success = false;

   if ( !initialized_ )
   {
      coral::log::error("Adxl345Controller::read_acceleration_data: Controller must be initialized.\n");
      return false;
   }

   if ( i2c_.acquire( address_ ) == I2cInterface::kSuccess )
   {
      static constexpr uint32_t kBufferSize = 6;
      std::vector<uint8_t> buffer(kBufferSize, static_cast<uint8_t>(0));

      size_t bytes_received = 0;
      if ( i2c_.read(ADXL345_DATAX0, &buffer[0], buffer.size(), bytes_received) == I2cInterface::kSuccess)
      {
         if ( bytes_received == kBufferSize )
         {
            data.x = (int16_t)((((int)buffer[1]) << 8) | buffer[0]);
            data.y = (int16_t)((((int)buffer[3]) << 8) | buffer[2]);
            data.z = (int16_t)((((int)buffer[5]) << 8) | buffer[4]);

            success = true;
         }
         else
         {
            coral::log::error("Adxl345Controller::read_acceleration_data: Received less than expected data.\n");
         }
      } 
      else
      {
         coral::log::error("Adxl345Controller::read_acceleration_data: Failed attempting to read acceleration data.\n");
      }
   }
   else
   {
      coral::log::error("Adxl345Controller::read_acceleration_data: Failed to acquire bus.\n");
   }

   return success;
}
