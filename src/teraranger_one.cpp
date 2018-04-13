#include "teraranger_one.h"

namespace teraranger
{
static volatile bool _should_run;
TerarangerOne::TerarangerOne()
{
  
  portname_ = std::string("/dev/ttyUSB0");
  _should_run = true;
  // Serial Port init
  serial_port_.setPort(portname_);
  serial_port_.setBaudrate(SERIAL_SPEED);
  serial_port_.setParity(serial::parity_none);
  serial_port_.setStopbits(serial::stopbits_one);
  serial_port_.setBytesize(serial::eightbits);
  serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
  serial_port_.setTimeout(to);

  serial_port_.open();

  // Connect serial port
  if(!serial_port_.isOpen())
  {
    ERROR("Could not open : %s ", portname_.c_str());
    return;
  }

  // Output loaded parameters to console for double checking
  // ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  // ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  // Set operation Mode
  //setMode(BINARY_MODE);
  setMode(PRECISE_MODE);
}

TerarangerOne::~TerarangerOne()
{
}
TerarangerOne::init(){
  _mavlink = new Mavlink_TCP();
  if (!_mavlink) {
    ERROR("No memory to allocate Mavlink_TCP");
    goto mavlink_memory_error;
  }
  
  if (_mavlink->init(mavlink_tcp_ip, mavlink_tcp_port)) {
    ERROR("Unable to initialize Mavlink_TCP");
    goto mavlink_init_error;
  }

  return 0;

  mavlink_init_error:
    delete;
  mavlink_memory_error:
    delete;
    return -1;

}

void TerarangerOne::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  if (single_character == 'T' && buffer_ctr == 0)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    return;
  }
  else if (buffer_ctr >= 1 && buffer_ctr < BUFFER_SIZE-1)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    return;
  }
  if (buffer_ctr == BUFFER_SIZE-1)
  {
    input_buffer[buffer_ctr] = single_character;
    uint8_t crc = HelperLib::crc8(input_buffer, BUFFER_SIZE-1);
    if(crc == input_buffer[BUFFER_SIZE-1])
    {
      int16_t range = input_buffer[1] << 8;
      range |= input_buffer[2];

      float final_range;
      float float_range = range * VALUE_TO_METER_FACTOR;

      if(range == TOO_CLOSE_VALUE)// Too close, 255 is for short range
      {
        final_range = -std::numeric_limits<float>::infinity();
      }
      else if(range == OUT_OF_RANGE_VALUE)// Out of range
      {
        final_range = std::numeric_limits<float>::infinity();
      }
      // Enforcing min and max range
      else if(float_range > range_msg.max_range)
      {
        final_range = std::numeric_limits<float>::infinity();
      }
      else if(float_range < range_msg.min_range)
      {
        final_range = -std::numeric_limits<float>::infinity();
      }
      else
      {
        final_range = float_range;
      }
      // ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_msg.range);
      //TODO: Publish mavlink here
      mavlink_distance_sensor_t msg;
      //msg.time_boot_ms
      uint16_t range;
      range = (uint16_t)(final_range*100)

      msg.min_distance = 20;
      msg.max_distance = 1400;
      msg.current_distance=final_range; /*< Current distance reading*/
      msg.type=2; /*< Type from MAV_DISTANCE_SENSOR enum.*/
      msg.id=0; /*< Onboard ID of the sensor*/
      msg.orientation=8; /*< Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.*/
      msg.covariance=7; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
      _mavlink->distance_sensor_msg_write(&msg);
    }
    else
    {
      //ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      //TODO: mavlink debug here
    }
  }
  // reset
  buffer_ctr = 0;
  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);
}

void TerarangerOne::setMode(const char *c)
{
  if(!serial_port_.write((uint8_t*)c, CMD_BYTE_LENGTH))// 1 byte commands
  {
    ERROR("Timeout or error while reading serial");
  }
  serial_port_.flushOutput();
}
void TerarangerOne::run(){
  static uint8_t buffer[1];
   while(_should_run);
   {
     if(serial_port_.read(buffer, 1))
     {
       serialDataCallback(buffer[0]);
     }
     else
     {
       ERROR("Timeout or error while reading serial");
     }
   }

}
void TerarangerOne::shutdown(){
  _should_run=false;

}

} // namespace teraranger