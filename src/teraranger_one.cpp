#include "teraranger_one.h"

namespace teraranger
{
bool _should_run;
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
int TerarangerOne::init(){
  _mavlink = new Mavlink_TCP();
  //DEBUG("Initialing mavlink");
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
    delete _mavlink;
    return -1;
  mavlink_memory_error:
    delete _mavlink;
    return -1;

}
static void _highres_imu_msg_callback(const mavlink_highres_imu_t *msg, void *data)
{
  TerarangerOne * tera =  (TerarangerOne *)(data);
  tera->timerUpdate(msg->time_usec);
}
void TerarangerOne::timerUpdate(uint64_t time){
  gettimeofday(&tp, NULL);
  _update_time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  _offset_timestamp_msec = time/1000;


}


void TerarangerOne::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  //DEBUG("Running the callback");

  if (single_character == 'T' && buffer_ctr == 0)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    return;
  }
  else if (buffer_ctr >= 1 && buffer_ctr < BUFFER_SIZE-1)
  {
    //DEBUG("Filling the buffer");
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    return;
  }
  if (buffer_ctr == BUFFER_SIZE-1)
  {
    //DEBUG("Filling the buffer");
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
      else if(float_range > MAX_RANGE)
      {
        final_range = std::numeric_limits<float>::infinity();
      }
      else if(float_range < MIN_RANGE)
      {
        final_range = -std::numeric_limits<float>::infinity();
      }
      else
      {
        final_range = float_range;
      }
      // ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_msg.range);
      //TODO: Publish mavlink here
      gettimeofday(&tp, NULL);
      _current_time = tp.tv_sec * 1000 + tp.tv_usec / 1000;



      mavlink_distance_sensor_t msg;
      //msg.time_boot_ms
      uint16_t _range;
      _range = (uint16_t)(final_range*100);
      msg.time_boot_ms = (uint32_t)(_offset_timestamp_msec+(_current_time-_update_time));
      msg.min_distance = 10;
      msg.max_distance = 1400;
      msg.current_distance=_range; /*< Current distance reading*/
      msg.type=2; /*< Type from MAV_DISTANCE_SENSOR enum.*/
      msg.id=0; /*< Onboard ID of the sensor*/
      msg.orientation=25; /*< Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.*/
      msg.covariance=7; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
      DEBUG("Range to be sent is: %u", _range);
      int status = _mavlink->distance_sensor_msg_write(&msg);
      if(status==-1){
	_mavlink->init(mavlink_tcp_ip, mavlink_tcp_port);
        ERROR("got a broken pipe! Reinitializing mavlink");
      }

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
  DEBUG("finished setting the MODE(defualt: PRECISE_MODE)");
  serial_port_.flushOutput();
}
void TerarangerOne::run(){
  static uint8_t buffer[1];
  _mavlink->highres_imu_msg_subscribe(_highres_imu_msg_callback,this);
  //DEBUG("LOOP STARTED!");
   while(_should_run)
   {
      //DEBUG("RUNNING");
     if(serial_port_.read(buffer, 1))
     {
      //DEBUG("ENTERING THE CALLBACK!");
       serialDataCallback(buffer[0]);
     }
     else
     {
       ERROR("Timeout or error while reading serial");
       //DEBUG("Timeout or error while reading serial");
     }
     
   }
   //DEBUG("ENDED STARTED!");

}
void TerarangerOne::shutdown(){
  _should_run=false;

}

} // namespace teraranger
