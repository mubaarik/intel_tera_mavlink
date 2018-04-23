#pragma once

//#include <ros/ros.h>
//#include <sensor_msgs/Range.h>
//#include <std_msgs/Char.h>
//#include <dynamic_reconfigure/server.h>

#include <mavlink.h>
#include "mavlink_tcp.h"
#include <string>
#include <serial/serial.h>
#include <sys/time.h>
//#include <teraranger/TerarangerOneConfig.h>
#include "log.h"
#include "helper_lib.h"
#include <limits>

#define OUT_OF_RANGE_VALUE 0
#define TOO_CLOSE_VALUE 200
#define VALUE_TO_METER_FACTOR 0.001

#define BUFFER_SIZE 4
#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000
#define CMD_BYTE_LENGTH 1
#define MAVLINK_TCP_IP "127.0.0.1"
#define MAVLINK_TCP_PORT 5760
#define MAX_RANGE 14.0
#define MIN_RANGE .2

namespace teraranger
{

static const char PRECISE_MODE[CMD_BYTE_LENGTH] = {'P'};
static const char FAST_MODE[CMD_BYTE_LENGTH] = {'F'};
static const char OUTDOOR_MODE[CMD_BYTE_LENGTH] = {'O'};

static const char BINARY_MODE[CMD_BYTE_LENGTH] = {'B'};
static const char TEXT_MODE[CMD_BYTE_LENGTH] = {'T'};

class TerarangerOne
{
public:
  TerarangerOne();
  virtual ~TerarangerOne();

  void serialDataCallback(uint8_t data);
  void timerUpdate(uint64_t time);

  //void dynParamCallback(const teraranger::TerarangerOneConfig &config, uint32_t level);

  //bool loadParameters();
  void setMode(const char *c);
  unsigned long mavlink_tcp_port = MAVLINK_TCP_PORT;
  const char *mavlink_tcp_ip = MAVLINK_TCP_IP;

  struct timeval tp;
  uint64_t _update_time=0;
  uint64_t _current_time=0;
  uint64_t _offset_timestamp_msec=0;


  Mavlink_TCP *_mavlink;

  serial::Serial serial_port_;
  //boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;
  //std::string frame_id_;
  int init();
  void run();
  void shutdown();
};

} // namespace teraranger
