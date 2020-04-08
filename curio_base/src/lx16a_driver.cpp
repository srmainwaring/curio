//
//  Software License Agreement (BSD-3-Clause)
//   
//  Copyright (c) 2019 Rhys Mainwaring
//  All rights reserved
//   
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1.  Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//  2.  Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//  3.  Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//

#include "curio_base/lx16a_driver.h"

#include <ros/ros.h>

#include <cstdarg>
#include <chrono>
#include <iostream>
#include <string>
#include <sstream>


// Macro function  get lower 8 bits of A
#define GET_LOW_BYTE(A) (uint8_t)((A))

// Macro function  get higher 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

// Macro Function  put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

// #define LOBOT_DEBUG 1  /*Debug ：print debug value*/

uint8_t LobotCheckSum(uint8_t buf[])
{
  uint8_t i;
  uint16_t temp = 0;
  for (uint8_t i = 2; i < buf[3] + 2; ++i)
  {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

void LobotSerialServoMove(serial::Serial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  uint8_t buf[10];
  if (position < 0)
  {
    position = 0;
  }
  if (position > 1000)
  {
    position = 1000;
  }
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}

void LobotSerialServoStopMove(serial::Serial &SerialX, uint8_t id)
{
  uint8_t buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

void LobotSerialServoAngleAdjust(serial::Serial &SerialX, uint8_t id, uint8_t deviation)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ANGLE_OFFSET_ADJUST;
  buf[5] = deviation;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
}

void LobotSerialServoSetID(serial::Serial &SerialX, uint8_t oldID, uint8_t newID)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT SERVO ID WRITE");
  std::ostringstream ss;
  for (int i = 0; i < buf[3] + 3; ++i)
  {
    ss << "0x"  << std::uppercase << std::setfill('0')
       << std::setw(2) << std::hex << int(buf[i]) << ":";
  }
  ROS_INFO_STREAM("" << ss.str());
#endif
}

void LobotSerialServoSetMode(serial::Serial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  uint8_t buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT SERVO Set Mode");
  std::ostringstream ss;
  for (int i = 0; i < buf[3] + 3; ++i)
  {
    ss << "0x"  << std::uppercase << std::setfill('0')
       << std::setw(2) << std::hex << int(buf[i]) << ":";
  }
  ROS_INFO_STREAM("" << ss.str());
#endif
  size_t cnt = SerialX.write(buf, 10);
  ROS_INFO_STREAM("wrote: " << cnt);
}

void LobotSerialServoLoad(serial::Serial &SerialX, uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT SERVO LOAD WRITE");
  std::ostringstream ss;
  for (int i = 0; i < buf[3] + 3; ++i)
  {
    ss << "0x"  << std::uppercase << std::setfill('0')
       << std::setw(2) << std::hex << int(buf[i]) << ":";
  }
  ROS_INFO_STREAM("" << ss.str());
#endif
}

void LobotSerialServoUnload(serial::Serial &SerialX, uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT SERVO LOAD WRITE");
  std::ostringstream ss;
  for (int i = 0; i < buf[3] + 3; ++i)
  {
    ss << "0x"  << std::uppercase << std::setfill('0')
       << std::setw(2) << std::hex << int(buf[i]) << ":";
  }
  ROS_INFO_STREAM("" << ss.str());
#endif
}

int LobotSerialServoReceiveHandle(serial::Serial &SerialX, uint8_t *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  uint8_t frameCount = 0;
  uint8_t dataCount = 0;
  uint8_t dataLength = 2;
  uint8_t rxBuf;
  uint8_t recvBuf[32];
  uint8_t i;

  while (SerialX.available())
  {
    uint8_t bytes_read = SerialX.read(&rxBuf, 1);


#ifdef LOBOT_DEBUG
    std::stringstream ss;
    ss << "Bytes Read: " << static_cast<int>(bytes_read)
      << ", RX: 0x" << std::hex << static_cast<int>(rxBuf);
    ROS_INFO_STREAM(ss.str());
#endif 

    // delayMicroseconds(100);
    bool is_readable = SerialX.waitReadable();
    
    if (!frameStarted)
    {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER)
      {
        frameCount++;
        if (frameCount == 2)
        {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else
      {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted)
    {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3)
      {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7)
        {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3)
      {
#ifdef LOBOT_DEBUG
        std::stringstream ss;
        ss << "RECEIVE_DATA:"; 
        for (i = 0; i < dataCount; i++)
        {
          ss << "0x"  << std::uppercase << std::setfill('0')
             << std::setw(2) << std::hex << int(recvBuf[i]) << ":";
        }
        ROS_INFO_STREAM("" << ss.str());
#endif
        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1])
        {
#ifdef LOBOT_DEBUG
          ROS_INFO("Check SUM OK!!");
          ROS_INFO("");
#endif
          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
#ifdef LOBOT_DEBUG
        ROS_INFO("LOBOT INVALID CHECKSUM");
#endif
        return -1;
      }
    }
  }
#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT NO DATA AVAILABLE");
#endif
  return -1;
}

int LobotSerialServoReadPosition(serial::Serial &SerialX, uint8_t id)
{
  int count = 100000;
  int16_t ret;
  uint8_t rxBuf;
  uint8_t buf[6];

  buf[0] = buf[1] = uint8_t(LOBOT_SERVO_FRAME_HEADER);
  buf[2] = id;
  buf[3] = 3;
  buf[4] = uint8_t(LOBOT_SERVO_POS_READ);
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT SERVO Pos READ");
  std::stringstream ss;
  for (int i = 0; i < buf[3] + 3; ++i)
  {
    ss << "0x"  << std::uppercase << std::setfill('0')
       << std::setw(2) << std::hex << int(buf[i]) << ":";
  }
  ROS_INFO_STREAM("" << ss.str());
#endif

  while (SerialX.available())
  {
    SerialX.read(&rxBuf, 1);
  }

  SerialX.write(buf, 6);

#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT WAIT FOR RESPONSE");
#endif
  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
    {
#ifdef LOBOT_DEBUG
      ROS_INFO("LOBOT NO RESPONSE");
#endif
      return -1000;
    }
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
  {
#ifdef LOBOT_DEBUG
    ROS_INFO("LOBOT GOT RESPONSE");
#endif
    ret = BYTE_TO_HW(buf[2], buf[1]);
  }
  else
  {
#ifdef LOBOT_DEBUG
    ROS_INFO("LOBOT INVALID RESPONSE");
#endif
    ret = -1000;
  }

#ifdef LOBOT_DEBUG
  ROS_INFO_STREAM("" << ret);
#endif
  return ret;
}

int LobotSerialServoReadVin(serial::Serial &SerialX, uint8_t id)
{
  int count = 100000;
  int16_t ret;
  uint8_t rxBuf;
  uint8_t buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  ROS_INFO("LOBOT SERVO VIN READ");
  std::ostringstream ss;
  for (int i = 0; i < buf[3] + 3; ++i)
  {
    ss << "0x"  << std::uppercase << std::setfill('0')
       << std::setw(2) << std::hex << int(buf[i]) << ":";
  }
  ROS_INFO_STREAM("" << ss.str());
#endif

  while (SerialX.available())
  {
    SerialX.read(&rxBuf, 1);
  }

  SerialX.write(buf, 6);

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
    {
      return -2048;
    }
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
  {
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  }
  else
  {
    ret = -2044;
  }
#ifdef LOBOT_DEBUG
  ROS_INFO_STREAM("" << ret);
#endif
  return ret;
}

namespace curio_base
{
    LX16ADriver::LX16ADriver()
    {
    }

    LX16ADriver::~LX16ADriver()
    {
        serial_.close();
    }

    void LX16ADriver::move(uint8_t id, int16_t position, uint16_t time)
    {
        LobotSerialServoMove(serial_, id, position, time);
    }

    void LX16ADriver::stopMove(uint8_t id)
    {
        LobotSerialServoStopMove(serial_, id);
    }

    void LX16ADriver::angleAdjust(uint8_t id, uint8_t deviation)
    {
        LobotSerialServoAngleAdjust(serial_, id, deviation);
    }

    void LX16ADriver::setMode(uint8_t id, uint8_t mode, int16_t duty)
    {
        LobotSerialServoSetMode(serial_, id, mode, duty);
    }

    int LX16ADriver::readPosition(uint8_t id)
    {
        return LobotSerialServoReadPosition(serial_, id);
    }

    int LX16ADriver::readVin(uint8_t id)
    {
        return LobotSerialServoReadVin(serial_, id);
    }

    // Serial
    void LX16ADriver::open()
    {
        serial_.open();
    }

    bool LX16ADriver::isOpen() const
    {
        return serial_.isOpen();
    }

    void LX16ADriver::close()
    {
        serial_.close();
    }

    void LX16ADriver::setPort(const std::string &port)
    {
        serial_.setPort(port);
    }

    std::string LX16ADriver::getPort() const
    {
        return serial_.getPort();
    }

    void LX16ADriver::setBaudrate(uint32_t baudrate)
    {
        serial_.setBaudrate(baudrate);
    }

    uint32_t LX16ADriver::getBaudrate() const
    {
        return serial_.getBaudrate();
    }

    void LX16ADriver::setTimeout(uint32_t timeout)
    {
        serial::Timeout t = serial::Timeout::simpleTimeout(timeout);
        serial_.setTimeout(t);
    }

} // namespace curio_base

