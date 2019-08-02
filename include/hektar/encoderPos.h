#ifndef _ROS_hektar_encoderPos_h
#define _ROS_hektar_encoderPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class encoderPos : public ros::Msg
  {
    public:
      typedef int32_t _wheelL_type;
      _wheelL_type wheelL;
      typedef int32_t _wheelR_type;
      _wheelR_type wheelR;

    encoderPos():
      wheelL(0),
      wheelR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_wheelL;
      u_wheelL.real = this->wheelL;
      *(outbuffer + offset + 0) = (u_wheelL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheelL.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheelL.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheelL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheelL);
      union {
        int32_t real;
        uint32_t base;
      } u_wheelR;
      u_wheelR.real = this->wheelR;
      *(outbuffer + offset + 0) = (u_wheelR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheelR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheelR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheelR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheelR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_wheelL;
      u_wheelL.base = 0;
      u_wheelL.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheelL.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheelL.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheelL.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheelL = u_wheelL.real;
      offset += sizeof(this->wheelL);
      union {
        int32_t real;
        uint32_t base;
      } u_wheelR;
      u_wheelR.base = 0;
      u_wheelR.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheelR.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheelR.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheelR.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheelR = u_wheelR.real;
      offset += sizeof(this->wheelR);
     return offset;
    }

    const char * getType(){ return "hektar/encoderPos"; };
    const char * getMD5(){ return "ef1c8bc5f8d177a857d6d8c7c645815c"; };

  };

}
#endif