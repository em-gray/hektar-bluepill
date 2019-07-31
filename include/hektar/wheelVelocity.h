#ifndef _ROS_hektar_wheelVelocity_h
#define _ROS_hektar_wheelVelocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class wheelVelocity : public ros::Msg
  {
    public:
      typedef int8_t _wheelL_type;
      _wheelL_type wheelL;
      typedef int8_t _wheelR_type;
      _wheelR_type wheelR;

    wheelVelocity():
      wheelL(0),
      wheelR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_wheelL;
      u_wheelL.real = this->wheelL;
      *(outbuffer + offset + 0) = (u_wheelL.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheelL);
      union {
        int8_t real;
        uint8_t base;
      } u_wheelR;
      u_wheelR.real = this->wheelR;
      *(outbuffer + offset + 0) = (u_wheelR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheelR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_wheelL;
      u_wheelL.base = 0;
      u_wheelL.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->wheelL = u_wheelL.real;
      offset += sizeof(this->wheelL);
      union {
        int8_t real;
        uint8_t base;
      } u_wheelR;
      u_wheelR.base = 0;
      u_wheelR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->wheelR = u_wheelR.real;
      offset += sizeof(this->wheelR);
     return offset;
    }

    const char * getType(){ return "hektar/wheelVelocity"; };
    const char * getMD5(){ return "dfd6621a64e3179001c9140f85940a78"; };

  };

}
#endif