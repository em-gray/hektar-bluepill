#ifndef _ROS_hektar_armCtrl_h
#define _ROS_hektar_armCtrl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class armCtrl : public ros::Msg
  {
    public:
      typedef int8_t _elbowVel_type;
      _elbowVel_type elbowVel;
      typedef int8_t _shoulderVel_type;
      _shoulderVel_type shoulderVel;
      typedef int8_t _baseVel_type;
      _baseVel_type baseVel;

    armCtrl():
      elbowVel(0),
      shoulderVel(0),
      baseVel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_elbowVel;
      u_elbowVel.real = this->elbowVel;
      *(outbuffer + offset + 0) = (u_elbowVel.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->elbowVel);
      union {
        int8_t real;
        uint8_t base;
      } u_shoulderVel;
      u_shoulderVel.real = this->shoulderVel;
      *(outbuffer + offset + 0) = (u_shoulderVel.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->shoulderVel);
      union {
        int8_t real;
        uint8_t base;
      } u_baseVel;
      u_baseVel.real = this->baseVel;
      *(outbuffer + offset + 0) = (u_baseVel.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->baseVel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_elbowVel;
      u_elbowVel.base = 0;
      u_elbowVel.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->elbowVel = u_elbowVel.real;
      offset += sizeof(this->elbowVel);
      union {
        int8_t real;
        uint8_t base;
      } u_shoulderVel;
      u_shoulderVel.base = 0;
      u_shoulderVel.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->shoulderVel = u_shoulderVel.real;
      offset += sizeof(this->shoulderVel);
      union {
        int8_t real;
        uint8_t base;
      } u_baseVel;
      u_baseVel.base = 0;
      u_baseVel.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->baseVel = u_baseVel.real;
      offset += sizeof(this->baseVel);
     return offset;
    }

    const char * getType(){ return "hektar/armCtrl"; };
    const char * getMD5(){ return "6b577512aa234a678766e6bdcc5b164b"; };

  };

}
#endif