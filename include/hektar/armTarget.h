#ifndef _ROS_hektar_armTarget_h
#define _ROS_hektar_armTarget_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class armTarget : public ros::Msg
  {
    public:
      typedef int16_t _theta_type;
      _theta_type theta;
      typedef int16_t _r_type;
      _r_type r;
      typedef int16_t _z_type;
      _z_type z;

    armTarget():
      theta(0),
      r(0),
      z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        int16_t real;
        uint16_t base;
      } u_r;
      u_r.real = this->r;
      *(outbuffer + offset + 0) = (u_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->r);
      union {
        int16_t real;
        uint16_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        int16_t real;
        uint16_t base;
      } u_r;
      u_r.base = 0;
      u_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->r = u_r.real;
      offset += sizeof(this->r);
      union {
        int16_t real;
        uint16_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->z = u_z.real;
      offset += sizeof(this->z);
     return offset;
    }

    const char * getType(){ return "hektar/armTarget"; };
    const char * getMD5(){ return "ab77415dcc390997814ebb4c3c00b756"; };

  };

}
#endif