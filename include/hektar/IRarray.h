#ifndef _ROS_hektar_IRarray_h
#define _ROS_hektar_IRarray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class IRarray : public ros::Msg
  {
    public:
      typedef int16_t _ir_0_type;
      _ir_0_type ir_0;
      typedef int16_t _ir_1_type;
      _ir_1_type ir_1;
      typedef int16_t _ir_2_type;
      _ir_2_type ir_2;
      typedef int16_t _ir_3_type;
      _ir_3_type ir_3;
      typedef int16_t _ir_4_type;
      _ir_4_type ir_4;

    IRarray():
      ir_0(0),
      ir_1(0),
      ir_2(0),
      ir_3(0),
      ir_4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ir_0;
      u_ir_0.real = this->ir_0;
      *(outbuffer + offset + 0) = (u_ir_0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_0.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ir_0);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_1;
      u_ir_1.real = this->ir_1;
      *(outbuffer + offset + 0) = (u_ir_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ir_1);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_2;
      u_ir_2.real = this->ir_2;
      *(outbuffer + offset + 0) = (u_ir_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ir_2);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_3;
      u_ir_3.real = this->ir_3;
      *(outbuffer + offset + 0) = (u_ir_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ir_3);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_4;
      u_ir_4.real = this->ir_4;
      *(outbuffer + offset + 0) = (u_ir_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ir_4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ir_0;
      u_ir_0.base = 0;
      u_ir_0.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_0.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ir_0 = u_ir_0.real;
      offset += sizeof(this->ir_0);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_1;
      u_ir_1.base = 0;
      u_ir_1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ir_1 = u_ir_1.real;
      offset += sizeof(this->ir_1);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_2;
      u_ir_2.base = 0;
      u_ir_2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ir_2 = u_ir_2.real;
      offset += sizeof(this->ir_2);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_3;
      u_ir_3.base = 0;
      u_ir_3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ir_3 = u_ir_3.real;
      offset += sizeof(this->ir_3);
      union {
        int16_t real;
        uint16_t base;
      } u_ir_4;
      u_ir_4.base = 0;
      u_ir_4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ir_4 = u_ir_4.real;
      offset += sizeof(this->ir_4);
     return offset;
    }

    const char * getType(){ return "hektar/IRarray"; };
    const char * getMD5(){ return "46ec36a9cb1faaed9c7bc44be77845e0"; };

  };

}
#endif