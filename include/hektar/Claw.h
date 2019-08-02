#ifndef _ROS_hektar_Claw_h
#define _ROS_hektar_Claw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class Claw : public ros::Msg
  {
    public:
      typedef uint16_t _posL_type;
      _posL_type posL;
      typedef uint16_t _posR_type;
      _posR_type posR;

    Claw():
      posL(0),
      posR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->posL >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->posL >> (8 * 1)) & 0xFF;
      offset += sizeof(this->posL);
      *(outbuffer + offset + 0) = (this->posR >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->posR >> (8 * 1)) & 0xFF;
      offset += sizeof(this->posR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->posL =  ((uint16_t) (*(inbuffer + offset)));
      this->posL |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->posL);
      this->posR =  ((uint16_t) (*(inbuffer + offset)));
      this->posR |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->posR);
     return offset;
    }

    const char * getType(){ return "hektar/Claw"; };
    const char * getMD5(){ return "7c3e67a9e385e42dff347f06d2d88ca2"; };

  };

}
#endif