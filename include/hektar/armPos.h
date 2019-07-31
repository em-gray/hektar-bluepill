#ifndef _ROS_hektar_armPos_h
#define _ROS_hektar_armPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class armPos : public ros::Msg
  {
    public:
      typedef uint16_t _elbowPos_type;
      _elbowPos_type elbowPos;
      typedef uint16_t _shoulderPos_type;
      _shoulderPos_type shoulderPos;
      typedef uint16_t _basePos_type;
      _basePos_type basePos;

    armPos():
      elbowPos(0),
      shoulderPos(0),
      basePos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->elbowPos >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->elbowPos >> (8 * 1)) & 0xFF;
      offset += sizeof(this->elbowPos);
      *(outbuffer + offset + 0) = (this->shoulderPos >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->shoulderPos >> (8 * 1)) & 0xFF;
      offset += sizeof(this->shoulderPos);
      *(outbuffer + offset + 0) = (this->basePos >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->basePos >> (8 * 1)) & 0xFF;
      offset += sizeof(this->basePos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->elbowPos =  ((uint16_t) (*(inbuffer + offset)));
      this->elbowPos |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->elbowPos);
      this->shoulderPos =  ((uint16_t) (*(inbuffer + offset)));
      this->shoulderPos |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->shoulderPos);
      this->basePos =  ((uint16_t) (*(inbuffer + offset)));
      this->basePos |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->basePos);
     return offset;
    }

    const char * getType(){ return "hektar/armPos"; };
    const char * getMD5(){ return "31d223e37bebb09ea2a79675a20ee96d"; };

  };

}
#endif