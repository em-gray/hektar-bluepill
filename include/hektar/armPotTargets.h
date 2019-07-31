#ifndef _ROS_hektar_armPotTargets_h
#define _ROS_hektar_armPotTargets_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hektar
{

  class armPotTargets : public ros::Msg
  {
    public:
      typedef uint16_t _shoulderVal_type;
      _shoulderVal_type shoulderVal;
      typedef uint16_t _elbowVal_type;
      _elbowVal_type elbowVal;
      typedef uint16_t _baseVal_type;
      _baseVal_type baseVal;

    armPotTargets():
      shoulderVal(0),
      elbowVal(0),
      baseVal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->shoulderVal >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->shoulderVal >> (8 * 1)) & 0xFF;
      offset += sizeof(this->shoulderVal);
      *(outbuffer + offset + 0) = (this->elbowVal >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->elbowVal >> (8 * 1)) & 0xFF;
      offset += sizeof(this->elbowVal);
      *(outbuffer + offset + 0) = (this->baseVal >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->baseVal >> (8 * 1)) & 0xFF;
      offset += sizeof(this->baseVal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->shoulderVal =  ((uint16_t) (*(inbuffer + offset)));
      this->shoulderVal |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->shoulderVal);
      this->elbowVal =  ((uint16_t) (*(inbuffer + offset)));
      this->elbowVal |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->elbowVal);
      this->baseVal =  ((uint16_t) (*(inbuffer + offset)));
      this->baseVal |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->baseVal);
     return offset;
    }

    const char * getType(){ return "hektar/armPotTargets"; };
    const char * getMD5(){ return "fa0bebe251fc669ce73cd8cf7425f87d"; };

  };

}
#endif