#ifndef _ROS_checkpoint_3_encoderData_h
#define _ROS_checkpoint_3_encoderData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace checkpoint_3
{

  class encoderData : public ros::Msg
  {
    public:
      typedef float _left_speed_type;
      _left_speed_type left_speed;
      typedef float _right_speed_type;
      _right_speed_type right_speed;

    encoderData():
      left_speed(0),
      right_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->left_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_speed));
     return offset;
    }

    const char * getType(){ return "checkpoint_3/encoderData"; };
    const char * getMD5(){ return "7bc1d5de217d0bb7fcf88504a02ab155"; };

  };

}
#endif
