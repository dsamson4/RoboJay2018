#ifndef _ROS_mrpt_msgs_SingleRangeBearingObservation_h
#define _ROS_mrpt_msgs_SingleRangeBearingObservation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mrpt_msgs
{

  class SingleRangeBearingObservation : public ros::Msg
  {
    public:
      float range;
      float yaw;
      float pitch;
      int32_t id;
      enum { INVALID_LANDMARK_ID =  -1 };

    SingleRangeBearingObservation():
      range(0),
      yaw(0),
      pitch(0),
      id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->range);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->range));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
     return offset;
    }

    const char * getType(){ return "mrpt_msgs/SingleRangeBearingObservation"; };
    const char * getMD5(){ return "f87f32b82ea55d6b1af49bd000fbcdb7"; };

  };

}
#endif