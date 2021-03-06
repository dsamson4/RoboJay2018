#ifndef _ROS_mrpt_msgs_ObservationRangeBeacon_h
#define _ROS_mrpt_msgs_ObservationRangeBeacon_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "mrpt_msgs/SingleRangeBeaconObservation.h"

namespace mrpt_msgs
{

  class ObservationRangeBeacon : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose sensor_pose_on_robot;
      float min_sensor_distance;
      float max_sensor_distance;
      float sensor_std_range;
      uint32_t sensed_data_length;
      mrpt_msgs::SingleRangeBeaconObservation st_sensed_data;
      mrpt_msgs::SingleRangeBeaconObservation * sensed_data;

    ObservationRangeBeacon():
      header(),
      sensor_pose_on_robot(),
      min_sensor_distance(0),
      max_sensor_distance(0),
      sensor_std_range(0),
      sensed_data_length(0), sensed_data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->sensor_pose_on_robot.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_sensor_distance);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_sensor_distance);
      offset += serializeAvrFloat64(outbuffer + offset, this->sensor_std_range);
      *(outbuffer + offset + 0) = (this->sensed_data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensed_data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensed_data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensed_data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensed_data_length);
      for( uint32_t i = 0; i < sensed_data_length; i++){
      offset += this->sensed_data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->sensor_pose_on_robot.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_sensor_distance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_sensor_distance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sensor_std_range));
      uint32_t sensed_data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sensed_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sensed_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sensed_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sensed_data_length);
      if(sensed_data_lengthT > sensed_data_length)
        this->sensed_data = (mrpt_msgs::SingleRangeBeaconObservation*)realloc(this->sensed_data, sensed_data_lengthT * sizeof(mrpt_msgs::SingleRangeBeaconObservation));
      sensed_data_length = sensed_data_lengthT;
      for( uint32_t i = 0; i < sensed_data_length; i++){
      offset += this->st_sensed_data.deserialize(inbuffer + offset);
        memcpy( &(this->sensed_data[i]), &(this->st_sensed_data), sizeof(mrpt_msgs::SingleRangeBeaconObservation));
      }
     return offset;
    }

    const char * getType(){ return "mrpt_msgs/ObservationRangeBeacon"; };
    const char * getMD5(){ return "94cf8974a1eec8843e3b69c87cc25d62"; };

  };

}
#endif