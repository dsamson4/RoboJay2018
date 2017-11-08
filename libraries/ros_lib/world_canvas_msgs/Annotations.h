#ifndef _ROS_world_canvas_msgs_Annotations_h
#define _ROS_world_canvas_msgs_Annotations_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "world_canvas_msgs/Annotation.h"

namespace world_canvas_msgs
{

  class Annotations : public ros::Msg
  {
    public:
      const char* map_id;
      uint32_t annotations_length;
      world_canvas_msgs::Annotation st_annotations;
      world_canvas_msgs::Annotation * annotations;

    Annotations():
      map_id(""),
      annotations_length(0), annotations(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_map_id = strlen(this->map_id);
      memcpy(outbuffer + offset, &length_map_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->map_id, length_map_id);
      offset += length_map_id;
      *(outbuffer + offset + 0) = (this->annotations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->annotations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->annotations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->annotations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->annotations_length);
      for( uint32_t i = 0; i < annotations_length; i++){
      offset += this->annotations[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_map_id;
      memcpy(&length_map_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_map_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_map_id-1]=0;
      this->map_id = (char *)(inbuffer + offset-1);
      offset += length_map_id;
      uint32_t annotations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      annotations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      annotations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      annotations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->annotations_length);
      if(annotations_lengthT > annotations_length)
        this->annotations = (world_canvas_msgs::Annotation*)realloc(this->annotations, annotations_lengthT * sizeof(world_canvas_msgs::Annotation));
      annotations_length = annotations_lengthT;
      for( uint32_t i = 0; i < annotations_length; i++){
      offset += this->st_annotations.deserialize(inbuffer + offset);
        memcpy( &(this->annotations[i]), &(this->st_annotations), sizeof(world_canvas_msgs::Annotation));
      }
     return offset;
    }

    const char * getType(){ return "world_canvas_msgs/Annotations"; };
    const char * getMD5(){ return "5b47cb1c51cbc7a899bf81b52bdb8c27"; };

  };

}
#endif