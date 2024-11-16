#ifndef _ROS_moveo_moveit_ArmJointState_h
#define _ROS_moveo_moveit_ArmJointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moveo_moveit
{

  class ArmJointState : public ros::Msg
  {
    public:
      typedef int16_t _position1_type;
      _position1_type position1;
      typedef int16_t _position2_type;
      _position2_type position2;
      typedef int16_t _position3_type;
      _position3_type position3;
      typedef int16_t _position4_type;
      _position4_type position4;
      typedef int16_t _position5_type;
      _position5_type position5;
      typedef int16_t _position6_type;
      _position6_type position6;

    ArmJointState():
      position1(0),
      position2(0),
      position3(0),
      position4(0),
      position5(0),
      position6(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_position1;
      u_position1.real = this->position1;
      *(outbuffer + offset + 0) = (u_position1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position1);
      union {
        int16_t real;
        uint16_t base;
      } u_position2;
      u_position2.real = this->position2;
      *(outbuffer + offset + 0) = (u_position2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position2);
      union {
        int16_t real;
        uint16_t base;
      } u_position3;
      u_position3.real = this->position3;
      *(outbuffer + offset + 0) = (u_position3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position3);
      union {
        int16_t real;
        uint16_t base;
      } u_position4;
      u_position4.real = this->position4;
      *(outbuffer + offset + 0) = (u_position4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position4);
      union {
        int16_t real;
        uint16_t base;
      } u_position5;
      u_position5.real = this->position5;
      *(outbuffer + offset + 0) = (u_position5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position5.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position5);
      union {
        int16_t real;
        uint16_t base;
      } u_position6;
      u_position6.real = this->position6;
      *(outbuffer + offset + 0) = (u_position6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position6.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_position1;
      u_position1.base = 0;
      u_position1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position1 = u_position1.real;
      offset += sizeof(this->position1);
      union {
        int16_t real;
        uint16_t base;
      } u_position2;
      u_position2.base = 0;
      u_position2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position2 = u_position2.real;
      offset += sizeof(this->position2);
      union {
        int16_t real;
        uint16_t base;
      } u_position3;
      u_position3.base = 0;
      u_position3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position3 = u_position3.real;
      offset += sizeof(this->position3);
      union {
        int16_t real;
        uint16_t base;
      } u_position4;
      u_position4.base = 0;
      u_position4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position4 = u_position4.real;
      offset += sizeof(this->position4);
      union {
        int16_t real;
        uint16_t base;
      } u_position5;
      u_position5.base = 0;
      u_position5.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position5.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position5 = u_position5.real;
      offset += sizeof(this->position5);
      union {
        int16_t real;
        uint16_t base;
      } u_position6;
      u_position6.base = 0;
      u_position6.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position6.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position6 = u_position6.real;
      offset += sizeof(this->position6);
     return offset;
    }

    virtual const char * getType() override { return "moveo_moveit/ArmJointState"; };
    virtual const char * getMD5() override { return "7bf56d1cde4c613af8c16b02c640040e"; };

  };

}
#endif
