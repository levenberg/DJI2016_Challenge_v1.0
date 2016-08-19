// Generated by gencpp from file dji_sdk/MissionHpSetSpeedRequest.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONHPSETSPEEDREQUEST_H
#define DJI_SDK_MESSAGE_MISSIONHPSETSPEEDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct MissionHpSetSpeedRequest_
{
  typedef MissionHpSetSpeedRequest_<ContainerAllocator> Type;

  MissionHpSetSpeedRequest_()
    : speed(0.0)
    , direction(0)  {
    }
  MissionHpSetSpeedRequest_(const ContainerAllocator& _alloc)
    : speed(0.0)
    , direction(0)  {
  (void)_alloc;
    }



   typedef float _speed_type;
  _speed_type speed;

   typedef uint8_t _direction_type;
  _direction_type direction;




  typedef boost::shared_ptr< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MissionHpSetSpeedRequest_

typedef ::dji_sdk::MissionHpSetSpeedRequest_<std::allocator<void> > MissionHpSetSpeedRequest;

typedef boost::shared_ptr< ::dji_sdk::MissionHpSetSpeedRequest > MissionHpSetSpeedRequestPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionHpSetSpeedRequest const> MissionHpSetSpeedRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/root/Documents/roswork/DJI2016_Challenge_v1.0/src/dji_sdk/msg', '/root/Documents/roswork/DJI2016_Challenge_v1.0/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6ccccf547dbec8854aa1e989f62cba0e";
  }

  static const char* value(const ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6ccccf547dbec885ULL;
  static const uint64_t static_value2 = 0x4aa1e989f62cba0eULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionHpSetSpeedRequest";
  }

  static const char* value(const ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 speed\n\
uint8 direction\n\
";
  }

  static const char* value(const ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.speed);
      stream.next(m.direction);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MissionHpSetSpeedRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionHpSetSpeedRequest_<ContainerAllocator>& v)
  {
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
    s << indent << "direction: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.direction);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONHPSETSPEEDREQUEST_H
