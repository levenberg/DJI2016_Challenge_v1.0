// Generated by gencpp from file dji_sdk/MissionHpUploadRequest.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONHPUPLOADREQUEST_H
#define DJI_SDK_MESSAGE_MISSIONHPUPLOADREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dji_sdk/MissionHotpointTask.h>

namespace dji_sdk
{
template <class ContainerAllocator>
struct MissionHpUploadRequest_
{
  typedef MissionHpUploadRequest_<ContainerAllocator> Type;

  MissionHpUploadRequest_()
    : hotpoint_task()  {
    }
  MissionHpUploadRequest_(const ContainerAllocator& _alloc)
    : hotpoint_task(_alloc)  {
  (void)_alloc;
    }



   typedef  ::dji_sdk::MissionHotpointTask_<ContainerAllocator>  _hotpoint_task_type;
  _hotpoint_task_type hotpoint_task;




  typedef boost::shared_ptr< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MissionHpUploadRequest_

typedef ::dji_sdk::MissionHpUploadRequest_<std::allocator<void> > MissionHpUploadRequest;

typedef boost::shared_ptr< ::dji_sdk::MissionHpUploadRequest > MissionHpUploadRequestPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionHpUploadRequest const> MissionHpUploadRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e36e66ca170c4d03ee023ad56c6bb5a0";
  }

  static const char* value(const ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe36e66ca170c4d03ULL;
  static const uint64_t static_value2 = 0xee023ad56c6bb5a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionHpUploadRequest";
  }

  static const char* value(const ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MissionHotpointTask hotpoint_task\n\
\n\
================================================================================\n\
MSG: dji_sdk/MissionHotpointTask\n\
float64 latitude\n\
float64 longitude\n\
float64 altitude\n\
float64 radius\n\
float32 angular_speed\n\
uint8 is_clockwise\n\
uint8 start_point\n\
uint8 yaw_mode\n\
";
  }

  static const char* value(const ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.hotpoint_task);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MissionHpUploadRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionHpUploadRequest_<ContainerAllocator>& v)
  {
    s << indent << "hotpoint_task: ";
    s << std::endl;
    Printer< ::dji_sdk::MissionHotpointTask_<ContainerAllocator> >::stream(s, indent + "  ", v.hotpoint_task);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONHPUPLOADREQUEST_H
