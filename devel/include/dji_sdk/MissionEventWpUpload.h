// Generated by gencpp from file dji_sdk/MissionEventWpUpload.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONEVENTWPUPLOAD_H
#define DJI_SDK_MESSAGE_MISSIONEVENTWPUPLOAD_H


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
struct MissionEventWpUpload_
{
  typedef MissionEventWpUpload_<ContainerAllocator> Type;

  MissionEventWpUpload_()
    : incident_type(0)
    , mission_valid(0)
    , estimated_runtime(0)  {
    }
  MissionEventWpUpload_(const ContainerAllocator& _alloc)
    : incident_type(0)
    , mission_valid(0)
    , estimated_runtime(0)  {
  (void)_alloc;
    }



   typedef uint8_t _incident_type_type;
  _incident_type_type incident_type;

   typedef uint8_t _mission_valid_type;
  _mission_valid_type mission_valid;

   typedef uint16_t _estimated_runtime_type;
  _estimated_runtime_type estimated_runtime;




  typedef boost::shared_ptr< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> const> ConstPtr;

}; // struct MissionEventWpUpload_

typedef ::dji_sdk::MissionEventWpUpload_<std::allocator<void> > MissionEventWpUpload;

typedef boost::shared_ptr< ::dji_sdk::MissionEventWpUpload > MissionEventWpUploadPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionEventWpUpload const> MissionEventWpUploadConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg', '/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8bbd22d7335594c91d2500ae8d41ab36";
  }

  static const char* value(const ::dji_sdk::MissionEventWpUpload_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8bbd22d7335594c9ULL;
  static const uint64_t static_value2 = 0x1d2500ae8d41ab36ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionEventWpUpload";
  }

  static const char* value(const ::dji_sdk::MissionEventWpUpload_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 incident_type\n\
uint8 mission_valid\n\
uint16 estimated_runtime\n\
";
  }

  static const char* value(const ::dji_sdk::MissionEventWpUpload_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.incident_type);
      stream.next(m.mission_valid);
      stream.next(m.estimated_runtime);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MissionEventWpUpload_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionEventWpUpload_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionEventWpUpload_<ContainerAllocator>& v)
  {
    s << indent << "incident_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.incident_type);
    s << indent << "mission_valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mission_valid);
    s << indent << "estimated_runtime: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.estimated_runtime);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONEVENTWPUPLOAD_H
