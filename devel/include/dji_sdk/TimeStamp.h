// Generated by gencpp from file dji_sdk/TimeStamp.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_TIMESTAMP_H
#define DJI_SDK_MESSAGE_TIMESTAMP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace dji_sdk
{
template <class ContainerAllocator>
struct TimeStamp_
{
  typedef TimeStamp_<ContainerAllocator> Type;

  TimeStamp_()
    : header()
    , time(0)
    , time_ns(0)
    , sync_flag(0)  {
    }
  TimeStamp_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(0)
    , time_ns(0)
    , sync_flag(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _time_type;
  _time_type time;

   typedef uint32_t _time_ns_type;
  _time_ns_type time_ns;

   typedef uint8_t _sync_flag_type;
  _sync_flag_type sync_flag;




  typedef boost::shared_ptr< ::dji_sdk::TimeStamp_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::TimeStamp_<ContainerAllocator> const> ConstPtr;

}; // struct TimeStamp_

typedef ::dji_sdk::TimeStamp_<std::allocator<void> > TimeStamp;

typedef boost::shared_ptr< ::dji_sdk::TimeStamp > TimeStampPtr;
typedef boost::shared_ptr< ::dji_sdk::TimeStamp const> TimeStampConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::TimeStamp_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::TimeStamp_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/root/Documents/roswork/DJI2016_Challenge/src/dji_sdk/msg', '/root/Documents/roswork/DJI2016_Challenge/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::TimeStamp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::TimeStamp_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::TimeStamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::TimeStamp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::TimeStamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::TimeStamp_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::TimeStamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cdf94dfbb71b3e4aec0ba55884cec090";
  }

  static const char* value(const ::dji_sdk::TimeStamp_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcdf94dfbb71b3e4aULL;
  static const uint64_t static_value2 = 0xec0ba55884cec090ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::TimeStamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/TimeStamp";
  }

  static const char* value(const ::dji_sdk::TimeStamp_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::TimeStamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
uint32 time\n\
uint32 time_ns\n\
uint8 sync_flag\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::dji_sdk::TimeStamp_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::TimeStamp_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
      stream.next(m.time_ns);
      stream.next(m.sync_flag);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct TimeStamp_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::TimeStamp_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::TimeStamp_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time);
    s << indent << "time_ns: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time_ns);
    s << indent << "sync_flag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sync_flag);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_TIMESTAMP_H
