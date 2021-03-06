/* Auto-generated by genmsg_cpp for file /home/rss-student/rss14/oldStuff/lab5_msgs/msg/ColorMsg.msg */
#ifndef LAB5_MSGS_MESSAGE_COLORMSG_H
#define LAB5_MSGS_MESSAGE_COLORMSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace lab5_msgs
{
template <class ContainerAllocator>
struct ColorMsg_ {
  typedef ColorMsg_<ContainerAllocator> Type;

  ColorMsg_()
  : r(0)
  , g(0)
  , b(0)
  {
  }

  ColorMsg_(const ContainerAllocator& _alloc)
  : r(0)
  , g(0)
  , b(0)
  {
  }

  typedef int64_t _r_type;
  int64_t r;

  typedef int64_t _g_type;
  int64_t g;

  typedef int64_t _b_type;
  int64_t b;


private:
  static const char* __s_getDataType_() { return "lab5_msgs/ColorMsg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "07ab5540bb99421ccc74bc089ceae6c1"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int64 r\n\
int64 g\n\
int64 b\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, r);
    ros::serialization::serialize(stream, g);
    ros::serialization::serialize(stream, b);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, r);
    ros::serialization::deserialize(stream, g);
    ros::serialization::deserialize(stream, b);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(r);
    size += ros::serialization::serializationLength(g);
    size += ros::serialization::serializationLength(b);
    return size;
  }

  typedef boost::shared_ptr< ::lab5_msgs::ColorMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lab5_msgs::ColorMsg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ColorMsg
typedef  ::lab5_msgs::ColorMsg_<std::allocator<void> > ColorMsg;

typedef boost::shared_ptr< ::lab5_msgs::ColorMsg> ColorMsgPtr;
typedef boost::shared_ptr< ::lab5_msgs::ColorMsg const> ColorMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::lab5_msgs::ColorMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::lab5_msgs::ColorMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace lab5_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::lab5_msgs::ColorMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::lab5_msgs::ColorMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::lab5_msgs::ColorMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "07ab5540bb99421ccc74bc089ceae6c1";
  }

  static const char* value(const  ::lab5_msgs::ColorMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x07ab5540bb99421cULL;
  static const uint64_t static_value2 = 0xcc74bc089ceae6c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::lab5_msgs::ColorMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "lab5_msgs/ColorMsg";
  }

  static const char* value(const  ::lab5_msgs::ColorMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::lab5_msgs::ColorMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int64 r\n\
int64 g\n\
int64 b\n\
";
  }

  static const char* value(const  ::lab5_msgs::ColorMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::lab5_msgs::ColorMsg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::lab5_msgs::ColorMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.r);
    stream.next(m.g);
    stream.next(m.b);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ColorMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lab5_msgs::ColorMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::lab5_msgs::ColorMsg_<ContainerAllocator> & v) 
  {
    s << indent << "r: ";
    Printer<int64_t>::stream(s, indent + "  ", v.r);
    s << indent << "g: ";
    Printer<int64_t>::stream(s, indent + "  ", v.g);
    s << indent << "b: ";
    Printer<int64_t>::stream(s, indent + "  ", v.b);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LAB5_MSGS_MESSAGE_COLORMSG_H

