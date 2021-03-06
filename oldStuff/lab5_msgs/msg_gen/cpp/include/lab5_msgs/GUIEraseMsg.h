/* Auto-generated by genmsg_cpp for file /home/rss-student/rss14/oldStuff/lab5_msgs/msg/GUIEraseMsg.msg */
#ifndef LAB5_MSGS_MESSAGE_GUIERASEMSG_H
#define LAB5_MSGS_MESSAGE_GUIERASEMSG_H
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

#include "std_msgs/String.h"

namespace lab5_msgs
{
template <class ContainerAllocator>
struct GUIEraseMsg_ {
  typedef GUIEraseMsg_<ContainerAllocator> Type;

  GUIEraseMsg_()
  : erase()
  {
  }

  GUIEraseMsg_(const ContainerAllocator& _alloc)
  : erase(_alloc)
  {
  }

  typedef  ::std_msgs::String_<ContainerAllocator>  _erase_type;
   ::std_msgs::String_<ContainerAllocator>  erase;


private:
  static const char* __s_getDataType_() { return "lab5_msgs/GUIEraseMsg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "25d2385573bedcaf3e10c201d306a3be"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "std_msgs/String erase\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, erase);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, erase);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(erase);
    return size;
  }

  typedef boost::shared_ptr< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GUIEraseMsg
typedef  ::lab5_msgs::GUIEraseMsg_<std::allocator<void> > GUIEraseMsg;

typedef boost::shared_ptr< ::lab5_msgs::GUIEraseMsg> GUIEraseMsgPtr;
typedef boost::shared_ptr< ::lab5_msgs::GUIEraseMsg const> GUIEraseMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace lab5_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "25d2385573bedcaf3e10c201d306a3be";
  }

  static const char* value(const  ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x25d2385573bedcafULL;
  static const uint64_t static_value2 = 0x3e10c201d306a3beULL;
};

template<class ContainerAllocator>
struct DataType< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "lab5_msgs/GUIEraseMsg";
  }

  static const char* value(const  ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "std_msgs/String erase\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
\n\
";
  }

  static const char* value(const  ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.erase);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GUIEraseMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::lab5_msgs::GUIEraseMsg_<ContainerAllocator> & v) 
  {
    s << indent << "erase: ";
s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.erase);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LAB5_MSGS_MESSAGE_GUIERASEMSG_H

