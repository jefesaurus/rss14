/* Auto-generated by genmsg_cpp for file /home/rss-student/RSS-I-group/lab6_msgs/msg/GUIPolyMsg.msg */
#ifndef LAB6_MSGS_MESSAGE_GUIPOLYMSG_H
#define LAB6_MSGS_MESSAGE_GUIPOLYMSG_H
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

#include "lab5_msgs/ColorMsg.h"

namespace lab6_msgs
{
template <class ContainerAllocator>
struct GUIPolyMsg_ {
  typedef GUIPolyMsg_<ContainerAllocator> Type;

  GUIPolyMsg_()
  : c()
  , numVertices(0)
  , x()
  , y()
  , closed(0)
  , filled(0)
  {
  }

  GUIPolyMsg_(const ContainerAllocator& _alloc)
  : c(_alloc)
  , numVertices(0)
  , x(_alloc)
  , y(_alloc)
  , closed(0)
  , filled(0)
  {
  }

  typedef  ::lab5_msgs::ColorMsg_<ContainerAllocator>  _c_type;
   ::lab5_msgs::ColorMsg_<ContainerAllocator>  c;

  typedef int32_t _numVertices_type;
  int32_t numVertices;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  x;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _y_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  y;

  typedef int32_t _closed_type;
  int32_t closed;

  typedef int32_t _filled_type;
  int32_t filled;


  ROS_DEPRECATED uint32_t get_x_size() const { return (uint32_t)x.size(); }
  ROS_DEPRECATED void set_x_size(uint32_t size) { x.resize((size_t)size); }
  ROS_DEPRECATED void get_x_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->x; }
  ROS_DEPRECATED void set_x_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->x = vec; }
  ROS_DEPRECATED uint32_t get_y_size() const { return (uint32_t)y.size(); }
  ROS_DEPRECATED void set_y_size(uint32_t size) { y.resize((size_t)size); }
  ROS_DEPRECATED void get_y_vec(std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) const { vec = this->y; }
  ROS_DEPRECATED void set_y_vec(const std::vector<float, typename ContainerAllocator::template rebind<float>::other > & vec) { this->y = vec; }
private:
  static const char* __s_getDataType_() { return "lab6_msgs/GUIPolyMsg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "3e548996167a0b4f0dd625274639c5b7"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "lab5_msgs/ColorMsg c\n\
int32 numVertices\n\
float32[] x\n\
float32[] y\n\
int32 closed\n\
int32 filled\n\
================================================================================\n\
MSG: lab5_msgs/ColorMsg\n\
int64 r\n\
int64 g\n\
int64 b\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, c);
    ros::serialization::serialize(stream, numVertices);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, closed);
    ros::serialization::serialize(stream, filled);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, c);
    ros::serialization::deserialize(stream, numVertices);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, closed);
    ros::serialization::deserialize(stream, filled);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(c);
    size += ros::serialization::serializationLength(numVertices);
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(closed);
    size += ros::serialization::serializationLength(filled);
    return size;
  }

  typedef boost::shared_ptr< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GUIPolyMsg
typedef  ::lab6_msgs::GUIPolyMsg_<std::allocator<void> > GUIPolyMsg;

typedef boost::shared_ptr< ::lab6_msgs::GUIPolyMsg> GUIPolyMsgPtr;
typedef boost::shared_ptr< ::lab6_msgs::GUIPolyMsg const> GUIPolyMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace lab6_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3e548996167a0b4f0dd625274639c5b7";
  }

  static const char* value(const  ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3e548996167a0b4fULL;
  static const uint64_t static_value2 = 0x0dd625274639c5b7ULL;
};

template<class ContainerAllocator>
struct DataType< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "lab6_msgs/GUIPolyMsg";
  }

  static const char* value(const  ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "lab5_msgs/ColorMsg c\n\
int32 numVertices\n\
float32[] x\n\
float32[] y\n\
int32 closed\n\
int32 filled\n\
================================================================================\n\
MSG: lab5_msgs/ColorMsg\n\
int64 r\n\
int64 g\n\
int64 b\n\
";
  }

  static const char* value(const  ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.c);
    stream.next(m.numVertices);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.closed);
    stream.next(m.filled);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GUIPolyMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::lab6_msgs::GUIPolyMsg_<ContainerAllocator> & v) 
  {
    s << indent << "c: ";
s << std::endl;
    Printer< ::lab5_msgs::ColorMsg_<ContainerAllocator> >::stream(s, indent + "  ", v.c);
    s << indent << "numVertices: ";
    Printer<int32_t>::stream(s, indent + "  ", v.numVertices);
    s << indent << "x[]" << std::endl;
    for (size_t i = 0; i < v.x.size(); ++i)
    {
      s << indent << "  x[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x[i]);
    }
    s << indent << "y[]" << std::endl;
    for (size_t i = 0; i < v.y.size(); ++i)
    {
      s << indent << "  y[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.y[i]);
    }
    s << indent << "closed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.closed);
    s << indent << "filled: ";
    Printer<int32_t>::stream(s, indent + "  ", v.filled);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LAB6_MSGS_MESSAGE_GUIPOLYMSG_H

