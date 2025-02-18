// Generated by gencpp from file optitrack_ros/killResponse.msg
// DO NOT EDIT!


#ifndef OPTITRACK_ROS_MESSAGE_KILLRESPONSE_H
#define OPTITRACK_ROS_MESSAGE_KILLRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace optitrack_ros
{
template <class ContainerAllocator>
struct killResponse_
{
  typedef killResponse_<ContainerAllocator> Type;

  killResponse_()
    : genom_success(false)
    , genom_exdetail()  {
    }
  killResponse_(const ContainerAllocator& _alloc)
    : genom_success(false)
    , genom_exdetail(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _genom_success_type;
  _genom_success_type genom_success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _genom_exdetail_type;
  _genom_exdetail_type genom_exdetail;





  typedef boost::shared_ptr< ::optitrack_ros::killResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack_ros::killResponse_<ContainerAllocator> const> ConstPtr;

}; // struct killResponse_

typedef ::optitrack_ros::killResponse_<std::allocator<void> > killResponse;

typedef boost::shared_ptr< ::optitrack_ros::killResponse > killResponsePtr;
typedef boost::shared_ptr< ::optitrack_ros::killResponse const> killResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack_ros::killResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack_ros::killResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'optitrack_ros': ['optitrack_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::killResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::killResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::killResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::killResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::killResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::killResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack_ros::killResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0d79450287345aef3f3e331856b25242";
  }

  static const char* value(const ::optitrack_ros::killResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0d79450287345aefULL;
  static const uint64_t static_value2 = 0x3f3e331856b25242ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack_ros::killResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack_ros/killResponse";
  }

  static const char* value(const ::optitrack_ros::killResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack_ros::killResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool genom_success\n\
string genom_exdetail\n\
\n\
";
  }

  static const char* value(const ::optitrack_ros::killResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack_ros::killResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.genom_success);
      stream.next(m.genom_exdetail);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct killResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack_ros::killResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack_ros::killResponse_<ContainerAllocator>& v)
  {
    s << indent << "genom_success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.genom_success);
    s << indent << "genom_exdetail: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.genom_exdetail);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_ROS_MESSAGE_KILLRESPONSE_H
