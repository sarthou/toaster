// Generated by gencpp from file optitrack_ros/set_logfileRequest.msg
// DO NOT EDIT!


#ifndef OPTITRACK_ROS_MESSAGE_SET_LOGFILEREQUEST_H
#define OPTITRACK_ROS_MESSAGE_SET_LOGFILEREQUEST_H


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
struct set_logfileRequest_
{
  typedef set_logfileRequest_<ContainerAllocator> Type;

  set_logfileRequest_()
    : logfile()  {
    }
  set_logfileRequest_(const ContainerAllocator& _alloc)
    : logfile(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _logfile_type;
  _logfile_type logfile;





  typedef boost::shared_ptr< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> const> ConstPtr;

}; // struct set_logfileRequest_

typedef ::optitrack_ros::set_logfileRequest_<std::allocator<void> > set_logfileRequest;

typedef boost::shared_ptr< ::optitrack_ros::set_logfileRequest > set_logfileRequestPtr;
typedef boost::shared_ptr< ::optitrack_ros::set_logfileRequest const> set_logfileRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack_ros::set_logfileRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3f67da92839419477a7021defc0f8465";
  }

  static const char* value(const ::optitrack_ros::set_logfileRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3f67da9283941947ULL;
  static const uint64_t static_value2 = 0x7a7021defc0f8465ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack_ros/set_logfileRequest";
  }

  static const char* value(const ::optitrack_ros::set_logfileRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string logfile\n\
";
  }

  static const char* value(const ::optitrack_ros::set_logfileRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.logfile);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct set_logfileRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack_ros::set_logfileRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack_ros::set_logfileRequest_<ContainerAllocator>& v)
  {
    s << indent << "logfile: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.logfile);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_ROS_MESSAGE_SET_LOGFILEREQUEST_H
