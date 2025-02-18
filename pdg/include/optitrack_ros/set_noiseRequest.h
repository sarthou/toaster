// Generated by gencpp from file optitrack_ros/set_noiseRequest.msg
// DO NOT EDIT!


#ifndef OPTITRACK_ROS_MESSAGE_SET_NOISEREQUEST_H
#define OPTITRACK_ROS_MESSAGE_SET_NOISEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <optitrack_ros/optitrack_ids_noise_s.h>

namespace optitrack_ros
{
template <class ContainerAllocator>
struct set_noiseRequest_
{
  typedef set_noiseRequest_<ContainerAllocator> Type;

  set_noiseRequest_()
    : noise()  {
    }
  set_noiseRequest_(const ContainerAllocator& _alloc)
    : noise(_alloc)  {
  (void)_alloc;
    }



   typedef  ::optitrack_ros::optitrack_ids_noise_s_<ContainerAllocator>  _noise_type;
  _noise_type noise;





  typedef boost::shared_ptr< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct set_noiseRequest_

typedef ::optitrack_ros::set_noiseRequest_<std::allocator<void> > set_noiseRequest;

typedef boost::shared_ptr< ::optitrack_ros::set_noiseRequest > set_noiseRequestPtr;
typedef boost::shared_ptr< ::optitrack_ros::set_noiseRequest const> set_noiseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack_ros::set_noiseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'optitrack_ros': ['optitrack_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "17af2fc49adf580ce23325b827981feb";
  }

  static const char* value(const ::optitrack_ros::set_noiseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x17af2fc49adf580cULL;
  static const uint64_t static_value2 = 0xe23325b827981febULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack_ros/set_noiseRequest";
  }

  static const char* value(const ::optitrack_ros::set_noiseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
optitrack_ids_noise_s noise\n\
\n\
================================================================================\n\
MSG: optitrack_ros/optitrack_ids_noise_s\n\
# IDL struct ::optitrack::ids::noise_s\n\
float64 pstddev\n\
float64 qstddev\n\
";
  }

  static const char* value(const ::optitrack_ros::set_noiseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.noise);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct set_noiseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack_ros::set_noiseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack_ros::set_noiseRequest_<ContainerAllocator>& v)
  {
    s << indent << "noise: ";
    s << std::endl;
    Printer< ::optitrack_ros::optitrack_ids_noise_s_<ContainerAllocator> >::stream(s, indent + "  ", v.noise);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_ROS_MESSAGE_SET_NOISEREQUEST_H
