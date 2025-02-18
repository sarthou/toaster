// Generated by gencpp from file optitrack_ros/genom_state_component.msg
// DO NOT EDIT!


#ifndef OPTITRACK_ROS_MESSAGE_GENOM_STATE_COMPONENT_H
#define OPTITRACK_ROS_MESSAGE_GENOM_STATE_COMPONENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <optitrack_ros/genom_state_task.h>

namespace optitrack_ros
{
template <class ContainerAllocator>
struct genom_state_component_
{
  typedef genom_state_component_<ContainerAllocator> Type;

  genom_state_component_()
    : task()
    , digest()
    , date()
    , version()  {
    }
  genom_state_component_(const ContainerAllocator& _alloc)
    : task(_alloc)
    , digest(_alloc)
    , date(_alloc)
    , version(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::optitrack_ros::genom_state_task_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::optitrack_ros::genom_state_task_<ContainerAllocator> >::other >  _task_type;
  _task_type task;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _digest_type;
  _digest_type digest;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _date_type;
  _date_type date;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _version_type;
  _version_type version;





  typedef boost::shared_ptr< ::optitrack_ros::genom_state_component_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack_ros::genom_state_component_<ContainerAllocator> const> ConstPtr;

}; // struct genom_state_component_

typedef ::optitrack_ros::genom_state_component_<std::allocator<void> > genom_state_component;

typedef boost::shared_ptr< ::optitrack_ros::genom_state_component > genom_state_componentPtr;
typedef boost::shared_ptr< ::optitrack_ros::genom_state_component const> genom_state_componentConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack_ros::genom_state_component_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack_ros::genom_state_component_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/msg'], 'optitrack_ros': ['optitrack_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack_ros::genom_state_component_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack_ros::genom_state_component_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack_ros::genom_state_component_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7045eed14f90201e8d03c3bee8bd266";
  }

  static const char* value(const ::optitrack_ros::genom_state_component_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7045eed14f90201ULL;
  static const uint64_t static_value2 = 0xe8d03c3bee8bd266ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack_ros/genom_state_component";
  }

  static const char* value(const ::optitrack_ros::genom_state_component_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# IDL struct ::genom::state::component\n\
genom_state_task[] task\n\
string digest\n\
string date\n\
string version\n\
\n\
================================================================================\n\
MSG: optitrack_ros/genom_state_task\n\
# IDL struct ::genom::state::task\n\
string name\n\
genom_state_rusage rusage\n\
genom_state_activity[] activity\n\
\n\
================================================================================\n\
MSG: optitrack_ros/genom_state_rusage\n\
# IDL struct ::genom::state::rusage\n\
uint32 cycles\n\
genom_state_stats timings\n\
genom_state_stats load\n\
\n\
================================================================================\n\
MSG: optitrack_ros/genom_state_stats\n\
# IDL struct ::genom::state::stats\n\
float32 last\n\
float32 max\n\
float32 avg\n\
\n\
================================================================================\n\
MSG: optitrack_ros/genom_state_activity\n\
# IDL struct ::genom::state::activity\n\
uint32 id\n\
string name\n\
";
  }

  static const char* value(const ::optitrack_ros::genom_state_component_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.task);
      stream.next(m.digest);
      stream.next(m.date);
      stream.next(m.version);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct genom_state_component_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack_ros::genom_state_component_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack_ros::genom_state_component_<ContainerAllocator>& v)
  {
    s << indent << "task[]" << std::endl;
    for (size_t i = 0; i < v.task.size(); ++i)
    {
      s << indent << "  task[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::optitrack_ros::genom_state_task_<ContainerAllocator> >::stream(s, indent + "    ", v.task[i]);
    }
    s << indent << "digest: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.digest);
    s << indent << "date: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.date);
    s << indent << "version: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.version);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_ROS_MESSAGE_GENOM_STATE_COMPONENT_H
