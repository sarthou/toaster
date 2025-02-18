// Generated by gencpp from file optitrack_ros/set_logfile.msg
// DO NOT EDIT!


#ifndef OPTITRACK_ROS_MESSAGE_SET_LOGFILE_H
#define OPTITRACK_ROS_MESSAGE_SET_LOGFILE_H

#include <ros/service_traits.h>


#include <optitrack_ros/set_logfileRequest.h>
#include <optitrack_ros/set_logfileResponse.h>


namespace optitrack_ros
{

struct set_logfile
{

typedef set_logfileRequest Request;
typedef set_logfileResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct set_logfile
} // namespace optitrack_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::optitrack_ros::set_logfile > {
  static const char* value()
  {
    return "f124a180a19de8f2d86129d2fdada6a6";
  }

  static const char* value(const ::optitrack_ros::set_logfile&) { return value(); }
};

template<>
struct DataType< ::optitrack_ros::set_logfile > {
  static const char* value()
  {
    return "optitrack_ros/set_logfile";
  }

  static const char* value(const ::optitrack_ros::set_logfile&) { return value(); }
};


// service_traits::MD5Sum< ::optitrack_ros::set_logfileRequest> should match 
// service_traits::MD5Sum< ::optitrack_ros::set_logfile > 
template<>
struct MD5Sum< ::optitrack_ros::set_logfileRequest>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack_ros::set_logfile >::value();
  }
  static const char* value(const ::optitrack_ros::set_logfileRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack_ros::set_logfileRequest> should match 
// service_traits::DataType< ::optitrack_ros::set_logfile > 
template<>
struct DataType< ::optitrack_ros::set_logfileRequest>
{
  static const char* value()
  {
    return DataType< ::optitrack_ros::set_logfile >::value();
  }
  static const char* value(const ::optitrack_ros::set_logfileRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::optitrack_ros::set_logfileResponse> should match 
// service_traits::MD5Sum< ::optitrack_ros::set_logfile > 
template<>
struct MD5Sum< ::optitrack_ros::set_logfileResponse>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack_ros::set_logfile >::value();
  }
  static const char* value(const ::optitrack_ros::set_logfileResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack_ros::set_logfileResponse> should match 
// service_traits::DataType< ::optitrack_ros::set_logfile > 
template<>
struct DataType< ::optitrack_ros::set_logfileResponse>
{
  static const char* value()
  {
    return DataType< ::optitrack_ros::set_logfile >::value();
  }
  static const char* value(const ::optitrack_ros::set_logfileResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPTITRACK_ROS_MESSAGE_SET_LOGFILE_H
