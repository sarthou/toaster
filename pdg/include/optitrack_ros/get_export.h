// Generated by gencpp from file optitrack_ros/get_export.msg
// DO NOT EDIT!


#ifndef OPTITRACK_ROS_MESSAGE_GET_EXPORT_H
#define OPTITRACK_ROS_MESSAGE_GET_EXPORT_H

#include <ros/service_traits.h>


#include <optitrack_ros/get_exportRequest.h>
#include <optitrack_ros/get_exportResponse.h>


namespace optitrack_ros
{

struct get_export
{

typedef get_exportRequest Request;
typedef get_exportResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct get_export
} // namespace optitrack_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::optitrack_ros::get_export > {
  static const char* value()
  {
    return "7279d7473f4bbd6837a77c478ea5e664";
  }

  static const char* value(const ::optitrack_ros::get_export&) { return value(); }
};

template<>
struct DataType< ::optitrack_ros::get_export > {
  static const char* value()
  {
    return "optitrack_ros/get_export";
  }

  static const char* value(const ::optitrack_ros::get_export&) { return value(); }
};


// service_traits::MD5Sum< ::optitrack_ros::get_exportRequest> should match 
// service_traits::MD5Sum< ::optitrack_ros::get_export > 
template<>
struct MD5Sum< ::optitrack_ros::get_exportRequest>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack_ros::get_export >::value();
  }
  static const char* value(const ::optitrack_ros::get_exportRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack_ros::get_exportRequest> should match 
// service_traits::DataType< ::optitrack_ros::get_export > 
template<>
struct DataType< ::optitrack_ros::get_exportRequest>
{
  static const char* value()
  {
    return DataType< ::optitrack_ros::get_export >::value();
  }
  static const char* value(const ::optitrack_ros::get_exportRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::optitrack_ros::get_exportResponse> should match 
// service_traits::MD5Sum< ::optitrack_ros::get_export > 
template<>
struct MD5Sum< ::optitrack_ros::get_exportResponse>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack_ros::get_export >::value();
  }
  static const char* value(const ::optitrack_ros::get_exportResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack_ros::get_exportResponse> should match 
// service_traits::DataType< ::optitrack_ros::get_export > 
template<>
struct DataType< ::optitrack_ros::get_exportResponse>
{
  static const char* value()
  {
    return DataType< ::optitrack_ros::get_export >::value();
  }
  static const char* value(const ::optitrack_ros::get_exportResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPTITRACK_ROS_MESSAGE_GET_EXPORT_H
