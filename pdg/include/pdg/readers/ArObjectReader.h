/*
 * File:   ArObjectReader.h
 * Author: JPaul Marcade
 *
 * Created on July, 2016
 */

#ifndef AROBJECTREADER_H
#define AROBJECTREADER_H

//This class reads topic from Ar_track_alvar and converts data into toaster-lib type.

#include "ObjectReader.h"
#include <ros/ros.h>
#include <string>
#include "visualization_msgs/Marker.h"

#include "tf/transform_listener.h"

class ArObjectReader : public ObjectReader {

public:
  ArObjectReader();
	~ArObjectReader();

  void init(ros::NodeHandle* node, std::string topic, std::string param);

private:

  tf::TransformListener* listener_;

  void CallbackObj(const visualization_msgs::Marker::ConstPtr& msg);
};

#endif	/* AROBJECTREADER_H */
