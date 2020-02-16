/*
 * File:   ArObjectReader.cpp
 * Author: JPaul Marcade
 *
 * Created on July, 2016
 */

#include "pdg/readers/ArObjectReader.h"

#include "toaster_msgs/OntologeniusService.h"

#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <sys/time.h>
#include <ostream>

ArObjectReader::ArObjectReader() : ObjectReader()
{
    childs_.push_back(this);
}

ArObjectReader::~ArObjectReader()
{
  if(listener_)
    delete listener_;
}

void ArObjectReader::init(ros::NodeHandle* node, std::string topic, std::string param)
{
  std::cout << "[PDG] Initializing ArObjectReader" << std::endl;
  Reader<MovableObject>::init(node, param);

  // Starts listening to the joint_states
  listener_ = new tf::TransformListener;
  onto_client_ = node_->serviceClient<toaster_msgs::OntologeniusService>("ontologenius/individual/robot", true);

  sub_ = node_->subscribe(topic, 1, &ArObjectReader::CallbackObj, this);
}

void ArObjectReader::CallbackObj(const visualization_msgs::Marker::ConstPtr& msg) {

  if(activated_)
  {
    if(msg->ns != "main_shapes")
      return;

    std::string object_id = getObjectId(msg->id);
    std::string object_name = "";
    if(object_id == "")
    {
      object_id = "obj_" + std::to_string(msg->id);
      object_name = object_id;
    }
    else
      object_name = getObjectType(object_id);

  	ros::Time now = ros::Time::now();
  	MovableObject* curObject;

  	//create a new object with the same id as the message
    lastConfigMutex_.lock();

  	if (globalLastConfig_.find(object_id) == globalLastConfig_.end()) {
  		curObject = new MovableObject(object_id);
  		curObject->setName(object_name);
      increaseNbObjects();
  	} else
  		curObject = globalLastConfig_[object_id];

      tf::Transform cam_to_target;
      tf::poseMsgToTF(msg->pose, cam_to_target);

      tf::StampedTransform map_to_cam;
      listener_->lookupTransform("/map", msg->header.frame_id, ros::Time(0), map_to_cam);

      tf::Transform map_to_target;
      map_to_target = map_to_cam * cam_to_target;

      geometry_msgs::Pose pose;
      tf::poseTFToMsg(map_to_target, pose);

    lastConfigMutex_.unlock();

  	//set object position
  	bg::model::point<double, 3, bg::cs::cartesian> objectPosition;
  	objectPosition.set<0>(pose.position.x);
  	objectPosition.set<1>(pose.position.y);
  	objectPosition.set<2>(pose.position.z);

  	//set the object orientation
  	std::vector<double> objectOrientation;

  	tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  	double roll, pitch, yaw;
  	tf::Matrix3x3 m(q);
  	m.getEulerYPR(yaw,pitch,roll);

  	objectOrientation.push_back(roll);
  	objectOrientation.push_back(pitch);
  	objectOrientation.push_back(yaw);

  	//put the data in the object
  	curObject->setOrientation(objectOrientation);
  	curObject->setPosition(objectPosition);
  	curObject->setTime(now.toNSec());      //Similar to AdreamMoCapHumanReader. Is it better to use time stamp from msg

    lastConfigMutex_.lock();
  	globalLastConfig_[object_id]=curObject;
    lastConfigMutex_.unlock();
    lastConfig_[object_id]=curObject;
  }
}

std::string ArObjectReader::getObjectId(int tag_id)
{
  toaster_msgs::OntologeniusService srv;
  srv.request.action = "getFrom";
  srv.request.param = "real#" + std::to_string(tag_id) + ":hasArId";

  if(onto_client_.call(srv))
  {
    if(srv.response.values.size())
      return srv.response.values[0];
    else
      return "";
  }
  else
  {
    onto_client_ = node_->serviceClient<toaster_msgs::OntologeniusService>("ontologenius/individual/robot", true);
    if(onto_client_.call(srv))
    {
      if(srv.response.values.size())
        return srv.response.values[0];
      else
        return "";
    }
    else
      return "";
  }
}

std::string ArObjectReader::getObjectType(const std::string& obj_id)
{
  toaster_msgs::OntologeniusService srv;
  srv.request.action = "getUp";
  srv.request.param = "obj_id -d 1";

  if(onto_client_.call(srv))
  {
    if(srv.response.values.size())
      return srv.response.values[0];
    else
      return "";
  }
  else
  {
    onto_client_ = node_->serviceClient<toaster_msgs::OntologeniusService>("ontologenius/individual/robot", true);
    if(onto_client_.call(srv))
    {
      if(srv.response.values.size())
        return srv.response.values[0];
      else
        return "";
    }
    else
      return "";
  }
}
