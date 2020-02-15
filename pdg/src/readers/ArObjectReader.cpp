/*
 * File:   ArObjectReader.cpp
 * Author: JPaul Marcade
 *
 * Created on July, 2016
 */

#include "pdg/readers/ArObjectReader.h"

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
  // ******************************************
  // Starts listening to the joint_states
  listener_ = new tf::TransformListener;
  sub_ = node_->subscribe(topic, 1, &ArObjectReader::CallbackObj, this);
}

void ArObjectReader::CallbackObj(const visualization_msgs::Marker::ConstPtr& msg) {

  if(activated_)
  {
  	ros::Time now = ros::Time::now();
  	MovableObject* curObject;

  	//create a new object with the same id as the message
    lastConfigMutex_.lock();
//    std::cout << "ar obj " << msg->ns << std::endl;
  	if (globalLastConfig_.find(msg->ns) == globalLastConfig_.end()) {
  		curObject = new MovableObject(msg->ns);
  		curObject->setName("MilkBox"); //msg->ns
      increaseNbObjects();
  	} else
  		curObject = globalLastConfig_[msg->ns];

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
  	globalLastConfig_[msg->ns]=curObject;
    lastConfigMutex_.unlock();
    lastConfig_[msg->ns]=curObject;
  }
}
