/*
 * File:   AdreamMocapHumanReader.cpp
 * Author: sdevin
 *
 * Created on October 8, 2015, 1:24 PM
 */

#include "pdg/readers/AdreamMocapHumanReader.h"

#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

AdreamMocapHumanReader::AdreamMocapHumanReader(bool fullHuman) : HumanReader()
{
  fullHuman_ = fullHuman;
}

// A human reader is a class that will read data from human(s)
void AdreamMocapHumanReader::init(ros::NodeHandle* node, std::string topicTorso, std::string topicHead, std::string topicHand, std::string param)
{
  std::cout << "[PDG] Initializing AdreamMocapHumanReader" << std::endl;
  Reader<Human>::init(node, param);
  torso_ = false;
  // ******************************************
  // Starts listening to the joint_states
  subTorso_ = node_->subscribe(topicTorso, 1, &AdreamMocapHumanReader::optitrackCallbackTorso, this);
  subHead_ = node_->subscribe(topicHead, 1, &AdreamMocapHumanReader::optitrackCallbackHead, this);
  subHand_ = node_->subscribe(topicHand, 1, &AdreamMocapHumanReader::optitrackCallbackHand, this);

    if (node_->hasParam("mocap_calib_world_x"))
      node_->getParam("mocap_calib_world_x", offset_x);
    else
    {
      offset_x = 6.164;
      std::cout << "param mocap_calib_world_x note find : use default value : " << offset_x << std::endl;
    }

    if (node_->hasParam("mocap_calib_world_y"))
      node_->getParam("mocap_calib_world_y", offset_y);
    else
    {
      offset_y = 2.956;
      std::cout << "param mocap_calib_world_y note find : use default value : " << offset_y << std::endl;
    }

    if (node_->hasParam("mocap_calib_world_z"))
      node_->getParam("mocap_calib_world_z", offset_z);
    else
    {
      offset_z = 0;
      std::cout << "param mocap_calib_world_z note find : use default value : " << offset_z << std::endl;
    }
}

void AdreamMocapHumanReader::Publish(struct toasterList_t& list_msg)
{
  if(activated_)
  {
    for (std::map<std::string, Human*>::iterator it = lastConfig_.begin(); it != lastConfig_.end(); ++it)
    {
      if (isPresent(it->first))
      {
        toaster_msgs::Fact fact_msg = DefaultFactMsg(it->first, it->second->getTime());
        list_msg.fact_msg.factList.push_back(fact_msg);

        toaster_msgs::Human human_msg;
        fillEntity(it->second, human_msg.meAgent.meEntity);

        for (std::map<std::string, Joint*>::iterator itJoint = lastConfig_[it->first]->skeleton_.begin(); itJoint != lastConfig_[it->first]->skeleton_.end(); ++itJoint)
        {
          toaster_msgs::Joint joint_msg;
          human_msg.meAgent.skeletonNames.push_back(itJoint->first);
          fillEntity((itJoint->second), joint_msg.meEntity);
          joint_msg.jointOwner = it->first;

          human_msg.meAgent.skeletonJoint.push_back(joint_msg);
        }
        list_msg.human_msg.humanList.push_back(human_msg);
      }
    }
  }
}

void AdreamMocapHumanReader::optitrackCallbackHead(const optitrack_ros::or_pose_estimator_state::ConstPtr & msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {
        std::string humId = "HERAKLES_HUMAN1";
        //create a new human with the same id as the message
        if (lastConfig_.find(humId) == lastConfig_.end())
        {
          curHuman = new Human(humId);
          curHuman->setName(humId);
        }
        else
          curHuman = lastConfig_[humId];

        if (msg->pos.size() != 0) {

            tf::Quaternion q(msg->att[0].qx, msg->att[0].qy, msg->att[0].qz, msg->att[0].qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getEulerYPR(yaw, pitch, roll);

            if (!torso_)
            {
              Mobility curHumanMobility = FULL;
              curHuman->setId(humId);
              curHuman->setName(humId);

              curHuman->setMobility(curHumanMobility);

                //set human position
                bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
                humanPosition.set<0>(msg->pos[0].x + offset_x);
                humanPosition.set<1>(msg->pos[0].y + offset_y);
                humanPosition.set<2>(0/*msg->pos[0].z + offset_z - 1.48*/);

                //set the human orientation
                std::vector<double> humanOrientation;

                //transform the pose message
                humanOrientation.push_back(0.0);
                humanOrientation.push_back(0.0);
                humanOrientation.push_back(yaw);

                //put the data in the human
                curHuman->setOrientation(humanOrientation);
                curHuman->setPosition(humanPosition);
                curHuman->setTime(now.toNSec());

                lastConfig_[humId] = curHuman;

                //update the base
                std::string jointName = "base";

                if (curHuman->skeleton_.find(jointName) == curHuman->skeleton_.end())
                {
                  curJoint = new Joint(jointName, humId);
                  curJoint->setName(jointName);
                }
                else
                  curJoint = curHuman->skeleton_[jointName];

                curJoint->setPosition(humanPosition);
                curJoint->setOrientation(humanOrientation);
                curJoint->setTime(now.toNSec());

                lastConfig_[humId]->skeleton_[jointName] = curJoint;
            }

            //update the head
            std::string jointNameHead = "head";

            if (curHuman->skeleton_.find(jointNameHead) == curHuman->skeleton_.end()) {
                curJoint = new Joint(jointNameHead, humId);
                curJoint->setName(jointNameHead);
            } else {
                curJoint = curHuman->skeleton_[jointNameHead];
            }

            bg::model::point<double, 3, bg::cs::cartesian> headPosition;
            headPosition.set<0>(msg->pos[0].x + offset_x);
            headPosition.set<1>(msg->pos[0].y + offset_y);
            headPosition.set<2>(msg->pos[0].z + offset_z);
            std::vector<double> headOrientation;
            headOrientation.push_back(roll);
            headOrientation.push_back(pitch);
            headOrientation.push_back(yaw);

            curJoint->setPosition(headPosition);
            curJoint->setOrientation(headOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointNameHead] = curJoint;
        }

    } catch (tf::TransformException ex) {
        ROS_ERROR("[AdreamMocap Head transfor] %s", ex.what());
    }
}

void AdreamMocapHumanReader::optitrackCallbackHand(const optitrack_ros::or_pose_estimator_state::ConstPtr & msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {

        std::string humId = "HERAKLES_HUMAN1";
        std::string jointName = "rightHand";

        if (lastConfig_.find(humId) == lastConfig_.end())
          return; // We wait that head is detected so that the human is created
        else
          curHuman = lastConfig_[humId];

        if (curHuman->skeleton_.find(jointName) == curHuman->skeleton_.end())
        {
          curJoint = new Joint(jointName, humId);
          curJoint->setName(jointName);
        }
        else
          curJoint = curHuman->skeleton_[jointName];

        if (msg->pos.size() != 0) {
            bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
            jointPosition.set<0>(msg->pos[0].x + offset_x);
            jointPosition.set<1>(msg->pos[0].y + offset_y);
            jointPosition.set<2>(msg->pos[0].z + offset_z);

            std::vector<double> jointOrientation;

            tf::Quaternion q(msg->att[0].qx, msg->att[0].qy, msg->att[0].qz, msg->att[0].qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getEulerYPR(yaw, pitch, roll);

            jointOrientation.push_back(roll);
            jointOrientation.push_back(pitch);
            jointOrientation.push_back(yaw);

            curJoint->setPosition(jointPosition);
            curJoint->setOrientation(jointOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointName] = curJoint;
        }

    } catch (tf::TransformException ex) {
        ROS_ERROR("[AdreamMocap Hand transfor] %s", ex.what());
    }
}

void AdreamMocapHumanReader::optitrackCallbackTorso(const optitrack_ros::or_pose_estimator_state::ConstPtr & msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;
    Joint* jointBase;

    try {

        std::string humId = "HERAKLES_HUMAN1";
        std::string jointName = "torso";

        if (lastConfig_.find(humId) == lastConfig_.end()) {
            // We wait that head is detected so that the human is created
            return;
        } else {
            curHuman = lastConfig_[humId];
            if (!torso_)
                torso_ = true;
        }

        if (curHuman->skeleton_.find(jointName) == curHuman->skeleton_.end()) {
            curJoint = new Joint(jointName, humId);
            curJoint->setName(jointName);
        } else {
            curJoint = curHuman->skeleton_[jointName];
        }

        if (msg->pos.size() != 0) {

          Mobility curHumanMobility = FULL;
          curHuman->setId(humId);
          curHuman->setName(humId);

          curHuman->setMobility(curHumanMobility);
          
            //set human position
            bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
            humanPosition.set<0>(msg->pos[0].x + offset_x);
            humanPosition.set<1>(msg->pos[0].y + offset_y);
            humanPosition.set<2>(0/*msg->pos[0].z + offset_z - 1.31*/);

            //set the human orientation
            std::vector<double> humanOrientation;

            //transform the pose message

            tf::Quaternion q(msg->att[0].qx, msg->att[0].qy, msg->att[0].qz, msg->att[0].qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getEulerYPR(yaw, pitch, roll);

            humanOrientation.push_back(roll);
            humanOrientation.push_back(pitch);
            humanOrientation.push_back(yaw);

            //put the data in the human
            curHuman->setOrientation(humanOrientation);
            curHuman->setPosition(humanPosition);
            curHuman->setTime(now.toNSec());

            lastConfig_[humId] = curHuman;

            //update the base
            std::string jointBaseName = "base";
            std::vector<double> baseOrientation;

            baseOrientation.push_back(0.0);
            baseOrientation.push_back(0.0);
            baseOrientation.push_back(yaw);


            if (curHuman->skeleton_.find(jointBaseName) == curHuman->skeleton_.end()) {
                jointBase = new Joint(jointBaseName, humId);
                jointBase->setName(jointBaseName);
            } else {
                jointBase = curHuman->skeleton_[jointBaseName];
            }

            jointBase->setPosition(humanPosition);
            jointBase->setOrientation(baseOrientation);
            jointBase->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointBaseName] = jointBase;

            bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
            jointPosition.set<0>(msg->pos[0].x + offset_x);
            jointPosition.set<1>(msg->pos[0].y + offset_y);
            jointPosition.set<2>(msg->pos[0].z + offset_z);

            curJoint->setPosition(jointPosition);
            curJoint->setOrientation(humanOrientation);
            curJoint->setTime(now.toNSec());

            lastConfig_[humId]->skeleton_[jointName] = curJoint;
        }

    } catch (tf::TransformException ex) {
        ROS_ERROR("[AdreamMocap Torso transfor] %s", ex.what());

    }
}
