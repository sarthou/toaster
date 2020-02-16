#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
//std
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>
#include <time.h>
#include <map>
//toaster_msgs
#include "toaster_msgs/Entity.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/AreaList.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/RobotListStamped.h"
#include "toaster_msgs/Joint.h"
#include "toaster_msgs/Empty.h"
#include "toaster_msgs/Scale.h"

#include "markerCreator.h"

#include "visualizer.h"

#include <tinyxml.h>
#include <tf/transform_listener.h>

//nameMarker rpoportionnal scale

class Run {
    visualization_msgs::MarkerArray area_list;
    visualization_msgs::MarkerArray obj_list;
    visualization_msgs::MarkerArray human_list;
    visualization_msgs::MarkerArray robot_list;
    visualization_msgs::MarkerArray arrow_list;

    std::vector<toaster_msgs::Fact> factList;

    Visualizer visualizer;

    //a vector to store marker's color
    std::map<std::string, double> agentMoving_map;

    //subscribers
    ros::Subscriber sub_objList;
    ros::Subscriber sub_areaList;
    ros::Subscriber sub_humanList;
    ros::Subscriber sub_robotList;
    ros::Subscriber sub_factList;
    ros::Subscriber sub_agentFactList;

    //publishers
    ros::Publisher pub_obj;
    ros::Publisher pub_area;
    ros::Publisher pub_human;
    ros::Publisher pub_robot;
    ros::Publisher pub_movingTwrd;

    TiXmlDocument listObj;
    TiXmlDocument listHuman;
    TiXmlDocument listHumanJoint;
    TiXmlDocument listRobot;
    TiXmlDocument listRobotJoint;

public:

    /**
     * Constructor of the run class for toaster_visualizer
     */
    Run(ros::NodeHandle& node) : visualizer(&node) {
        area_list = visualization_msgs::MarkerArray();
        obj_list = visualization_msgs::MarkerArray();
        human_list = visualization_msgs::MarkerArray();
        robot_list = visualization_msgs::MarkerArray();
        arrow_list = visualization_msgs::MarkerArray();

        //definition of subscribers
        sub_objList = node.subscribe("/pdg/objectList", 1000, &Run::chatterCallbackObjList, this);
        sub_areaList = node.subscribe("/area_manager/areaList", 1000, &Run::chatterCallbackAreaList, this);
        sub_humanList = node.subscribe("/pdg/humanList", 1000, &Run::chatterCallbackHumanList, this);
        sub_robotList = node.subscribe("/pdg/robotList", 1000, &Run::chatterCallbackRobotList, this);
        sub_factList = node.subscribe("/pdg/factList", 1000, &Run::chatterCallbackFactList, this);
        sub_agentFactList = node.subscribe("/agent_monitor/factList", 1000, &Run::chatterCallbackAgentFactList, this);

        //definition of publishers
        pub_obj = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_object", 1000);
        pub_area = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_area", 1000);
        pub_human = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_human", 1000);
        pub_robot = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_robot", 1000);
        pub_movingTwrd = node.advertise<visualization_msgs::MarkerArray>("/toaster_visualizer/marker_motion", 1000);


        // ********************************** Services ****************************************//


        // **************************************** definition function of rviz markers ********************************************************

        openXmlFile("/src/list_obj.xml", listObj);
        openXmlFile("/src/list_human.xml", listHuman);
        openXmlFile("/src/list_human_joints.xml", listHumanJoint);
        openXmlFile("/src/list_robot.xml", listRobot);
        openXmlFile("/src/list_robot_joints.xml", listRobotJoint);
    }

    bool openXmlFile(std::string fileName, TiXmlDocument& doc)
    {
      std::stringstream path;
      path << ros::package::getPath("toaster_visualizer") << fileName;
      doc = TiXmlDocument(path.str());

      if (!doc.LoadFile()) {
          ROS_WARN_ONCE("Error while loading xml file");
          ROS_WARN_ONCE("error # %d", doc.ErrorId());
          ROS_WARN_ONCE("%s", doc.ErrorDesc());
      }
    }

    // **************************************** definition function of rviz markers ********************************************************

  bool isActivated(std::string id)
  {
    bool active = false;
    for (unsigned int i = 0; i < factList.size(); i++)
    {
      if(factList[i].subjectId == id)
      {
        if(factList[i].stringValue == "active")
          active = true;
      }
    }
    return active;
  }


    bool isSeating(const toaster_msgs::Human hum) {
        //Find the head:
        unsigned int i = 0;
        for (i = 0; i < hum.meAgent.skeletonNames.size(); i++) {
            if (hum.meAgent.skeletonNames[i] == "head") {
                break;
            }
        }
        if (i == hum.meAgent.skeletonNames.size()) {
            return false;
        } else {
            if (hum.meAgent.skeletonJoint[i].meEntity.pose.position.z < 1.4)
                return true;
            else
                return false;
        }
    }

    //******************************************************** emission/reception ********************************************************

    //reception

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to obj_list
     * @param msg			reference to receive toaster_msgs::ObjectList
     * @return 			void
     */
    void chatterCallbackObjList(const toaster_msgs::ObjectListStamped::ConstPtr& msg) //toaster object list reception
    {
        obj_list.markers.clear();

        for (int i = 0; i < msg->objectList.size(); i++)
        {
            visualization_msgs::Marker m = MarkerCreator::defineObj(msg->objectList[i].meEntity.pose,
                                                      msg->objectList[i].meEntity.name,
                                                      isActivated(msg->objectList[i].meEntity.id),
                                                      visualizer.id_generator(msg->objectList[i].meEntity.id),
                                                      msg->objectList[i].meEntity.id,
                                                      listObj);

            if (visualizer.mustPrintName())
            {
                visualization_msgs::Marker mn = MarkerCreator::defineName(m);
                mn = MarkerCreator::setSize(mn, 0, 0, visualizer.getObjectNameScale());

                obj_list.markers.push_back(mn);
            }
            MarkerCreator::setMeshFromOnto(m, msg->objectList[i].meEntity.id, visualizer.getNode());

            obj_list.markers.push_back(m);

            //ROS_DEBUG("obj %d", m.id);
        }
        // extra object for environment
        geometry_msgs::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        p.position.z = -0.05;
        p.orientation.w = 1.0;
        visualization_msgs::Marker m = MarkerCreator::defineObj(p, "env", false, visualizer.id_generator("env"), "env", listObj);
        obj_list.markers.push_back(m);
    }

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to area_list
     * There is two different types of area markers so this function manages both circle and polygon types
     * @param msg			reference to receive toaster_msgs::AreaList
     * @return 			void
     */
    void chatterCallbackAreaList(const toaster_msgs::AreaList::ConstPtr& msg) {
        area_list.markers.clear();

        for (int i = 0; i < msg->areaList.size(); i++) {

            if (msg->areaList[i].isCircle == true) //circle case
            {
                visualization_msgs::Marker m = MarkerCreator::defineCircle(msg->areaList[i].center,
                        msg->areaList[i].ray, msg->areaList[i].height, msg->areaList[i].name, visualizer.id_generator(msg->areaList[i].name));

                m = MarkerCreator::setRandomColor(m);

                visualization_msgs::Marker mn = MarkerCreator::defineName(m);
                mn = MarkerCreator::setSize(mn, 0.0, 0.0, 0.3);
                mn = MarkerCreator::setPosition(mn, mn.pose.position.x, mn.pose.position.y, mn.pose.position.z + 0.5);

                area_list.markers.push_back(m);
                area_list.markers.push_back(mn);

                ROS_DEBUG("circle %d", m.id);
            } else // polygon case
            {
                visualization_msgs::MarkerArray m = MarkerCreator::definePolygon(msg->areaList[i].poly, msg->areaList[i].name, msg->areaList[i].zmin, msg->areaList[i].zmax);

               for(int iMarker =0 ; iMarker<3 ; iMarker++)
                m.markers[iMarker] = MarkerCreator::setRandomColor(m.markers[iMarker]);

                visualization_msgs::Marker mn = MarkerCreator::defineName(m.markers[0]);
                mn = MarkerCreator::setSize(mn, 0.0, 0.0, 0.1);
                mn = MarkerCreator::setPosition(mn, m.markers[0].points[0].x, m.markers[0].points[0].y,  msg->areaList[i].zmax + 0.5);
                area_list.markers.push_back(mn);

            		for (int j = 0; j<3; j++)
                  area_list.markers.push_back(m.markers[j]);
            }

        }
    }

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to robot_list
     * Robots can be represented by a single unarticulated mesh or by multiple meshs for an articulated model
     * @param msg			reference to receive toaster_msgs::RobotList
     * @return 			void
     */
    void chatterCallbackRobotList(const toaster_msgs::RobotListStamped::ConstPtr& msg)
    {
      robot_list.markers.clear();

      for(auto& robot : msg->robotList)
      {
        visualization_msgs::Marker m = MarkerCreator::defineRobot(robot.meAgent.meEntity.pose,
                1.0,robot.meAgent.meEntity.name,
                visualizer.id_generator(robot.meAgent.meEntity.name),
                listRobot);

        if (visualizer.mustPrintName())
        {
          visualization_msgs::Marker mn = MarkerCreator::defineName(m);
          mn = MarkerCreator::setPosition(mn, mn.pose.position.x, mn.pose.position.y, mn.pose.position.z + 1);
          mn = MarkerCreator::setSize(mn, 0, 0, visualizer.getRobotNameScale());

          //If the robot is moving, we intensify its name color
          std::map<std::string, double>::const_iterator it = agentMoving_map.find(robot.meAgent.meEntity.id);
          if (it != agentMoving_map.end())
              mn = MarkerCreator::setColor(mn, 0.4 + it->second * 0.6, 0.0, 0.0);
          else
              mn = MarkerCreator::setColor(mn, 0.2, 0.0, 0.0);

          robot_list.markers.push_back(mn);
        }

        size_t robot_list_size = robot_list.markers.size();
        for(auto& joint : robot.meAgent.skeletonJoint)
        {
          visualization_msgs::Marker markerTempo = MarkerCreator::defineJoint(joint, visualizer.id_generator(joint.meEntity.name));
          MarkerCreator::setMesh(markerTempo, joint.meEntity.name, &listRobotJoint);

          if(markerTempo.mesh_resource != "") // only display joint with mesh
            robot_list.markers.push_back(markerTempo);
        }

        if(robot_list.markers.size() == robot_list_size)
          robot_list.markers.push_back(m); // display the full model if no joint is displayed
      }
    }

    /**
     * CallBack creating markers based on received toaster_msgs and adding then to human_list
     * Humans can be represented by a single unarticulated mesh or by multiple meshs for an articulated model
     * @param msg			reference to receive toaster_msgs::HumanList
     * @return 			void
     */
    void chatterCallbackHumanList(const toaster_msgs::HumanListStamped::ConstPtr& msg)
    {
      human_list.markers.clear();

      for(auto& human : msg->humanList)
      {
        visualization_msgs::Marker m = MarkerCreator::defineHuman(human.meAgent.meEntity.pose,
                1.0, human.meAgent.meEntity.name,
                human.meAgent.meEntity.id,
                visualizer.id_generator(human.meAgent.meEntity.name),
                listHuman);

        if (visualizer.mustPrintName())
        {
          visualization_msgs::Marker mn = MarkerCreator::defineName(m);
          mn = MarkerCreator::setPosition(mn, mn.pose.position.x, mn.pose.position.y, mn.pose.position.z + 1);
          mn = MarkerCreator::setSize(mn, 0, 0, visualizer.getHumanNameScale());

          //If the human is moving, we intensify its color
          std::map<std::string, double>::const_iterator it = agentMoving_map.find(human.meAgent.meEntity.id);
          if (it != agentMoving_map.end())
            mn = MarkerCreator::setColor(mn, 0.0, 0.4 + it->second * 0.6, 0.0);
          else
            mn = MarkerCreator::setColor(mn, 0.0, 0.2, 0.0);

          human_list.markers.push_back(mn);
        }

        size_t human_list_size = human_list.markers.size();
        for(auto& joint : human.meAgent.skeletonJoint)
        {
          visualization_msgs::Marker markerTempo = MarkerCreator::defineJoint(joint, visualizer.id_generator(joint.meEntity.name));
          MarkerCreator::setMesh(markerTempo, joint.meEntity.name, &listHumanJoint);
          markerTempo.ns = human.meAgent.meEntity.id;

          human_list.markers.push_back(markerTempo);
        }

        if(human_list.markers.size() == human_list_size)
          human_list.markers.push_back(m); // display the full model if no joint is displayed
      }
    }

    void chatterCallbackFactList(const toaster_msgs::FactList::ConstPtr& msg)
    {
      factList.clear();
      for(unsigned int i = 0; i < msg->factList.size(); i++)
      {
        factList.push_back(msg->factList[i]);
      }
    }

    void chatterCallbackAgentFactList(const toaster_msgs::FactList::ConstPtr& msg) {
        agentMoving_map.clear();
        arrow_list.markers.clear();

        for (int iFact = 0; iFact < msg->factList.size(); iFact++) {
            if (msg->factList[iFact].property == "IsMoving")
                agentMoving_map[msg->factList[iFact].subjectId] = msg->factList[iFact].confidence;
            else if (msg->factList[iFact].property == "IsMovingToward")
            {
              visualization_msgs::Marker sub;
              visualization_msgs::Marker targ;
              visualization_msgs::Marker arrow;

              if(!visualizer.findInArray(human_list, sub, msg->factList[iFact].subjectId))
                if(!visualizer.findInArray(robot_list, sub, msg->factList[iFact].subjectId))
                  continue; // subject not found

              if(!visualizer.findInArray(human_list, targ, msg->factList[iFact].targetId))
                if(!visualizer.findInArray(robot_list, targ, msg->factList[iFact].targetId))
                  if(!visualizer.findInArray(obj_list, targ, msg->factList[iFact].targetId))
                    continue; // target not found

              // Create arrow
              bool distance = msg->factList[iFact].subProperty.compare("distance") == 0;
              std::string subtype = distance ? "distance" : "direction";
              std::string nameSpace = sub.text + " MvTwd " + subtype + targ.text;
              arrow = MarkerCreator::defineArrow(sub, targ, msg->factList[iFact].confidence,
                                                distance, visualizer.id_generator(nameSpace));
              arrow_list.markers.push_back(arrow);
            }
        }
    }

    /**
     * Function sending all marker list to rviz
     * @return 			void
     */
    void send() {
        pub_area.publish(area_list);
        pub_obj.publish(obj_list);
        pub_human.publish(human_list);
        pub_robot.publish(robot_list);
        pub_movingTwrd.publish(arrow_list);

        ros::spinOnce();
    }
};

/**
 * Main function using class run
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "Run");
    ros::NodeHandle node;
    ROS_INFO("[toaster-visu] launched");

    Run c = Run(node);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        c.send();
        loop_rate.sleep();
    }

    return 0;
}
