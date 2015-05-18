/*
 *  ros_control_monitor_node.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 5/18/2015
 *      Author: luca
 */

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <controller_manager_msgs/ListControllers.h>


class RosControlMonitor
{
public:
  RosControlMonitor():
    nh_("ros_control_monitor_node")
  {
    srvClient_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers", true);
    diagnosticPublisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",10,false);
  }

  void run()
  {
    controller_manager_msgs::ListControllersRequest req;
    controller_manager_msgs::ListControllersResponse resp;
    bool res = srvClient_.call(req, resp);

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostic_msgs::DiagnosticStatus controller_mgr_status;
    controller_mgr_status.name  = "Functionalities:control:controller_manager";
    if(!res)
    {
      controller_mgr_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      controller_mgr_status.message = "service call failed";
    }
    else
    {
      controller_mgr_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      controller_mgr_status.message = "service call succeded";
    }
    diagnostics.status.push_back(controller_mgr_status);

    if(res)
    {
      for( unsigned int i=0; i < resp.controller.size(); ++i)
      {
        diagnostic_msgs::DiagnosticStatus controller_status;
        controller_status.name  = std::string("Functionalities:control:controller_manager:controllers:") + resp.controller.at(i).name;
        controller_status.message = resp.controller.at(i).state;
        controller_status.level = (resp.controller.at(i).state == std::string("running") ) ?
              diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
        diagnostic_msgs::KeyValue value;
        value.key = "type";
        value.value = resp.controller.at(i).type;
        controller_status.values.push_back(value);

        value.key = "hardware_interface";
        value.value = resp.controller.at(i).hardware_interface;
        controller_status.values.push_back(value);

        std::string resources;
        for (unsigned int j=0; j< resp.controller.at(i).resources.size(); ++j)
        {
          resources += resp.controller.at(i).resources.at(j) + std::string(", ");
        }
        value.key = "resources";
        value.value = resources;
        controller_status.values.push_back(value);

        diagnostics.status.push_back(controller_status);
      }
    }

    diagnosticPublisher_.publish(diagnostics);
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient srvClient_;
  ros::Publisher  diagnosticPublisher_;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_control_monitor_node");

  ros::NodeHandle nh;
  RosControlMonitor control_monitor;

  ros::Rate rate(0.2);
  while(ros::ok())
  {
    control_monitor.run();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
