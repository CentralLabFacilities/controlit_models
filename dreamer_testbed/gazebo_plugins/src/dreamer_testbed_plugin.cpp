/*
 * Copyright (C) 2015 The University of Texas at Austin and the
 * Institute of Human Machine Cognition. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <stdexcept>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Angle.hh>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#define JOINT_STATE_PUBLISH_FREQ_HZ 400
#define TOLERANCE 1e-6

// namespace gazebo {
//   namespace common {
//     #define gzout (gazebo::common::Console::Instance()->ColorErr("Dbg",__FILE__, __LINE__,36))
//     #define gzprint (gazebo::common::Console::Instance()->ColorErr("FYI:",__FILE__, __LINE__,25))
//   }
// }

namespace gazebo {
  class DreamerTestbedPlugin : public ModelPlugin
  {

    public: DreamerTestbedPlugin()
    {
      // Start up ROS
      std::string name = "dreamer_testbed_plugin";
      int argc = 0;
      ros::init(argc, NULL, name);

      lastPubTime = ros::Time::now();

      CAMERA_JOINT_NAMES = new std::string[4] {
        "webcam1_mount_x",
        "webcam1_mount_y",
        "webcam2_mount_x",
        "webcam2_mount_y"
      };
    }

    public: ~DreamerTestbedPlugin()
    {
      delete this->node;
      // delete[] this->JOINT_NAMES;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      physics::Joint_V allJoints = this->model->GetJoints();
      std::vector<std::string> joint_names;

      for (size_t ii = 0; ii < allJoints.size(); ii++)
      {
        // Pick out non-fixed joints
        if(allJoints[ii]->GetLowerLimit(0).Radian() < -TOLERANCE && allJoints[ii]->GetUpperLimit(0).Radian() > TOLERANCE)
        {
          joints.insert(make_pair(allJoints[ii]->GetName(), allJoints[ii]));
          joint_names.push_back(allJoints[ii]->GetName());
          //gzprint<<"Joint "<<allJoints[ii]->GetName()<<" is movable."<<std::endl;
        }

      }

      // TODO : CHECK base joints are in all joints!!!
      // TODO : Get KP/KD from parameters at startup!!!

      // Set up parameters for this plugin
      for (size_t ii = 0; ii < 4; ii++)
      {
          physics::JointPtr _jptr = this->model->GetJoint(CAMERA_JOINT_NAMES[ii]);
          camera_joints.insert(std::make_pair(CAMERA_JOINT_NAMES[ii], _jptr));
          gzwarn<<CAMERA_JOINT_NAMES[ii]<<" : "<<ii%2<<std::endl;
          if(ii%2 == 0) //x-axis joints
          {
            kp.insert(std::make_pair(CAMERA_JOINT_NAMES[ii], 0.0001));
            kd.insert(std::make_pair(CAMERA_JOINT_NAMES[ii], 0.001));
          }
          else //y-axis joints
          {
            kp.insert(std::make_pair(CAMERA_JOINT_NAMES[ii], 0.001));
            kd.insert(std::make_pair(CAMERA_JOINT_NAMES[ii], 0.01));
          }
          goalPosition.insert(std::make_pair(CAMERA_JOINT_NAMES[ii],0.0));
      }

      // ROS Nodehandle
      this->node = new ros::NodeHandle;

      // Listeners for published updates to embedded controller
      this->jointPositionGoalSub = this->node->subscribe("camera_goal_position", 1000, &DreamerTestbedPlugin::GoalJointPositionCallback, this);

      // for(auto & jnt : camera_joints)
      // {
      //   gzprint<<"Have a pointer to joint : "<<jnt.first<<std::endl;
      // }

      // For publishing the robot's current state
      // this->pub = this->node->advertise<sensor_msgs::JointState>("robot_state", 1000);

      // For subscribing to the command issued by the controller
      // this->subJointState = this->node->subscribe<sensor_msgs::JointState>("command", 1000,
      //   &DreamerTestbedPlugin::commandCallbackJointState, this);

      // this->subMultiArray = this->node->subscribe<std_msgs::Float64MultiArray>("command_array", 1000,
      //   &DreamerTestbedPlugin::commandCallbackMultiArray, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DreamerTestbedPlugin::onUpdate, this));
    }

    public: void GoalJointPositionCallback(const std_msgs::Float64MultiArray goal_msg)
    {
        int ii = 0;
        for (auto & jnt : camera_joints)
        {
          goalPosition[jnt.first] = goal_msg.data[ii++];
          gzdbg<<"setting joint "<<jnt.first<<" to "<<goalPosition[jnt.first]<<std::endl;
        }

        gzdbg << "Dreamer Testbed Camera Position callback\n";
    }

    /*!
     * Periodically called by Gazebo during each cycle of the simulation.
     */
    public: void onUpdate()
    {
      // Process pending ROS messages, etc.
      ros::spinOnce();

      // // A message for holding the robot's current state
      // sensor_msgs::JointState currRobotState;
      // currRobotState.header.stamp = ros::Time::now();

      // Update the torque commands and fill in the joint state message
      for(auto & jnt : camera_joints)
      {
        physics::JointPtr camera_joint_ptr = camera_joints[jnt.first];
        double angle = camera_joint_ptr->GetAngle(0).Radian();
        double velocity = camera_joint_ptr->GetVelocity(0);
        double torque = kp[jnt.first] * (goalPosition[jnt.first] - angle)
                              - kd[jnt.first] * velocity;
        camera_joint_ptr->SetForce(0,torque);
      }

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle * node;

    // ROS Subscriber for JointState messages
    ros::Subscriber subJointState;

    // ROS subscriber for Float64MultiArray messages
    // ros::Subscriber subMultiArray;

    // ROS Publisher
    ros::Publisher pub;

    // Store joint pointers in a map indexed by the joint name interpretted from the URDF file

  private:
    // Stores a pointer to each joint
    std::map<std::string, physics::JointPtr> joints;
    std::map<std::string, physics::JointPtr> camera_joints;

    // Embedded control parameters (for base joints)
    std::map<std::string, double> kp;
    std::map<std::string, double> kd;
    std::map<std::string, double> goalPosition;

    // Subscribers to ros topics for hand controls
    ros::Subscriber jointPositionGoalSub;

    // The time when the robot's state was last published
    ros::Time lastPubTime;

    // Hard code the joint names.  TODO: Obtain this via a ROS service call to the controller.
    std::string * CAMERA_JOINT_NAMES;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DreamerTestbedPlugin)
}
