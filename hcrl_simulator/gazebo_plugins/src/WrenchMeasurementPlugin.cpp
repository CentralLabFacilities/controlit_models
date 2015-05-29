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

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <hcrl_simulator/Wrenches.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  class WrenchMeasurementPlugin : public ModelPlugin
  {
    public: 
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
      {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WrenchMeasurementPlugin::OnUpdate, this, _1));
      
        //set up ros publisher
        ros::M_string map;
        ros::init(map, "wrench_measurement_plugin");
        m_nh = new ros::NodeHandle("~");
        m_ft_pub = m_nh->advertise<hcrl_simulator::Wrenches>("/joint_wrenches", 1, true);
        m_js_pub = m_nh->advertise<sensor_msgs::JointState>("/joint_states", 1, true);
      }

      geometry_msgs::Wrench gazeboWrenchToROS(gazebo::physics::JointWrench gazebo_wrench)
      {
        geometry_msgs::Wrench ros_wrench;
        ros_wrench.force.x = gazebo_wrench.body2Force.x;
        ros_wrench.force.y = gazebo_wrench.body2Force.y;
        ros_wrench.force.z = gazebo_wrench.body2Force.z;
        ros_wrench.torque.x = gazebo_wrench.body2Torque.x;
        ros_wrench.torque.y = gazebo_wrench.body2Torque.y;
        ros_wrench.torque.z = gazebo_wrench.body2Torque.z;
        return ros_wrench;
      }

      // Called by the world update start event
      void OnUpdate(const common::UpdateInfo & /*_info*/)
      {
        hcrl_simulator::Wrenches wrenches;
        sensor_msgs::JointState js;
        gazebo::physics::Joint_V joints = this->model->GetJoints();
        for(unsigned int i = 0; i < joints.size(); i++)
        {
          wrenches.names.push_back(joints[i]->GetName());
          js.name.push_back(joints[i]->GetName());

          gazebo::physics::JointWrench gazebo_wrench = joints[i]->GetForceTorque(0);
          wrenches.wrenches.push_back(gazeboWrenchToROS(gazebo_wrench));

          //figure out which torque element is on the axis of the joint
          math::Vector3 axis = joints[i]->GetLocalAxis(0);
          double pos = joints[i]->GetAngle(0).Radian();
          double vel = joints[i]->GetVelocity(0);
          double torque = axis.x*gazebo_wrench.body2Torque.x + axis.y*gazebo_wrench.body2Torque.y + axis.z*gazebo_wrench.body2Torque.z;
          js.position.push_back(pos);
          js.velocity.push_back(vel);
          js.effort.push_back(torque);
        }

        js.header.stamp = ros::Time::now();
        m_js_pub.publish(js);

        wrenches.header.stamp = ros::Time::now();
        m_ft_pub.publish(wrenches);
      }

    
    private: 
      // Pointer to the model
      physics::ModelPtr model;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      // ros stuff
      ros::NodeHandle* m_nh;
      ros::Publisher m_ft_pub;
      ros::Publisher m_js_pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WrenchMeasurementPlugin)
}
