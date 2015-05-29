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

/*!
 * Pins the root of a robot to the world if "pin_root_link" is defined to be 1 within the URDF.
 *
 * Example of what should be in the URDF:
 *
 * <pre>
 *     <plugin filename="libPinPlugin.so" name="PinPlugin">
 *         <pin_root_link>1</pin_root_link>
 *     </plugin>
 * </pre>
 *
 * Based on code in:
 * https://bitbucket.org/Jraipxg/matec_control/src/5ba6eb529281d5decd8aa3b834d32a2ff8005b4f/atlas_sim/src/sm_plugin.cpp?at=develop
 */

// #include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/parser.hh>
#include <stdio.h>
#include <unistd.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  class PinPlugin: public ModelPlugin
  {
  public:
    PinPlugin()
    {
      int argc = 0;
      // ros::init(argc, NULL, "pin_plugin");
    }

    ~PinPlugin()
    {
    }

    template<typename T>
    T sdf_get_value(sdf::ElementPtr sdf, const std::string &key, const T &def)
    {
      if(sdf->HasElement(key))
      {
        std::string value = sdf->GetElement(key)->Get<std::string>();
        return boost::lexical_cast<T>(value);
      }
      else
        return def;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdfElement)
    {
      std::cerr << "PinPlugin [" << _parent->GetName() << "]: Loading the pin plugin" << std::endl;
      this->model = _parent;

      gazebo::physics::Joint_V joints = this->model->GetJoints();
      std::vector<std::string> joint_names;
      for(unsigned int i = 0; i < joints.size(); i++)
      {
        joint_names.push_back(joints[i]->GetName());
      }

      if(sdf_get_value<bool>(sdfElement, "pin_root_link", "/sm_plugin"))
      {
        std::cerr << "PinPlugin [" << model->GetName() << "]: Pinning the robot at its root!" << std::endl;
        pinRootLink();
      }
      else
      {
        std::cerr << "PinPlugin [" << model->GetName() << "]: Not pinning the robot." << std::endl;
      }

      std::cerr << "PinPlugin [" << model->GetName() << "]: Done loading pin plugin" << std::endl;
    }

    void pinRootLink()
    {
      gazebo::physics::Link_V links = this->model->GetLinks();
      gazebo::physics::LinkPtr root = links[0];
      std::cerr << "PinPlugin [" << model->GetName() << "]: Root link is " << root->GetName() << std::endl;

      gazebo::physics::WorldPtr world = this->model->GetWorld();
      std::cerr << "PinPlugin [" << model->GetName() << "]: World is " << world->GetName() << std::endl;

      physics::LinkPtr world_link = physics::LinkPtr();
      physics::JointPtr joint = world->GetPhysicsEngine()->CreateJoint("revolute", this->model);
      joint->Attach(world_link, root);
      joint->Load(world_link, root, math::Pose(root->GetWorldPose().pos, math::Quaternion())); // load adds the joint to a vector of shared pointers kept in parent and child links, preventing joint from being destroyed.
      joint->SetAxis(0, math::Vector3(0, 0, 1));
      joint->SetHighStop(0, 0);
      joint->SetLowStop(0, 0);
      joint->SetName("pin");
      joint->Init();
    }

  private:
    physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PinPlugin)

} // namespace gazebo