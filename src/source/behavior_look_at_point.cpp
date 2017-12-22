/*!*******************************************************************************************
 *  \file       behavior_rotate.cpp
 *  \brief      Behavior Rotate implementation file.
 *  \details    This file implements the behaviorRotate class.
 *  \authors    Rafael Artiñano Muñoz
 *  \copyright  Copyright 2016 Universidad Politecnica de Madrid (UPM)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/
#include "../include/behavior_look_at_point.h"
BehaviorLookAtPoint::BehaviorLookAtPoint():BehaviorProcess()
{

}

BehaviorLookAtPoint::~BehaviorLookAtPoint()
{

}

void BehaviorLookAtPoint::ownSetUp()
{
 std::cout << "ownSetup" << std::endl;

 ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("drone_id", drone_id, "1");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone"+drone_id);
  private_nh.param<std::string>("my_stack_directory", my_stack_directory,
                  "~/workspace/ros/quadrotor_stack_catkin/src/quadrotor_stack");

  private_nh.param<std::string>("estimated_pose_topic", estimated_pose_str, "estimated_pose");
  private_nh.param<std::string>("controllers_topic", controllers_str, "command/high_level");
  private_nh.param<std::string>("rotation_angles_topic", rotation_angles_str, "rotation_angles");
  private_nh.param<std::string>("estimated_speed_topic",estimated_speed_str,"estimated_speed");
  private_nh.param<std::string>("yaw_controller_str",yaw_controller_str , "droneControllerYawRefCommand");
  private_nh.param<std::string>("service_topic_str",service_topic_str , "droneTrajectoryController/setControlMode");
  private_nh.param<std::string>("drone_position_str",drone_position_str , "dronePositionRefs");
  private_nh.param<std::string>("drone_yaw_to_look_str",yaw_to_look_str, "droneYawToLook");
  private_nh.param<std::string>("drone_yaw_ref",drone_yaw_ref_str,"droneControllerYawRefCommand");
  private_nh.param<std::string>("drone_control_mode",drone_control_mode_str,"droneTrajectoryController/controlMode");
  private_nh.param<std::string>("consult_belief",execute_query_srv,"consult_belief");
}

void BehaviorLookAtPoint::ownStart()
{
  std::cout << "ownStart" << std::endl;
  is_finished = false;

  /*Initialize topics*/
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorLookAtPoint::estimatedPoseCallBack, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorLookAtPoint::estimatedSpeedCallback, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorLookAtPoint::rotationAnglesCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  yaw_command_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(drone_yaw_ref_str,1000);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);

  estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));

  /*behavior implementation*/

  // Extract target yaw
  std::string arguments=getArguments();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["coordinates"].IsDefined()){
    std::vector<double> points=config_file["coordinates"].as<std::vector<double>>();
    target_position.x=points[0];
    target_position.y=points[1];
    target_position.z=points[2];
  }
  else{
    if(config_file["relative_coordinates"].IsDefined())
    {
      std::vector<double> points=config_file["relative_coordinates"].as<std::vector<double>>();
      target_position.x=estimated_pose_msg.x+points[0]+0.0001;
      target_position.y=estimated_pose_msg.y+points[1];
      target_position.z=estimated_pose_msg.z+points[2];
    }
    else{
      setStarted(false);
      return;
    }
  }
  if(config_file["angle"].IsDefined())
  {
    angle=config_file["angle"].as<double>()* M_PI/180;
  }
  else
  {
    angle=0;
    std::cout<<"Could not read angle! Angle set to default: "<<angle<<std::endl;
  }

  target_position.yaw=atan2(target_position.y-estimated_pose_msg.y,target_position.x-estimated_pose_msg.x)+angle;
  if(target_position.yaw > 2* M_PI || target_position.yaw < -2*M_PI)
  {
    target_position.yaw=target_position.yaw*180/M_PI;
    target_position.yaw= fmod(target_position.yaw,360);
    target_position.yaw=target_position.yaw*M_PI/180;
  }
  droneMsgsROS::setControlMode mode;
  mode.request.controlMode.command=mode.request.controlMode.POSITION_CONTROL;
  mode_service.call(mode);

  // Send reference to controller
  droneMsgsROS::dronePositionRefCommandStamped reference_position_msg;
  reference_position_msg.position_command.x = estimated_pose_msg.x;
  reference_position_msg.position_command.y = estimated_pose_msg.y;
  reference_position_msg.position_command.z = estimated_pose_msg.z;
  reference_position_msg.header.stamp = ros::Time::now();
  drone_position_pub.publish(reference_position_msg);

  // Wait for controller to enter POSITION mode
  ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
    drone_control_mode_str, node_handle
  );

  // Send target yaw
  droneMsgsROS::droneYawRefCommand yaw_msg;
  yaw_msg.yaw=target_position.yaw;
  yaw_command_pub.publish(yaw_msg);
  if (target_position.yaw<0)
  {
    target_position.yaw=2*M_PI + target_position.yaw;
  }
  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::MOVE;
  controllers_pub.publish(msg);
  static_pose.yaw=estimated_pose_msg.yaw;
}

void BehaviorLookAtPoint::ownRun()
{
  float angle_variation_maximum=0.1;
  if(!is_finished)
  {
   float actual_yaw=estimated_pose_msg.yaw;
   if (actual_yaw < 0)
   {
     actual_yaw=2*M_PI + actual_yaw;
   }
   if(std::abs(target_position.yaw-actual_yaw)<angle_variation_maximum)
   {
      BehaviorProcess::setFinishEvent(droneMsgsROS::BehaviorEvent::GOAL_ACHIEVED);
      BehaviorProcess::setFinishConditionSatisfied(true);
      is_finished = true;
      return;
   }
   if(timerIsFinished()){
      BehaviorProcess::setFinishEvent(droneMsgsROS::BehaviorEvent::TIME_OUT);
      BehaviorProcess::setFinishConditionSatisfied(true);
      is_finished = true;
      return;
   }
   /*if(staticity_timer.isFinished() && (static_pose.yaw-estimated_pose_msg.yaw)<angle_variation_maximum)
     {
      BehaviorProcess::setFinishEvent(droneMsgsROS::behaviorEvent::BLOCKED);
      BehaviorProcess::setFinishConditionSatisfied(true);
      is_finished = true;
      return;

     }
   if(target_position.yaw-estimated_pose_msg.yaw > angle_variation_maximum)
    {
      BehaviorProcess::setFinishEvent(droneMsgsROS::behaviorEvent::WRONG_PROGRESS);
      BehaviorProcess::setFinishConditionSatisfied(true);
      is_finished = true;
      return;

     }*/
  }


}
std::tuple<bool,std::string> BehaviorLookAtPoint::ownCheckSituation()
{
  droneMsgsROS::ConsultBelief query_service;
  std::ostringstream capturador;
  capturador << "battery_level(self,LOW)";
  std::string query(capturador.str());
  query_service.request.query = query;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Battery low, unable to perform action");
    //return false;
  }
  std::ostringstream capturador2;
  capturador2<<"flight_state(self,LANDED)";
  std::string query2(capturador2.str());
  query_service.request.query = query2;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Drone landed");
    //return false;
  }

  return std::make_tuple(true,"");
  //return true;
}
void BehaviorLookAtPoint::ownStop()
{
  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  controllers_pub.publish(msg);
  estimated_pose_sub.shutdown();
  rotation_angles_sub.shutdown();
  estimated_speed_sub.shutdown();
  // staticity_timer.stop();
}

void BehaviorLookAtPoint::estimatedSpeedCallback(const droneMsgsROS::droneSpeeds& msg)
{
estimated_speed_msg=msg;
}
void BehaviorLookAtPoint::estimatedPoseCallBack(const droneMsgsROS::dronePose& msg)
{
estimated_pose_msg=msg;
}
void BehaviorLookAtPoint::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg)
{
rotation_angles_msg=msg;
}
