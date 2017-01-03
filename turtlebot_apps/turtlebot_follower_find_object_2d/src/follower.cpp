/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower_find_object_2d/FollowerConfig.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

	

class TurtlebotFollowerFindObject2D
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotFollowerFindObject2D(ros::NodeHandle& nh) 
  :	nh_(nh),
	priv_nh_("~")
  {
    priv_nh_.param<bool>("navigation", navigation,false);
    priv_nh_.param<double>("goal_x", goal_x_,2.0);
    priv_nh_.param<double>("z_scale", z_scale_,5.0);
    priv_nh_.param<double>("x_scale", x_scale_,1.0);
    priv_nh_.param<bool>("enabled", enabled_, true);
    priv_nh_.param<double>("loop_hz",loop_hz_, 10);

    cmdpub_ = priv_nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    switch_srv_ = priv_nh_.advertiseService("change_state", &TurtlebotFollowerFindObject2D::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower_find_object_2d::FollowerConfig>(priv_nh_);
    dynamic_reconfigure::Server<turtlebot_follower_find_object_2d::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollowerFindObject2D::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);

    ac = new MoveBaseClient ("move_base", true);
    while(!ac->waitForServer(ros::Duration(5.0))){
 	ROS_INFO("Waiting for the move_base action server to come up");
    }
  }

  ~TurtlebotFollowerFindObject2D()
  {
    delete config_srv_;
  }

  void update()
  {
	ros::Rate rate(loop_hz_);
	geometry_msgs::Twist vel_msg;
	move_base_msgs::MoveBaseGoal goal;
	float x;
	float y;
	
  	while (ros::ok())
	{

		try{
		      listener.lookupTransform("/base_link", "/object_7", 
				               ros::Time(0), transform);
			ROS_WARN("x transform :%f, y transform :%f, z transform :%f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z() );
			x=transform.getOrigin().x();
			y=transform.getOrigin().y();
			

			if (navigation){
				goal.target_pose.header.frame_id = "base_link";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = x;
				goal.target_pose.pose.position.y=y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (std::atan2(y,x));
				ac->sendGoal(goal);		
			}

			else{
				
				vel_msg.angular.z = z_scale_ * atan2(y, x);
				vel_msg.linear.x = x_scale_ * (sqrt(pow(x, 2) + pow(y, 2))-goal_x_);
				if (enabled_){
					cmdpub_.publish(vel_msg);
				}
			}

		    	rate.sleep();
		    }
		catch (tf::TransformException &ex) {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		      continue;
		    }

	}

  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  double goal_x_; /**< The distance away from the robot to hold the torso frame */
  double x_scale_; /**< The scaling factor for translational robot speed */
  double z_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */
  double loop_hz_;
  bool navigation;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  MoveBaseClient *ac;

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower_find_object_2d::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */

  void reconfigure(turtlebot_follower_find_object_2d::FollowerConfig &config, uint32_t level)
  {
    goal_x_ = config.goal_x;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /*!
   * @brief Callback for point clouds.
   * Callback for depth images. It finds the centroid
   * of the points in a box in the center of the image. 
   * Publishes cmd_vel messages with the goal from the image.
   * @param cloud The point cloud message.
   */

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  ros::Publisher cmdpub_;
};

	




int main(int argc, char** argv){

  ros::init(argc, argv, "follower_find_object_2d");
  ros::NodeHandle nh;

  TurtlebotFollowerFindObject2D follower(nh);

  try
  {
    follower.update();
  }
  catch (std::runtime_error& ex)
  {
    ROS_FATAL_STREAM("[TurtlebotFollower] Runtime error: " << ex.what());
    return 1;
  }

  return 0;

    
  }


