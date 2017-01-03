#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower_nite/FollowerConfig.h"

namespace turtlebot_follower_nite{
class TurtlebotFollowerNite
{
public:
  
  TurtlebotFollowerNite(ros::NodeHandle& nh);
	~TurtlebotFollowerNite();

    priv_nh.param<double>("goal_x", goal_x_ 0.6);
    priv_nh.param<double>("z_scale", z_scale_,5.0);
    priv_nh.param<double>("x_scale", x_scale_,1.0);
    priv_nh.param<bool>("enabled", enabled_, true);
    priv_nh.param<double>("loop_hz",loop_hz_, 10)

    cmdpub_ = priv_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollowerNite::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower_nite::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower_nite::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollowerNite::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);

  }

  ~TurtlebotFollowerNite()
  {
    delete config_srv_;
  }

private:
  double goal_x_; /**< The distance away from the robot to hold the torso frame */
  double x_scale_; /**< The scaling factor for translational robot speed */
  double z_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower_nite::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */

  void reconfigure(turtlebot_follower_nite::FollowerConfig &config, uint32_t level)
  {
    goal_z_ = config.goal_z;
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
  void update()
  {
	ros::Rate rate(loop_hz_);
  	while (ros::ok())
	{

		try{
		      listener.lookupTransform("/torso", "/base_link",
				               ros::Time(0), transform);
		    }
		catch (tf::TransformException &ex) {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		      continue;
		    }
		
		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = z_scale_ * atan2(transform.getOrigin().y(),
		                            transform.getOrigin().x());
		vel_msg.linear.x = x_scale_ * (sqrt(pow(transform.getOrigin().x(), 2) +
		                          pow(transform.getOrigin().y(), 2))-goal_x_);
		if (enabled_){
			cmdpub_.publish(vel_msg);
		}

	    	rate.sleep();
	}

  }

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

  ros::init(argc, argv, "follower_nite");
  ros::NodeHandle nh;

  TurtlebotFollowerNite follower(nh);

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

}
