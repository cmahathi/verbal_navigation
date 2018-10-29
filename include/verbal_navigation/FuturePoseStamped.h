#ifndef FUTURE_POSE_STAMPED
#define FUTURE_POSE_STAMPED

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class FuturePoseStamped {
	geometry_msgs::PoseStamped myPose;
	bool available;

public:
	FuturePoseStamped();

	void setFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	void setFromPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

	bool isAvailable();

	geometry_msgs::PoseStamped getPose();

	void reset();
};

#endif
