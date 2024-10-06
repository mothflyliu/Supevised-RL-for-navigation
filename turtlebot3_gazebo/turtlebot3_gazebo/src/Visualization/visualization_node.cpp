#ifndef VISUALIZATION_NODE_H_
#define VISUALIZATION_NODE_H_

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "nav_msgs/Path.h"
#include "turtlebot3_msgs/VisualReset.h"

class visualization
{
    public:
        visualization(ros::NodeHandle& nh);
        ~visualization();
        void OdomCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
        void PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path);
        void VisualizationReset(const turtlebot3_msgs::VisualReset::ConstPtr &msg);

    private:
        /* data */
        int pose_size_;
        std::vector<geometry_msgs::PoseStamped> path_;
        nav_msgs::Path vis_path_;
        ros::Subscriber odom_sub_;
        ros::Subscriber reset_sub_;
	bool reset;
        int count;
        ros::Publisher visualization_pub_;
};

visualization::visualization(ros::NodeHandle& nh)
{
    odom_sub_ = nh.subscribe<gazebo_msgs::ModelStates>(
        "/gazebo/model_states", 1, &visualization::OdomCallback,
        this);

    reset_sub_ = nh.subscribe<turtlebot3_msgs::VisualReset>(
        "visual_reset", 1, &visualization::VisualizationReset,
        this);
    visualization_pub_ = nh.advertise<nav_msgs::Path>("vis_path", 10);
    vis_path_.header.frame_id = "odom";
    pose_size_ = 1e10;
    reset = false;
}

visualization::~visualization() = default;

void visualization::OdomCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){
    if(reset && count<500){
	count ++;
	return;
    }
    reset = false; 
    count = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.orientation = msg->pose[4].orientation;
    pose.pose.position = msg->pose[4].position;
    if(path_.size() > pose_size_){
        path_.erase(path_.begin());
    }
    path_.push_back(pose);
    PathVisualization(path_);
}

void visualization::PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path){
    vis_path_.poses = path;
    visualization_pub_.publish(vis_path_);
}

void visualization::VisualizationReset(const turtlebot3_msgs::VisualReset::ConstPtr &msg){
    if(msg->reset){
        path_.clear();
	reset = true;
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualization_node");
  ros::NodeHandle nh;
  visualization vis(nh);
  ros::spin();
  return 0;
}


#endif // VISUALIZATION_NODE_H_
