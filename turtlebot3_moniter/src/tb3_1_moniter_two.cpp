#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Eigen>

ros::Subscriber box_state_sub;
geometry_msgs::Pose pose;
Eigen::Vector4d current_state_(-3.0,0.0,0.0,0.0);
Eigen::Vector4d last_state_;
double trajecty_sum_;

void StatesCallback(const gazebo_msgs::ModelStates &box_state_current)
{
    int box_index = -1;
    std::vector<std::string> model_names = box_state_current.name;
    for(size_t i = 0; i < model_names.size(); i++)
    {
        if(model_names[i] == "tb3_1")
            box_index = i;
    }
    pose = box_state_current.pose[box_index];
    //ROS_INFO_STREAM("Position:" << pose.position);

    last_state_ = current_state_;
    current_state_(0) =   pose.position.x;
    current_state_(1) =   pose.position.y;
    current_state_(2) =   pose.orientation.z;
    current_state_(3) =   pose.position.z;
    
    trajecty_sum_ += std::sqrt(std::pow(current_state_(0) - last_state_(0),2)+std::pow(current_state_(1) - last_state_(1),2));
    ROS_INFO("traval distance of tb3_1: %f !", trajecty_sum_);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb3_1_moniter_two_node");
  ros::NodeHandle node_handle;

  ros::Subscriber box_states_sub = node_handle.subscribe("/gazebo/model_states", 1, StatesCallback);
  
  ros::spin();
  return 0;
}

