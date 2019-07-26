#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <vector>

using namespace std;
class PatrolManager
{
public:
    PatrolManager(ros::NodeHandle &nh);
    ~PatrolManager();
    bool moveToGoal(std::string name, float x, float y ,float theta);
    bool makeSpin(double radians, bool clockwise = 1);


private:
    std::vector<double> quaternion_from_euler(double yaw, double pitch,double roll);
    std::vector<double> quaternion;
    
    ros::NodeHandle nh_;  //The node handle we'll be using
    ros::Publisher cmd_vel_pub_; //We will be publishing to the "cmd_vel" topic to issue commands
    tf::TransformListener listener_; //We will be listening to TF transforms 
     
};

