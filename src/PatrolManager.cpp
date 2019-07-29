#include <PatrolManager.h>

using namespace std;

PatrolManager::PatrolManager(ros::NodeHandle &nh) {
  nh_ = nh;
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

PatrolManager::~PatrolManager() {}

bool PatrolManager::makeSpin(double radians, bool clockwise) {
  while (radians < 0)         //because of ros::tf drawbacks - you can call make spin method fiew times 
    radians += 2 * M_PI;
  while (radians > 2 * M_PI)
    radians -= 2 * M_PI;

  listener_.waitForTransform(
      "/base_link", "/odom", ros::Time(0),
      ros::Duration(1.0)); // wait for the listener to get the first message

  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;
  listener_.lookupTransform("/base_link", "/odom", ros::Time(0),
                            start_transform);

  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.75; // set apropirate velocity for robot
  if (clockwise)
    base_cmd.angular.z = -base_cmd.angular.z;

  tf::Vector3 desired_turn_axis(0, 0, 1); // the axis we want to be rotating by
  if (!clockwise) {
    desired_turn_axis = -desired_turn_axis; // changing direction of vector
  }
  ros::Rate rate(5.0);
  bool done = false;

  while (!done && nh_.ok()) {
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();

    try {
      listener_.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(1.0)); //prevent start oscilation to have impact 
      listener_.lookupTransform("/base_link", "/odom", ros::Time(0),
                                current_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      break;
    }
    // see how far we've traveled
    tf::Transform relative_transform =
        start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if (fabs(angle_turned) < 0.02) { //to prevent taking oscilations as rotation
      continue;
    }

    if (actual_turn_axis.dot(desired_turn_axis) < 0) {
      angle_turned = 2 * M_PI - angle_turned;
    }
    if (angle_turned > radians) {
      done = true;
    }
  }
  if (done) {
    return true;
  } else {
    return false;
  }
}

bool PatrolManager::moveToGoal(std::string the_name, float the_x, float the_y,
                               float the_theta) {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);

  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = the_x;
  goal.target_pose.pose.position.y = the_y;
  quaternion = quaternion_from_euler(0.0, 0.0, the_theta);
  goal.target_pose.pose.orientation.x = quaternion[0];
  goal.target_pose.pose.orientation.y = quaternion[1];
  goal.target_pose.pose.orientation.z = quaternion[2];
  goal.target_pose.pose.orientation.w = quaternion[3];

  ROS_INFO("Sending goal location x: %f ,y: %f ,th: %f ", the_x, the_y, the_theta);
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached it's destination and started scanning area");
    return true;
  } else {
    ROS_INFO("The robot failed to reach the destination check that");
    return false;
  }
}

std::vector<double>
PatrolManager::quaternion_from_euler(double roll, double pitch, double yaw) {

  std::vector<double> quat;

  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  quat.push_back(cy * cp * sr - sy * sp * cr); // x
  quat.push_back(sy * cp * sr + cy * sp * cr); // y
  quat.push_back(sy * cp * cr - cy * sp * sr); // z
  quat.push_back(cy * cp * cr + sy * sp * sr); // w

  return quat;
}
