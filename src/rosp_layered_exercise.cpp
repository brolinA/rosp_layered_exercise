#include "rosp_layered_exercise/turtlebotGoalPublisher.hpp"

using namespace std::chrono_literals;

TurtleSimGoalPublisher::TurtleSimGoalPublisher() : Node("turtlesim_goal_publisher"), goal_id_(0)
{
  //load parameter and create data structure before creating the subscriber to avoid conflict
  LoadParams();

  velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/pose", 10, 
                      std::bind(&TurtleSimGoalPublisher::TurtlePoseCallback, this, std::placeholders::_1));
}

TurtleSimGoalPublisher::~TurtleSimGoalPublisher()
{
  //empty destructor
}

void TurtleSimGoalPublisher::LoadParams()
{
  //decalre the parameters
  declare_parameter("proportional_gain.linear", 0.5);
  declare_parameter("proportional_gain.angular", 0.5);
  declare_parameter("no_of_goal", 0);

  //reading parameters
  proportional_gain_linear_ = get_parameter("proportional_gain.linear").as_double();
  proportional_gain_angular_ = get_parameter("proportional_gain.angular").as_double();
  int num_of_goals = get_parameter("no_of_goal").as_int();

  //reading all goal points iteratively
  std::vector<std::vector<double>> goals;
  if(num_of_goals > 0)
  {
    for(int id=1; id<= num_of_goals; id++)
    {
      //declare parameters iteratively and read it.
      declare_parameter("goal"+std::to_string(id), std::vector<double>({0.0, 0.0, 0.0}));
      
      if(has_parameter("goal"+ std::to_string(id)))
        goals.push_back(get_parameter("goal"+ std::to_string(id)).as_double_array());
      else
        RCLCPP_ERROR(this->get_logger(), "Goal ID: %d is missing.", id);
    }
  }

  //convert data vector to internal structure
  ConvertDataStructure(goals);
  RCLCPP_INFO(this->get_logger(), "Loaded all parameters");
}

void TurtleSimGoalPublisher::TurtlePoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
  //check if all goal points have been reached.
  if(goal_id_ == (int)turtlesim_goals_.size())
  {
    RCLCPP_INFO(this->get_logger(), "Reached all points. Shutting down...");
    rclcpp::shutdown();
  }
  else //exectute the movements
    ControlLoop(ToGeometryPose(*pose));
}

void TurtleSimGoalPublisher::ConvertDataStructure(std::vector<std::vector<double>> pts)
{
  for (auto pt:pts)
  {
    //check if the goal point size is correct
    if(pt.size() != 3){
      RCLCPP_ERROR(this->get_logger(), "Skipping point. Goal dimension doesn't match. Got size %d, but required size is\
      3" , pt.size());
      continue;
    }
    
    //convert to custom structure for easy handling
    GoalData goal_pt;
    goal_pt.pose.x = pt[0];
    goal_pt.pose.y = pt[1];
    goal_pt.pose.theta = pt[2];
    turtlesim_goals_.push_back(goal_pt);
  }
}

geometry_msgs::msg::Pose2D TurtleSimGoalPublisher::ToGeometryPose(turtlesim::msg::Pose pose)
{
  geometry_msgs::msg::Pose2D ret_pose;
  ret_pose.x = pose.x;
  ret_pose.y = pose.y;
  ret_pose.theta = pose.theta;

  return ret_pose;
}

bool TurtleSimGoalPublisher::AlignAngle(geometry_msgs::msg::Pose2D current_pose, geometry_msgs::msg::Pose2D target_pose,
                                       float tolerance)
{ 
  double angle_to_goal = std::atan2(current_pose.y - target_pose.y, current_pose.x - target_pose.x);
  double angle_error = angle_to_goal - target_pose.theta;
  
  if (angle_error > M_PI)
    angle_error -= 2 * M_PI;
  if (angle_error < -M_PI)
    angle_error += 2 * M_PI;

  geometry_msgs::msg::Twist cmd;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), kLoggingRate, "Angle to goal: %lf Current angle: %lf\
  Angle error: %lf", angle_to_goal, target_pose.theta, std::abs(angle_error));
  
  if(std::abs(angle_error) > tolerance)
  {
    cmd.angular.z = proportional_gain_angular_ * angle_error;
    velocity_pub_->publish(cmd);

    //return false if the error is higher than tolerance
    return false;
  }
  else
    return true;
}

bool TurtleSimGoalPublisher::MoveToGoal(geometry_msgs::msg::Pose2D current_pose, geometry_msgs::msg::Pose2D target_pose,
                                       float tolerance)
{
  geometry_msgs::msg::Twist cmd;
  double dx = target_pose.x - current_pose.x;
  double dy = target_pose.y - current_pose.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  double direction = std::atan2(dy, dx);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), kLoggingRate,  "Distance to goal: %lf Current x: %lf,\
  Current y: %lf, Direction: %lf", distance, current_pose.x, current_pose.y, direction);
  
  if (distance > tolerance)
  {
    cmd.linear.x = proportional_gain_linear_ * distance * std::cos(direction - current_pose.theta);
    velocity_pub_->publish(cmd);
    return false;
  }
  else
    return true;
}

void TurtleSimGoalPublisher::ControlLoop(geometry_msgs::msg::Pose2D current_turtle_pose)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Moving to goal ID: %d", goal_id_);
  if(!turtlesim_goals_[goal_id_].isComplete)
  {
    //Step 1: Align with goal
    if(!AlignAngle(turtlesim_goals_[goal_id_].pose, current_turtle_pose, 0.01))
      return;

    // Step 2: Move to goal
    if(!MoveToGoal(current_turtle_pose, turtlesim_goals_[goal_id_].pose, 0.1))
      return;
    
    // Step 3: Align with goal orientation
    if(!AlignAngle(turtlesim_goals_[goal_id_].pose, current_turtle_pose, 0.01))
      return;
    
    //update the goal state and ID after completing the goal
    turtlesim_goals_[goal_id_].isComplete = true;
    goal_id_++;
  }
}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSimGoalPublisher>());
  rclcpp::shutdown();
  return 0;
}
