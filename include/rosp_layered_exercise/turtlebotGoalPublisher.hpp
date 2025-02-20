#include <chrono>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleSimGoalPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Turtle Sim Goal Publisher object
     */
    TurtleSimGoalPublisher();

    /**
     * @brief Destroy the Turtle Sim Goal Publisher object
     */
    ~TurtleSimGoalPublisher();

private:

    /**
     * @brief Structure to hold goal information
     */
    struct GoalData
    {
        geometry_msgs::msg::Pose2D pose; //pose of the goal points (x, y, theta in radian)
        bool isComplete = false; //indicate if the goal is reached or not
    };
    
    int kLoggingRate = 2000; //milliseconds
    double proportional_gain_angular_ = 5.0;   
    double proportional_gain_linear_ = 2.0;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_; //command velocity publisher
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_; //subscriber to position
    std::vector<GoalData> turtlesim_goals_;
    int goal_id_;

    //Functions
    /**
     * @brief Function to load the parameters from yaml file
     * @return none
     */
    void LoadParams();

    /**
     * @brief Function to exectue control action
     * 
     * @param current_turtle_pose current position of the turtle from simulation
     * @return none
     */
    void ControlLoop(geometry_msgs::msg::Pose2D current_turtle_pose);

    /**
     * @brief Callback function for position callback
     * 
     * @param pose current turtle pose
     * @return none
     */
    void TurtlePoseCallback(const turtlesim::msg::Pose::SharedPtr pose);

    /**
     * @brief Function to convert vector data into internal structure for easy access
     * 
     * @param pts Vector of all the goal points to reach
     * @return none
     */
    void ConvertDataStructure(std::vector<std::vector<double>> pts);

    /**
     * @brief Utility function to convert turtlesim pose to geometry pose
     * 
     * @param pose turtle's pose
     * @return geometry_msgs::msg::Pose2D converted pose to geometry message type
     */
    geometry_msgs::msg::Pose2D ToGeometryPose (turtlesim::msg::Pose pose);

    /**
     * @brief Function to align turtle to the target angle
     * 
     * @param current_pose current pose containing the current angle
     * @param target_pose target pose containing the target angle
     * @param tolerance tolerance value for angle error
     * @return true if angle error<tolerance
     * @return false otherwise
     */
    bool AlignAngle(geometry_msgs::msg::Pose2D current_pose, geometry_msgs::msg::Pose2D target_pose, float tolerance);

    /**
     * @brief Function to move turtle to the target position
     * 
     * @param current_pose current pose containing the current position
     * @param target_pose target pose containing the target position
     * @param tolerance tolerance value for distance error
     * @return true if distance error<tolerance
     * @return false otherwise
     */
    bool MoveToGoal(geometry_msgs::msg::Pose2D current_pose, geometry_msgs::msg::Pose2D target_pose, float tolerance);
};
