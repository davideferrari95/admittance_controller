#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "ros/ros.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "tf2_msgs/TFMessage.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetPositionFK.h"

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class admittance_controller {

    public:

        admittance_controller( 
            ros::NodeHandle &n, ros::Rate ros_rate,   
            std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber, std::string topic_tf_subscriber,
            std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher, 
            std::string topic_compute_ik_client, std::string topic_compute_fk_client,
            std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix, 
            double maximum_velocity, double maximum_acceleration, 
            std::vector<double> workspace_limits);

        ~admittance_controller();

        void spinner (void);

    private:

        ros::NodeHandle nh;
        ros::Rate loop_rate;

        Matrix6d mass_matrix, damping_matrix;
        Vector6d ws_limits;
        double max_vel, max_acc;

        ros::Subscriber force_sensor_subscriber, joint_states_subscriber, tf_subscriber;
        ros::Publisher  joint_trajectory_publisher, joint_group_vel_controller_publisher;

        ros::ServiceClient compute_ik_client, compute_fk_client;
        moveit_msgs::GetPositionIK compute_ik_srv;
        moveit_msgs::GetPositionFK compute_fk_srv;
        
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_client;
		control_msgs::FollowJointTrajectoryGoal trajectory_goal;

        sensor_msgs::JointState joint_state;
        geometry_msgs::WrenchStamped force_sensor;
        tf2_msgs::TFMessage tf;

        void force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &);
        void joint_states_Callback (const sensor_msgs::JointState::ConstPtr &);
        void tf_Callback (const tf2_msgs::TFMessage::ConstPtr &);


};

#endif /* ADMITTANCE_CONTROLLER_H */