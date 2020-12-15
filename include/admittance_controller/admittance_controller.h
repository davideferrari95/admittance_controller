#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <signal.h>
#include <fstream>
#include <numeric>
#include <math.h>

#include "spline_interpolation/spline.h" /* https://kluge.in-chemnitz.de/opensource/spline/ */

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include "admittance_controller/joint_trajectory.h"
#include "admittance_controller/parameter_srv.h"

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Eigen>

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Array<double, 6, 1> Array6d;

#define GET_VARIABLE_NAME(Variable) (#Variable)

class admittance_control {

    public:

        admittance_control( 
            ros::NodeHandle &n, ros::Rate ros_rate,   
            std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber,
            std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher, 
            std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix, 
            double force_dead_zone, double torque_dead_zone, double admittance_weight, std::vector<double> joint_limits,
            std::vector<double> maximum_velocity, std::vector<double> maximum_acceleration);

        ~admittance_control();

        void spinner (void);

        bool simple_debug;
        bool complete_debug;

    private:

        ros::NodeHandle nh;
        ros::Rate loop_rate;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- Admittance Parameters ---- //
        Matrix6d mass_matrix, damping_matrix;
        double force_dead_zone, torque_dead_zone, admittance_weight;
        bool use_feedback_velocity, use_ur_real_robot, inertia_reduction;
        
        // ---- Admittance IO ---- //
        Vector6d external_wrench, ftsensor_start_offset, x_dot, q_dot;
        Vector6d q_dot_last_cycle, x_dot_last_cycle;
        
        // ---- Limits ---- //
        Vector6d joint_lim, max_vel, max_acc;

        // ---- MoveIt Robot Model ---- //
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr kinematic_model;
        robot_state::RobotStatePtr kinematic_state;
        const robot_state::JointModelGroup *joint_model_group;
        std::vector<std::string> joint_names;
        Eigen::MatrixXd J;

        // ---- Trajectory Execution ---- //
        admittance_controller::joint_trajectory desired_trajectory;

        // ---- Feedback Variables ---- //
        bool force_callback, joint_state_callback;
        sensor_msgs::JointState joint_state;
        std::vector<double> joint_position, joint_velocity;
        std::vector<Vector6d> filter_elements;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- PUBLISHERS & SUBSCRIBERS ---- //
        ros::Subscriber force_sensor_subscriber, joint_states_subscriber, trajectory_execution_subscriber;
        ros::Publisher joint_trajectory_publisher, joint_group_vel_controller_publisher, ur10e_script_command_publisher;

        // ---- ROS SERVICE CLIENTS ---- //
        ros::ServiceClient switch_controller_client, list_controllers_client, zero_ft_sensor_client;
        controller_manager_msgs::SwitchController switch_controller_srv;
        controller_manager_msgs::ListControllers list_controllers_srv;
        std_srvs::Trigger zero_ft_sensor_srv;
        
        // ---- ROS SERVICE SERVERS ---- //
        ros::ServiceServer ur10e_freedrive_mode_service, admittance_controller_activation_service, change_admittance_parameters_service;
        bool admittance_control_request, freedrive_mode_request, trajectory_execution_request;

        // ---- ROS ACTIONS ---- //
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_client;
        control_msgs::FollowJointTrajectoryGoal trajectory_goal;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- CALLBACKS ---- //
        void force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &);
        void joint_states_Callback (const sensor_msgs::JointState::ConstPtr &);
        void trajectory_execution_Callback (const admittance_controller::joint_trajectory::ConstPtr &);

        // ---- SERVER CALLBACKS ---- //
        bool FreedriveMode_Service_Callback (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool Admittance_Controller_Activation_Service_Callback (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool Change_Admittance_Parameters_Service_Callback (admittance_controller::parameter_srv::Request &req, admittance_controller::parameter_srv::Response &res);

        // ---- KINEMATIC MODEL FUNCTIONS ---- //
        Eigen::Matrix4d compute_fk (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Eigen::MatrixXd compute_arm_jacobian (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Matrix6d get_ee_rotation_matrix (std::vector<double> joint_position, std::vector<double> joint_velocity);

        // ---- ADMITTANCE FUNCTIONS ---- //
        void compute_admittance (void);

        // ---- LIMIT DYNAMIC FUNCTIONS ---- //
        Vector6d limit_joint_dynamics (Vector6d joint_velocity);
        Vector6d compute_inertia_reduction (Vector6d velocity, Vector6d wrench);

        // ---- TRAJECTORY FUNCTIONS ---- //
        void trajectory_execution (admittance_controller::joint_trajectory desired_trajectory);
        void stop_robot (void);
        sensor_msgs::JointState add_stop_point (std::vector<sensor_msgs::JointState> *trajectory);

        // ---- TRAJECTORY SCALING ---- //
        std::vector<sensor_msgs::JointState> trajectory_scaling (admittance_controller::joint_trajectory trajectory);
        void check_requested_scaling (admittance_controller::joint_trajectory trajectory, bool *no_scaling_requested, bool *velocity_scaling_requested);
        double compute_scaled_velocity (admittance_controller::joint_trajectory trajectory, double s_dot_rec);
        std::vector<double> compute_s_des (double s_dot_des, double trajectory_time, double sampling_time, std::vector<std::string> extra_data);
        std::vector<Vector6d> compute_desired_positions (std::vector<double> s_des, std::vector<tk::spline> q_spline6d);
        std::vector<Vector6d> compute_desired_velocities (std::vector<Vector6d> q_des, double sampling_time);
        std::vector<sensor_msgs::JointState> create_scaled_trajectory (std::vector<sensor_msgs::JointState> input_trajectory, std::vector<Vector6d> q_des, std::vector<Vector6d> q_dot_des, double sampling_time);

        // ---- SPLINE INTERPOLATION ---- //
        std::vector<tk::spline> spline_interpolation (std::vector<Vector6d> data_vector, double spline_lenght, std::string output_file);
        std::vector<tk::spline> spline_interpolation (std::vector<Array6d> data_vector, double spline_lenght, std::string output_file);
        Vector6d get_spline_value (std::vector<tk::spline> spline6d, double s);
        
        // ---- CONTROL FUNCTIONS ---- //
        void send_velocity_to_robot (Vector6d velocity);
        void send_position_to_robot (Vector6d position);
        void wait_for_position_reached (Vector6d desired_position, double maximum_time);
        void freedrive_mode (bool activation);

        // ---- USEFUL FUNCTIONS ---- //
        Vector6d low_pass_filter(Vector6d input_vec);
        void wait_for_callbacks_initialization (void);
        Vector6d compute_ftsensor_starting_offset (void);
        int sign (double num);

        // ---- DEBUG ---- //
        std::ofstream ft_sensor_debug;
        void csv_debug (std::vector<double> vector, std::string name);
        void csv_debug (std::vector<Vector6d> vector6d, std::string name);
        void csv_debug (std::vector<tk::spline> spline6d, std::vector<double> s, std::vector<Vector6d> data_vector, std::string name);
        void trajectory_debug_csv (std::vector<sensor_msgs::JointState> trajectory, std::string trajectory_name);

};

#endif /* ADMITTANCE_CONTROLLER_H */
