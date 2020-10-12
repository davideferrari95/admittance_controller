#include "admittance_controller/admittance_controller.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

admittance_controller::admittance_controller(   
    ros::NodeHandle &n, ros::Rate ros_rate,   
    std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber, std::string topic_tf_subscriber,
    std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher,
    std::string topic_compute_ik_client, std::string topic_compute_fk_client,
    std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix, 
    std::vector<double> workspace_limits, std::vector<double> joint_limits,
    std::vector<double> maximum_velocity, std::vector<double> maximum_acceleration):

    nh(n), loop_rate(ros_rate), mass_matrix(mass_model_matrix.data()), damping_matrix(damping_model_matrix.data()), 
    workspace_lim(workspace_limits.data()), joint_lim(joint_limits.data()), max_vel(maximum_velocity.data()), max_acc(maximum_acceleration.data()) {

    // ---- ROS SUBSCRIBERS ---- //
    force_sensor_subscriber = nh.subscribe(topic_force_sensor_subscriber, 1, &admittance_controller::force_sensor_Callback, this);
    joint_states_subscriber = nh.subscribe(topic_joint_states_subscriber, 1, &admittance_controller::joint_states_Callback, this);
    tf_subscriber = nh.subscribe(topic_tf_subscriber, 1, &admittance_controller::tf_Callback, this);
    
    // ---- ROS PUBLISHERS ---- //
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(topic_joint_trajectory_publisher, 1);
    joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_joint_group_vel_controller_publisher, 1);

    // ---- ROS ACTIONS ---- //
    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(topic_action_trajectory_publisher, true);

    // ---- ROS SERVICES ---- //
    compute_ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>(topic_compute_ik_client);
    compute_fk_client = nh.serviceClient<moveit_msgs::GetPositionFK>(topic_compute_fk_client);

    // Initializing the Class Variables
    external_wrench.setZero();
    arm_desired_twist.setZero();

    // -- MoveIt Robot Model -- //
    robot_model_loader = new robot_model_loader::RobotModelLoader ("robot_description");

    

}

admittance_controller::~admittance_controller() {}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void admittance_controller::force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &msg) {

    force_sensor = *msg;

}

void admittance_controller::joint_states_Callback (const sensor_msgs::JointState::ConstPtr &msg) {

    joint_state = *msg;

}

void admittance_controller::tf_Callback (const tf2_msgs::TFMessage::ConstPtr &msg) {

    tf = *msg;

}


//----------------------------------------------------- FUNCTIONS ------------------------------------------------------//



//-------------------------------------------------------- MAIN --------------------------------------------------------//


void admittance_controller::spinner (void) {

    ros::spinOnce();
    
    // trajectory_goal.trajectory = goal;
    // trajectory_client -> waitForServer();
    // trajectory_client -> sendGoal(trajectory_goal);
}
