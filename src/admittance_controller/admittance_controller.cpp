#include "admittance_controller/admittance_controller.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

admittance_controller::admittance_controller(   
    ros::NodeHandle &n, ros::Rate ros_rate,   
    std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber, std::string topic_tf_subscriber,
    std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher,
    std::string topic_compute_ik_client, std::string topic_compute_fk_client,
    std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix, 
    double maximum_velocity, double maximum_acceleration, 
    std::vector<double> workspace_limits):

    nh(n), loop_rate(ros_rate), mass_matrix(mass_model_matrix.data()), damping_matrix(damping_model_matrix.data()), ws_limits(workspace_limits.data()),
    max_vel(maximum_velocity), max_acc(maximum_acceleration) {

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

//-------------------------------------------------------- MAIN --------------------------------------------------------//


void admittance_controller::spinner (void) {

    ros::spinOnce();
    
    // trajectory_goal.trajectory = goal;
    // trajectory_client -> waitForServer();
    // trajectory_client -> sendGoal(trajectory_goal);
}
