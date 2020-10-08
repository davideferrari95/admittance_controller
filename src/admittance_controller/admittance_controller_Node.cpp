#include "admittance_controller/admittance_controller.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "admittance_controller_Node");

    ros::NodeHandle nh;
    ros::Rate loop_rate = 100;

    // Parameters
    std::string topic_force_sensor_subscriber, topic_joint_states_subscriber, topic_tf_subscriber;
    std::string topic_joint_trajectory_publisher, topic_action_trajectory_publisher, topic_joint_group_vel_controller_publisher;
    std::string topic_compute_ik_client, topic_compute_fk_client;

    std::vector<double> mass_model_matrix, damping_model_matrix, workspace_limits;
    double maximum_velocity, maximum_acceleration;

    // ---- LOADING "TOPIC NAME" PARAMETERS FROM THE ROS SERVER ---- //
    if (!nh.param<std::string>("/admittance_controller_Node/topic_force_sensor", topic_force_sensor_subscriber, "/wrench")) {ROS_ERROR("Couldn't retrieve the Force Sensor Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_joint_states", topic_joint_states_subscriber, "/joint_states")) {ROS_ERROR("Couldn't retrieve the Jiont States Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_tf", topic_tf_subscriber, "/tf")) {ROS_ERROR("Couldn't retrieve the TF Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_joint_trajectory", topic_joint_trajectory_publisher, "/scaled_pos_joint_traj_controller/command")) {ROS_ERROR("Couldn't retrieve the Trajectory Publisher Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_action_trajectory", topic_action_trajectory_publisher, "/scaled_pos_joint_traj_controller/follow_joint_trajectory")) {ROS_ERROR("Couldn't retrieve the Action Publisher Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_joint_group_vel_controller", topic_joint_group_vel_controller_publisher, "/joint_group_vel_controller/command")) {ROS_ERROR("Couldn't retrieve the Velocity Controller Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_compute_ik_client", topic_compute_ik_client, "/compute_ik")) {ROS_ERROR("Couldn't retrieve the Compute IK Topic's name.");}
    if (!nh.param<std::string>("/admittance_controller_Node/topic_compute_fk_client", topic_compute_fk_client, "/compute_fk")) {ROS_ERROR("Couldn't retrieve the Compute FK Topic's name.");}

    // ---- LOADING "ADMITTANCE" PARAMETERS FROM THE ROS SERVER ---- //
    if (!nh.getParam("/admittance_controller_Node/mass_matrix", mass_model_matrix)) {ROS_ERROR("Couldn't retrieve the Mass Matrix parameter.");}
    if (!nh.getParam("/admittance_controller_Node/damping_matrix", damping_model_matrix)) {ROS_ERROR("Couldn't retrieve the Damping Matrix parameter.");}
    
    // ---- LOADING "SAFETY" PARAMETERS FROM THE ROS SERVER ---- //
    if (!nh.getParam("/admittance_controller_Node/workspace_limits", workspace_limits)) {ROS_ERROR("Couldn't retrieve the Workspace Limits parameter.");}
    if (!nh.param("/admittance_controller_Node/maximum_velocity", maximum_velocity, 1.0)) {ROS_ERROR("Couldn't retrieve the Maximum Velocity parameter.");}
    if (!nh.param("/admittance_controller_Node/maximum_acceleration", maximum_acceleration, 1.0)) {ROS_ERROR("Couldn't retrieve the Maximum Acceleration parameter.");}
    
    admittance_controller ac (
        nh, loop_rate, topic_force_sensor_subscriber, topic_joint_states_subscriber, topic_tf_subscriber,
        topic_joint_trajectory_publisher, topic_action_trajectory_publisher, topic_joint_group_vel_controller_publisher,
        topic_compute_ik_client, topic_compute_fk_client,
        mass_model_matrix, damping_model_matrix, maximum_velocity, maximum_acceleration, workspace_limits);

    while (ros::ok()) {

        ac.spinner();
        ros::shutdown();

    }

return 0;

}
