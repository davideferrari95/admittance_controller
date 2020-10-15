#include "admittance_controller/admittance_controller.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

admittance_controller::admittance_controller(   
    ros::NodeHandle &n, ros::Rate ros_rate,   
    std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber,
    std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher,
    std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix, 
    std::vector<double> workspace_limits, std::vector<double> joint_limits,
    std::vector<double> maximum_velocity, std::vector<double> maximum_acceleration):

    nh(n), loop_rate(ros_rate), mass_matrix(mass_model_matrix.data()), damping_matrix(damping_model_matrix.data()), 
    workspace_lim(workspace_limits.data()), joint_lim(joint_limits.data()), max_vel(maximum_velocity.data()), max_acc(maximum_acceleration.data()) {

    // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/admittance_controller_Node/use_feedback_velocity", use_feedback_velocity, false)) {ROS_ERROR("Couldn't retrieve the Feedback Velocity value.");}

    // ---- ROS SUBSCRIBERS ---- //
    force_sensor_subscriber = nh.subscribe(topic_force_sensor_subscriber, 1, &admittance_controller::force_sensor_Callback, this);
    joint_states_subscriber = nh.subscribe(topic_joint_states_subscriber, 1, &admittance_controller::joint_states_Callback, this);
    
    // ---- ROS PUBLISHERS ---- //
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(topic_joint_trajectory_publisher, 1);
    joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_joint_group_vel_controller_publisher, 1);

    // ---- ROS ACTIONS ---- //
    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(topic_action_trajectory_publisher, true);

    // Initializing the Class Variables
    external_wrench.setZero();
    x_dot.setZero();
    q_dot.setZero();

    force_callback = false;
    joint_state_callback = false;
    first_cycle = true;

    // ---- MoveIt Robot Model ---- //
    robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    joint_names = joint_model_group->getJointModelNames();

    // ---- DEBUG PRINT ---- //
    // ROS_INFO_STREAM("Mass Matrix:" << std::endl << std::endl << mass_matrix << std::endl);
    // ROS_INFO_STREAM("Damping Matrix:" << std::endl << std::endl << damping_matrix << std::endl);

}

admittance_controller::~admittance_controller() {}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void admittance_controller::force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &msg) {

    geometry_msgs::WrenchStamped force_sensor = *msg;

    external_wrench[0] = force_sensor.wrench.force.x;
    external_wrench[1] = force_sensor.wrench.force.y;
    external_wrench[2] = force_sensor.wrench.force.z;
    external_wrench[3] = force_sensor.wrench.torque.x;
    external_wrench[4] = force_sensor.wrench.torque.y;
    external_wrench[5] = force_sensor.wrench.torque.z;
    
    // std::cout << "Force   ->  x: " << external_wrench[0] << " y: " << external_wrench[1] << " z: " << external_wrench[2] << std::endl;
    // std::cout << "Torque  ->  x: " << external_wrench[3] << " y: " << external_wrench[4] << " z: " << external_wrench[5] << std::endl; 

    for (int i = 0; i < external_wrench.size(); i++) {if(fabs(external_wrench[i]) < 5.0) {external_wrench[i] = 0.0;}}
    
    force_callback = true;

}

void admittance_controller::joint_states_Callback (const sensor_msgs::JointState::ConstPtr &msg) {

    joint_state = *msg;

    joint_position.resize(6);
    joint_velocity.resize(6);
    for (unsigned int i = 0; i < joint_state.name.size(); i++) {joint_position[i] = joint_state.position[i];}
    for (unsigned int i = 0; i < joint_state.name.size(); i++) {joint_velocity[i] = joint_state.velocity[i];}

    joint_state_callback = true;
    
}


//----------------------------------------------------- FUNCTIONS ------------------------------------------------------//


Eigen::Matrix4d admittance_controller::compute_fk (std::vector<double> joint_position) {

    // Set the Real Value of the Joint to the Kinematic Model of Moveit!
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");

    // Compute the pose of the end-effector with respect to the /prbt_base frame
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();

    //Transformation Matrix
    Eigen::Matrix4d transformation_matrix;

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation_matrix.setIdentity();

    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation_vector;

    return transformation_matrix;

}

Eigen::MatrixXd admittance_controller::compute_arm_jacobian (std::vector<double> joint_position) {

    // Set the Real Value of the Joint to the Kinematic Model of Moveit!
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->enforceBounds();

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);

    return jacobian;

}

void admittance_controller::compute_admittance (void) {

    ros::spinOnce();

    Vector6d arm_desired_accelaration;

    if (use_feedback_velocity) {

        Vector6d joint_velocity_eigen = Eigen::Map<Vector6d>(joint_velocity.data());

        // Compute Cartesian Velocity
        J = compute_arm_jacobian(joint_position);
        x_dot = J * joint_velocity_eigen;
        // ROS_INFO_STREAM_ONCE("Manipulator Jacobian: " << std::endl << std::endl << J << std::endl);
        // ROS_INFO_STREAM_ONCE("First Velocity: " << std::endl << std::endl << x_dot << std::endl);
    
    }

    // Compute Acceleration with Admittance
    arm_desired_accelaration = mass_matrix.inverse() * ( - damping_matrix * x_dot + external_wrench);

    // Limiting the Accelaration for better stability and safety
    double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

    if (a_acc_norm > max_acc[0]) {
        ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!" << " norm: " << a_acc_norm);
        arm_desired_accelaration.segment(0, 3) *= (max_acc[0] / a_acc_norm);
    }

    // Integrate for Velocity Based Interface
    ros::Duration duration = loop_rate.expectedCycleTime();
    x_dot  += arm_desired_accelaration * duration.toSec();

    // Limiting Velocity of the arm along x, y, and z axis
    double norm_vel_des = (x_dot.segment(0, 3)).norm();

    if (norm_vel_des > max_vel[0]) {
        ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);
        x_dot.segment(0, 3) *= (max_vel[0] / norm_vel_des);
    }

    // Inverse Kinematic for Joint Velocity
    q_dot = J.inverse() * x_dot;

    ROS_INFO_STREAM_THROTTLE(2, "Desired Velocity: " << q_dot);

}

void admittance_controller::sending_velocity_to_robot (Vector6d velocity) {

    std_msgs::Float64MultiArray msg;

    std::vector<double> velocity_vector(velocity.data(), velocity.data() + velocity.size());
    // ROS_INFO("Velocity: %.2f %.2f %.2f %.2f %.2f %.2f", velocity_vector[0], velocity_vector[1], velocity_vector[2], velocity_vector[3], velocity_vector[4], velocity_vector[5]);

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = velocity.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "velocity";

    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), velocity_vector.begin(), velocity_vector.end());

    joint_group_vel_controller_publisher.publish(msg);

    // trajectory_goal.trajectory = goal;
    // trajectory_client -> waitForServer();
    // trajectory_client -> sendGoal(trajectory_goal);

}

void admittance_controller::wait_for_init (void) {

    // Wait for the Callbacks
    while (!force_callback || !joint_state_callback) {

        ros::spinOnce();
        
        if (!force_callback) {ROS_WARN_THROTTLE(2, "Wait for Force Sensor");}
        if (!joint_state_callback) {ROS_WARN_THROTTLE(2, "Wait for Joint State feedback");}
    
    }

    // TODO: Wait for Robot Model 

}



//-------------------------------------------------------- MAIN --------------------------------------------------------//


void admittance_controller::spinner (void) {

    if (first_cycle) {

        wait_for_init();
        ros::spinOnce();

        first_cycle = false;
    
    } else {

        compute_admittance();
        sending_velocity_to_robot(q_dot);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
