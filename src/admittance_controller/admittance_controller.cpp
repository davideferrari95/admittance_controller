#include "admittance_controller/admittance_controller.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

admittance_controller::admittance_controller(   
    ros::NodeHandle &n, ros::Rate ros_rate,   
    std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber,
    std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher,
    std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix,
    double force_dead_zone, double torque_dead_zone, double admittance_weight,
    std::vector<double> workspace_limits, std::vector<double> joint_limits,
    std::vector<double> maximum_velocity, std::vector<double> maximum_acceleration):

    nh(n), loop_rate(ros_rate), mass_matrix(mass_model_matrix.data()), damping_matrix(damping_model_matrix.data()), 
    force_dead_zone(force_dead_zone), torque_dead_zone(torque_dead_zone), admittance_weight(admittance_weight),
    workspace_lim(workspace_limits.data()), joint_lim(joint_limits.data()), max_vel(maximum_velocity.data()), max_acc(maximum_acceleration.data()) {

    // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/admittance_controller_Node/use_feedback_velocity", use_feedback_velocity, false)) {ROS_ERROR("Couldn't retrieve the Feedback Velocity value.");}
    if (!nh.param<bool>("/admittance_controller_Node/use_ur_real_robot", use_ur_real_robot, false)) {ROS_ERROR("Couldn't retrieve the Use Real Robot value.");}

    // ---- ROS SUBSCRIBERS ---- //
    force_sensor_subscriber = nh.subscribe(topic_force_sensor_subscriber, 1, &admittance_controller::force_sensor_Callback, this);
    joint_states_subscriber = nh.subscribe(topic_joint_states_subscriber, 1, &admittance_controller::joint_states_Callback, this);
    
    // ---- ROS PUBLISHERS ---- //
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(topic_joint_trajectory_publisher, 1);
    joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_joint_group_vel_controller_publisher, 1);

    // ---- ROS ACTIONS ---- //
    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(topic_action_trajectory_publisher, true);

    // Initializing the Class Variables
    joint_position.resize(6);
    joint_velocity.resize(6);
    external_wrench.setZero();
    joint_velocity_old.setZero();
    x_dot.setZero();
    q_dot.setZero();

    force_callback = false;
    joint_state_callback = false;

    // ---- MoveIt Robot Model ---- //
    robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    joint_names = joint_model_group->getJointModelNames();

    // ---- DEBUG PRINT ---- //
    std::cout << std::endl;
    ROS_INFO_STREAM_ONCE("Mass Matrix:" << std::endl << std::endl << mass_matrix << std::endl);
    ROS_INFO_STREAM_ONCE("Damping Matrix:" << std::endl << std::endl << damping_matrix << std::endl);
    ROS_INFO_ONCE("Force Dead Zone: %.2f", force_dead_zone);
    ROS_INFO_ONCE("Troque Dead Zone: %.2f", torque_dead_zone);
    ROS_INFO_ONCE("Admittance Weight: %.2f \n", admittance_weight);


    // ---- WAIT FOR INITIALIZATION ---- //
    wait_for_callbacks_initialization();

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
    
    ROS_DEBUG_THROTTLE(2, "Sensor Force  ->  x: %.2f  y: %.2f  z: %.2f", external_wrench[0], external_wrench[1], external_wrench[2]);
    ROS_DEBUG_THROTTLE(2, "Sensor Torque ->  x: %.2f  y: %.2f  z: %.2f", external_wrench[3], external_wrench[4], external_wrench[5]); 

    for (int i = 0; i < 3; i++) {if(fabs(external_wrench[i]) < fabs(force_dead_zone)) {external_wrench[i] = 0.0;}}
    for (int i = 3; i < 6; i++) {if(fabs(external_wrench[i]) < fabs(torque_dead_zone)) {external_wrench[i] = 0.0;}}
    
    force_callback = true;

}

void admittance_controller::joint_states_Callback (const sensor_msgs::JointState::ConstPtr &msg) {

    joint_state = *msg;

    // Ur10e Real Robot has Inverted Joints
    if (use_ur_real_robot) {

        std::swap(joint_state.name[0], joint_state.name[2]);
        std::swap(joint_state.effort[0], joint_state.effort[2]);
        std::swap(joint_state.position[0], joint_state.position[2]);
        std::swap(joint_state.velocity[0], joint_state.velocity[2]);

    }

    for (unsigned int i = 0; i < joint_state.position.size(); i++) {joint_position[i] = joint_state.position[i];}
    for (unsigned int i = 0; i < joint_state.velocity.size(); i++) {joint_velocity[i] = joint_state.velocity[i];}

    ROS_DEBUG_THROTTLE(2, "joint position: %.2f %.2f %.2f %.2f %.2f %.2f", joint_position[0], joint_position[1], joint_position[2], joint_position[3], joint_position[4], joint_position[5]);

    joint_state_callback = true;
    
}


//----------------------------------------------------- FUNCTIONS ------------------------------------------------------//


Eigen::Matrix4d admittance_controller::compute_fk (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    // Get the Translation Vector and Rotation Matrix
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();

    //Transformation Matrix
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix.setZero();

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation_matrix.setIdentity();

    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation_vector;

    return transformation_matrix;

}

Eigen::MatrixXd admittance_controller::compute_arm_jacobian (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    // TODO: controllare che joint_model_group->getLinkModelNames().back()) corrisponda a tool0 o ee_link
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);

    ROS_DEBUG_STREAM_THROTTLE(2, "Manipulator Jacobian: " << std::endl << std::endl << J << std::endl);
    ROS_DEBUG_STREAM_THROTTLE(2, "Manipulator Inverse Jacobian: " << std::endl << std::endl << J.inverse() << std::endl);

    return jacobian;

}

Matrix6d admittance_controller::get_ee_rotation_matrix (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    // Rotation Matrix 6x6
    Matrix6d rotation_matrix;
    rotation_matrix.setZero();

    rotation_matrix.topLeftCorner(3, 3) = end_effector_state.rotation();
    rotation_matrix.bottomRightCorner(3, 3) = end_effector_state.rotation();

    Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(0, 1, 2);
    Eigen::Quaterniond rotation_quaternion(end_effector_state.rotation());

    // -- DEBUG OUTPUT -- //
    ROS_DEBUG_THROTTLE(2, "Translation Vector   ->   X: %.3f  Y: %.3f  Z: %.3f", end_effector_state.translation().x(), end_effector_state.translation().y(), end_effector_state.translation().z());
    ROS_DEBUG_THROTTLE(2, "Euler Angles         ->   R: %.3f  P: %.3f  Y: %.3f", euler_angles[0], euler_angles[1], euler_angles[2]);
    ROS_DEBUG_THROTTLE(2, "Rotation Quaternion  ->   X: %.3f  Y: %.3f  Z: %.3f  W: %.3f", rotation_quaternion.x(), rotation_quaternion.y(), rotation_quaternion.z(), rotation_quaternion.w());

    ROS_DEBUG_STREAM_THROTTLE(2, "Rotation Matrix from Model:" << std::endl << std::endl << end_effector_state.rotation() << std::endl);
    ROS_DEBUG_STREAM_THROTTLE(2, "Rotation Matrix 6x6:" << std::endl << std::endl << rotation_matrix << std::endl);

    return rotation_matrix;

}

void admittance_controller::compute_admittance (void) {

    ros::spinOnce();

    // Compute Manipulator Jacobian
    J = compute_arm_jacobian(joint_position, joint_velocity);

    if (use_feedback_velocity) {

        Vector6d joint_velocity_eigen = Eigen::Map<Vector6d>(joint_velocity.data());

        // Compute Cartesian Velocity
        x_dot = J * joint_velocity_eigen;
        ROS_INFO_STREAM_ONCE("Start Velocity: " << std::endl << std::endl << x_dot << std::endl);
    
    } else {
        
        // Use the Cartesian Speed obtained the last cycle
        x_dot = x_dot;
        
    }

    // Compute Acceleration with Admittance //FIXME: sensore di forza non allineato alla posizione dell'ee
    Vector6d arm_desired_accelaration_cartesian = mass_matrix.inverse() * ( - damping_matrix * x_dot + admittance_weight * 
                                                  (get_ee_rotation_matrix(joint_position, joint_velocity) * external_wrench));

    // Integrate for Velocity Based Interface
    ros::Duration duration = loop_rate.expectedCycleTime();
    ROS_INFO_STREAM_ONCE("Cycle Time: " << duration.toSec()*1000 << " ms");
    x_dot  += arm_desired_accelaration_cartesian * duration.toSec();

    // Inverse Kinematic for Joint Velocity
    q_dot = J.inverse() * x_dot;

    limit_joint_dynamics(&q_dot);

    ROS_INFO_THROTTLE(2, "Desired Cartesian Velocity:  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f", x_dot[0], x_dot[1], x_dot[2], x_dot[3], x_dot[4], x_dot[5]);
    ROS_INFO_THROTTLE(2, "Desired  Joints   Velocity:  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f", q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5]);

}

void admittance_controller::send_velocity_to_robot (Vector6d velocity) {

    std_msgs::Float64MultiArray msg;

    std::vector<double> velocity_vector(velocity.data(), velocity.data() + velocity.size());

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = velocity.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "velocity";

    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), velocity_vector.begin(), velocity_vector.end());

    joint_group_vel_controller_publisher.publish(msg);

}

void admittance_controller::wait_for_callbacks_initialization (void) {

    ros::Duration(1).sleep();

    // Wait for the Callbacks
    while (ros::ok() && (!force_callback || !joint_state_callback)) {

        ros::spinOnce();
        
        if (!force_callback) {ROS_WARN_THROTTLE(3, "Wait for Force Sensor");}
        if (!joint_state_callback) {ROS_WARN_THROTTLE(3, "Wait for Joint State feedback");}

    }

}

void admittance_controller::limit_joint_dynamics(Vector6d *joint_velocity) {

    Vector6d joint_vel = *joint_velocity;
    double duration = loop_rate.expectedCycleTime().toSec();

    // Limit Joint Velocity

    for (int i = 0; i < joint_vel.size(); i++) {

        if (fabs(joint_vel[i]) > max_vel[i]) {

            ROS_DEBUG("Reached Maximum Velocity on Joint %d", i);
            joint_vel[i] = sign(joint_vel[i]) * max_vel[i];

        }

    }

    // Limit Joint Acceleration

    Vector6d delta_vel = joint_vel - joint_velocity_old;

    for (int i = 0; i < delta_vel.size(); i++) {

        if (fabs(delta_vel[i]) > max_acc[i] * duration) {
            ROS_DEBUG("Reached Maximum Acceleration on Joint %d", i);
            joint_vel[i] = joint_velocity_old[i] + sign(joint_vel[i]) * max_acc[i] * duration;
        }

    }

    joint_velocity = &joint_vel;
    joint_velocity_old = joint_vel;

}

int admittance_controller::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void admittance_controller::spinner (void) {

    ros::spinOnce();

    compute_admittance();
    send_velocity_to_robot(q_dot);

    // Matrix6d a = get_ee_rotation_matrix(joint_position, joint_velocity);

    ros::spinOnce();
    loop_rate.sleep();
    
}
