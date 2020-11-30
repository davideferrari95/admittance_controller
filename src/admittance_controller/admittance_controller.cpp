#include "admittance_controller/admittance_controller.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

admittance_control::admittance_control(   
    ros::NodeHandle &n, ros::Rate ros_rate,   
    std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber,
    std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher,
    std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix,
    double force_dead_zone, double torque_dead_zone, double admittance_weight,
    std::vector<double> joint_limits, std::vector<double> maximum_velocity, std::vector<double> maximum_acceleration):

    nh(n), loop_rate(ros_rate), mass_matrix(mass_model_matrix.data()), damping_matrix(damping_model_matrix.data()), 
    force_dead_zone(force_dead_zone), torque_dead_zone(torque_dead_zone), admittance_weight(admittance_weight),
    joint_lim(joint_limits.data()), max_vel(maximum_velocity.data()), max_acc(maximum_acceleration.data()) {

    // ---- LOAD PARAMETERS ---- //
    if (!nh.param<bool>("/admittance_controller_Node/use_feedback_velocity", use_feedback_velocity, false)) {ROS_ERROR("Couldn't retrieve the Feedback Velocity value.");}
    if (!nh.param<bool>("/admittance_controller_Node/inertia_reduction", inertia_reduction, false)) {ROS_ERROR("Couldn't retrieve the Inertia Reduction value.");}
    if (!nh.param<bool>("/admittance_controller_Node/use_ur_real_robot", use_ur_real_robot, false)) {ROS_ERROR("Couldn't retrieve the Use Real Robot value.");}
    
    // ---- ROS PUBLISHERS ---- //
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(topic_joint_trajectory_publisher, 1);
    joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_joint_group_vel_controller_publisher, 1);
    ur10e_script_command_publisher = nh.advertise<std_msgs::String>("/ur_hardware_interface/script_command",1);
    
    // ---- ROS SUBSCRIBERS ---- //
    force_sensor_subscriber = nh.subscribe(topic_force_sensor_subscriber, 1, &admittance_control::force_sensor_Callback, this);
    joint_states_subscriber = nh.subscribe(topic_joint_states_subscriber, 1, &admittance_control::joint_states_Callback, this);
    trajectory_execution_subscriber = nh.subscribe("/admittance_controller/trajectory_execution", 1, &admittance_control::trajectory_execution_Callback, this);
    
    // ---- ROS SERVICE SERVERS ---- //
    ur10e_freedrive_mode_service = nh.advertiseService("/admittance_controller/ur10e_freedrive_mode_service", &admittance_control::FreedriveMode_Service_Callback, this);
    admittance_controller_activation_service = nh.advertiseService("/admittance_controller/admittance_controller_activation_service", &admittance_control::Admittance_Controller_Activation_Service_Callback, this);

    // ---- ROS SERVICE CLIENTS ---- //
    switch_controller_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    list_controllers_client  = nh.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
    zero_ft_sensor_client    = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/zero_ftsensor");

    // ---- ROS ACTIONS ---- //
    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(topic_action_trajectory_publisher, true);

    // Initializing the Class Variables
    joint_position.resize(6);
    joint_velocity.resize(6);
    external_wrench.setZero();
    x_dot.setZero();
    q_dot.setZero();
    x_dot_last_cycle.setZero();
    q_dot_last_cycle.setZero();

    force_callback = false;
    joint_state_callback = false;
    freedrive_mode_request = false;
    admittance_control_request = true;
    simple_debug = true;
    complete_debug = false;

    // ---- MoveIt Robot Model ---- //
    robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    joint_names = joint_model_group->getJointModelNames();

    // ---- DEBUG PRINT ---- //
    if (simple_debug) {
        std::cout << std::endl;
        ROS_INFO_STREAM_ONCE("Mass Matrix:" << std::endl << std::endl << mass_matrix << std::endl);
        ROS_INFO_STREAM_ONCE("Damping Matrix:" << std::endl << std::endl << damping_matrix << std::endl);
        ROS_INFO_ONCE("Maximum Velocity:     %.2f %.2f %.2f %.2f %.2f %.2f", max_vel[0], max_vel[1], max_vel[2], max_vel[3], max_vel[4], max_vel[5]);
        ROS_INFO_ONCE("Maximum Acceleration: %.2f %.2f %.2f %.2f %.2f %.2f \n", max_acc[0], max_acc[1], max_acc[2], max_acc[3], max_acc[4], max_acc[5]);
        ROS_INFO_ONCE("Force Dead Zone:   %.2f", force_dead_zone);
        ROS_INFO_ONCE("Troque Dead Zone:  %.2f", torque_dead_zone);
        ROS_INFO_ONCE("Admittance Weight: %.2f", admittance_weight);
        ROS_INFO_ONCE("Inertia Reduction: %s", inertia_reduction ? "true" : "false");
        ROS_INFO_STREAM_ONCE("Cycle Time: " << loop_rate.expectedCycleTime().toSec()*1000 << " ms" << std::endl);
    }
    
    // ---- DEBUG ---- //
    if (complete_debug) {
        std::string package_path = ros::package::getPath("admittance_controller");
        ROS_INFO_STREAM_ONCE("Package Path:  " << package_path << std::endl);
        std::string save_file = package_path + "/debug/ft_sensor.csv";
        ft_sensor_debug = std::ofstream(save_file);
    }
    
    // ---- ZERO FT SENSOR ---- //
    while (use_ur_real_robot && !zero_ft_sensor_client.call(zero_ft_sensor_srv)) {ROS_WARN_THROTTLE(2,"Wait for Service: \"/ur_hardware_interface/zero_ftsensor\"");}
    if (zero_ft_sensor_client.call(zero_ft_sensor_srv)) ROS_INFO("Succesful Request \"zero_ftsensor\"\n");
    
    // ---- WAIT FOR INITIALIZATION ---- //
    wait_for_callbacks_initialization();
    std::cout << std::endl;

}

admittance_control::~admittance_control() {ft_sensor_debug.close();}


//--------------------------------------------------- TOPICS CALLBACK ---------------------------------------------------//


void admittance_control::force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &msg) {

    geometry_msgs::WrenchStamped force_sensor = *msg;

    external_wrench[0] = force_sensor.wrench.force.x;
    external_wrench[1] = force_sensor.wrench.force.y;
    external_wrench[2] = force_sensor.wrench.force.z;
    external_wrench[3] = force_sensor.wrench.torque.x;
    external_wrench[4] = force_sensor.wrench.torque.y;
    external_wrench[5] = force_sensor.wrench.torque.z;
    
    // ---- DEBUG ---- //
    if (complete_debug) {
        for (int i = 0; i < 6; i++) {ft_sensor_debug << external_wrench[i] << ",";}
        ft_sensor_debug << "\n";
    }

    // ---- DEBUG ---- //
    if (complete_debug) ROS_INFO_THROTTLE_NAMED(2, "FTSensor", "Sensor Force/Torque  ->  Fx: %.2f  Fy: %.2f  Fz: %.2f  |  Tx: %.2f  Ty: %.2f  Tz: %.2f", external_wrench[0], external_wrench[1], external_wrench[2], external_wrench[3], external_wrench[4], external_wrench[5]);

    for (int i = 0; i < 3; i++) {if(fabs(external_wrench[i]) < fabs(force_dead_zone)) {external_wrench[i] = 0.0;}}
    for (int i = 3; i < 6; i++) {if(fabs(external_wrench[i]) < fabs(torque_dead_zone)) {external_wrench[i] = 0.0;}}
    
    // ---- DEBUG ---- //
    if (complete_debug) ROS_INFO_THROTTLE_NAMED(2, "FTSensor", "Sensor Force/Torque Clamped  ->  Fx: %.2f  Fy: %.2f  Fz: %.2f  |  Tx: %.2f  Ty: %.2f  Tz: %.2f", external_wrench[0], external_wrench[1], external_wrench[2], external_wrench[3], external_wrench[4], external_wrench[5]);
    
    // LowPass Filter
    external_wrench = low_pass_filter(external_wrench);

    force_callback = true;

}

void admittance_control::joint_states_Callback (const sensor_msgs::JointState::ConstPtr &msg) {

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

    // ---- DEBUG ---- //
    if (complete_debug) ROS_INFO_THROTTLE_NAMED(2, "Joint Position", "joint position: %.2f %.2f %.2f %.2f %.2f %.2f", joint_position[0], joint_position[1], joint_position[2], joint_position[3], joint_position[4], joint_position[5]);

    joint_state_callback = true;
    
}

void admittance_control::trajectory_execution_Callback (const admittance_controller::joint_trajectory::ConstPtr &msg) {

    ROS_WARN("Trajectory Execution Callback");

    freedrive_mode_request = false;
    admittance_control_request = false;
    
    std::vector<sensor_msgs::JointState> desired_trajectory = trajectory_scaling(*msg);

    trajectory_execution(desired_trajectory);

    admittance_control_request = true;
    ROS_WARN("Restart Admittance Control\n");

}


//-------------------------------------------------- SERVICES CALLBACK --------------------------------------------------//


bool admittance_control::FreedriveMode_Service_Callback (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

    freedrive_mode_request = req.data;

    freedrive_mode(freedrive_mode_request);
    
    res.success = true;
    return true;

}

bool admittance_control::Admittance_Controller_Activation_Service_Callback (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    
    // Activate / Deactivate Admittance Controller
    admittance_control_request = req.data;

    if (admittance_control_request) {

        // Deactivate FreeDrive Mode if Active
        freedrive_mode_request = false;
        freedrive_mode(freedrive_mode_request);
    
    }

    res.success = true;
    return true;

}


//------------------------------------------------- KINEMATIC FUNCTIONS -------------------------------------------------//


Eigen::Matrix4d admittance_control::compute_fk (std::vector<double> joint_position, std::vector<double> joint_velocity) {

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

Eigen::MatrixXd admittance_control::compute_arm_jacobian (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;

    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);

    // ---- DEBUG ---- //
    if (complete_debug) ROS_INFO_STREAM_THROTTLE_NAMED(2, "Manipulator Jacobian", "Manipulator Jacobian: " << std::endl << std::endl << jacobian << std::endl);
    if (complete_debug) ROS_INFO_STREAM_THROTTLE_NAMED(2, "Manipulator Inverse Jacobian", "Manipulator Inverse Jacobian: " << std::endl << std::endl << jacobian.inverse() << std::endl);

    return jacobian;

}

Matrix6d admittance_control::get_ee_rotation_matrix (std::vector<double> joint_position, std::vector<double> joint_velocity) {

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

    // ---- DEBUG ---- //
    if (complete_debug) {

        ROS_INFO_THROTTLE_NAMED(2, "Translation Vector",  "Translation Vector   ->   X: %.3f  Y: %.3f  Z: %.3f", end_effector_state.translation().x(), end_effector_state.translation().y(), end_effector_state.translation().z());
        ROS_INFO_THROTTLE_NAMED(2, "Euler Angles",        "Euler Angles         ->   R: %.3f  P: %.3f  Y: %.3f", euler_angles[0], euler_angles[1], euler_angles[2]);
        ROS_INFO_THROTTLE_NAMED(2, "Rotation Quaternion", "Rotation Quaternion  ->   X: %.3f  Y: %.3f  Z: %.3f  W: %.3f", rotation_quaternion.x(), rotation_quaternion.y(), rotation_quaternion.z(), rotation_quaternion.w());

        ROS_INFO_STREAM_THROTTLE_NAMED(2, "Rotation Matrix from Model", "Rotation Matrix from Model:" << std::endl << std::endl << end_effector_state.rotation() << std::endl);
        ROS_INFO_STREAM_THROTTLE_NAMED(2, "Rotation Matrix 6x6",        "Rotation Matrix 6x6:" << std::endl << std::endl << rotation_matrix << std::endl);
    }

    return rotation_matrix;

}


//------------------------------------------------- ADMITTANCE FUNCTION -------------------------------------------------//


void admittance_control::compute_admittance (void) {

    ros::spinOnce();

    // Compute Manipulator Jacobian
    J = compute_arm_jacobian(joint_position, joint_velocity);

    if (use_feedback_velocity) {

        Vector6d joint_velocity_eigen = Eigen::Map<Vector6d>(joint_velocity.data());

        // Compute Cartesian Velocity
        x_dot = J * joint_velocity_eigen;
        
        // ---- DEBUG ---- //
        if (complete_debug) {ROS_INFO_STREAM_ONCE_NAMED("Start Velocity", "Start Velocity: " << std::endl << std::endl << x_dot << std::endl);}
    
    } else {
        
        // Use the Cartesian Speed obtained the last cycle
        x_dot = x_dot_last_cycle;
        
    }

    // Compute Acceleration with Admittance
    Vector6d arm_desired_accelaration_cartesian = mass_matrix.inverse() * ( - damping_matrix * x_dot + admittance_weight * 
                                                  (get_ee_rotation_matrix(joint_position, joint_velocity) * external_wrench));

    // Integrate for Velocity Based Interface
    ros::Duration duration = loop_rate.expectedCycleTime();
    // ROS_INFO_STREAM_ONCE("Cycle Time: " << duration.toSec()*1000 << " ms");
    x_dot  += arm_desired_accelaration_cartesian * duration.toSec();

    // Inertia Reduction Function
    if (inertia_reduction) {x_dot = compute_inertia_reduction(x_dot, external_wrench);}
    
    // Inverse Kinematic for Joint Velocity
    q_dot = J.inverse() * x_dot;

    // Limit System Dynamic
    q_dot = limit_joint_dynamics(q_dot);
    x_dot_last_cycle = J * q_dot;

    // ---- DEBUG ---- //
    if (complete_debug) {ROS_INFO_THROTTLE_NAMED(2, "Desired Cartesian Velocity", "Desired Cartesian Velocity:  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f", x_dot[0], x_dot[1], x_dot[2], x_dot[3], x_dot[4], x_dot[5]);}
    if (complete_debug) {ROS_INFO_THROTTLE_NAMED(2, "Desired Joints Velocity",    "Desired Joints Velocity:     %.2f  %.2f  %.2f  %.2f  %.2f  %.2f", q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5]);}

}


//----------------------------------------------- LIMIT DYNAMICS FUNCTIONS ----------------------------------------------//


Vector6d admittance_control::limit_joint_dynamics (Vector6d joint_velocity) {

    double duration = loop_rate.expectedCycleTime().toSec();

    // Limit Joint Velocity

    for (int i = 0; i < joint_velocity.size(); i++) {

        if (fabs(joint_velocity[i]) > max_vel[i]) {

            // ---- DEBUG ---- //
            if (complete_debug) ROS_INFO_NAMED("Reached Maximum Velocity", "Reached Maximum Velocity on Joint %d   ->   Velocity: %.3f   Limited at: %.3f", i, joint_velocity[i], sign(joint_velocity[i]) * max_vel[i]);
            joint_velocity[i] = sign(joint_velocity[i]) * max_vel[i];

        }

    }

    // Limit Joint Acceleration

    for (int i = 0; i < joint_velocity.size(); i++) {

        if (fabs(joint_velocity[i] - q_dot_last_cycle[i]) > max_acc[i] * duration) {

            // ---- DEBUG ---- //
            if (complete_debug) ROS_INFO_NAMED("Reached Maximum Acceleration", "Reached Maximum Acceleration on Joint %d   ->   Acceleration: %.3f   Limited at: %.3f", i, (joint_velocity[i] - q_dot_last_cycle[i]) / duration, q_dot_last_cycle[i] +  sign(joint_velocity[i] - q_dot_last_cycle[i]) * max_acc[i]);
            joint_velocity[i] = q_dot_last_cycle[i] + sign(joint_velocity[i] - q_dot_last_cycle[i]) * max_acc[i] * duration;
        
        }

    }

    q_dot_last_cycle = joint_velocity;
    return joint_velocity;

}
   
std::vector<sensor_msgs::JointState> admittance_control::limit_joint_dynamics (std::vector<sensor_msgs::JointState> trajectory) {

    // ---- LIMIT JOINTS DYNAMIC ---- //
    for (unsigned int i = 1; i < trajectory.size(); i++) {

        // Compute Sampling Time
        double sampling_time = (trajectory[i].header.stamp.toSec() - trajectory[i-1].header.stamp.toSec());

        for (int joint_n = 0; joint_n < 6; joint_n++) {
            
            // Limit Joint Velocity
            if (fabs(trajectory[i].velocity[joint_n]) > max_vel[joint_n]) {

                ROS_DEBUG("Reached Maximum Velocity on Joint %d   ->   Velocity: %.3f   Limited at: %.3f", joint_n, trajectory[i].velocity[joint_n], sign(trajectory[i].velocity[joint_n]) * max_vel[joint_n]);
                trajectory[i].velocity[joint_n] = sign(trajectory[i].velocity[joint_n]) * max_vel[joint_n];

            }
            
            // Limit Joint Acceleration
            if (fabs(trajectory[i].velocity[joint_n] - trajectory[i-1].velocity[joint_n]) > max_acc[joint_n] * sampling_time) {

                ROS_DEBUG("Reached Maximum Acceleration on Joint %d   ->   Acceleration: %.3f   Limited at: %.3f", joint_n, (trajectory[i].velocity[joint_n] - trajectory[i-1].velocity[joint_n]) / sampling_time, trajectory[i-1].velocity[joint_n] +  sign(trajectory[i].velocity[joint_n] - trajectory[i-1].velocity[joint_n]) * max_acc[joint_n]);
                trajectory[i].velocity[joint_n] = trajectory[i-1].velocity[joint_n] + sign(trajectory[i].velocity[joint_n] - trajectory[i-1].velocity[joint_n]) * max_acc[joint_n] * sampling_time;

            }

        }
        
    }

    // ---- DEBUG ---- //
    if (simple_debug) {trajectory_debug_csv(trajectory,"scaled_limit_dynamics_trajectory");}

    return trajectory;

}

Vector6d admittance_control::compute_inertia_reduction (Vector6d velocity, Vector6d wrench) {

    Array6d reduction, x_vel_array(velocity);

    // Returns 0 if the Wrench is 0, 1 otherwise
    for (unsigned i = 0; i < wrench.size(); i++) {
        
        if (wrench[i] == 0.0) {reduction[i] = 0;}
        else {reduction[i] = 1;}
    
    }

    return Vector6d(x_vel_array *= reduction);

}


//------------------------------------------------ TRAJECTORY FUNCTIONS -------------------------------------------------//


void admittance_control::trajectory_execution (std::vector<sensor_msgs::JointState> trajectory) {

    // Move Robot to Poin 0 (Position Controller)
    Vector6d starting_position(trajectory[0].position.data());
    send_position_to_robot(starting_position);

    // Creation of a Stop-Point (zero velocity) in the end of the trajectory
    sensor_msgs::JointState stop_point = add_stop_point(&trajectory);

    for (unsigned i = 0; i < trajectory.size() - 1; i++) {

        // Print Trajectory Execution Percentage
        ROS_INFO_STREAM_THROTTLE(5, "Trajectory Execution Status: " << i * 100 / int(trajectory.size() - 1) << "%");

        // Compute Trajectory Rate (Point [i+1] - Point [i])
        double trajectory_rate = (trajectory[i+1].header.stamp - trajectory[i].header.stamp).toSec();

        // Command robot in velocity
        send_velocity_to_robot(Vector6d(trajectory[i].velocity.data()));
        
        // ---- DEBUG ---- //
        if (complete_debug) {ROS_INFO_NAMED("Trajectory Execution Joint Names", "Trajectory Execution Joint Names: %s %s %s %s %s %s", trajectory[i].name[0].c_str(), trajectory[i].name[1].c_str(), trajectory[i].name[2].c_str(), trajectory[i].name[3].c_str(), trajectory[i].name[4].c_str(), trajectory[i].name[5].c_str());}
        if (complete_debug) {ROS_INFO_NAMED("Trajectory Execution Velocity", "Trajectory Execution Velocity: %.5f %.5f %.5f %.5f %.5f %.5f", trajectory[i].velocity[0], trajectory[i].velocity[1], trajectory[i].velocity[2], trajectory[i].velocity[3], trajectory[i].velocity[4], trajectory[i].velocity[5]);}

        // Sleep
        ros::Duration(trajectory_rate).sleep();

    }

    ROS_WARN("Trajectory Completed\n");

}

sensor_msgs::JointState admittance_control::add_stop_point (std::vector<sensor_msgs::JointState> *trajectory) {

    std::vector<sensor_msgs::JointState> trajectory_temp = *trajectory;

    // Creation of a Stop-Point (zero velocity) in the end of the trajectory
    sensor_msgs::JointState stop_point;
    stop_point.name = trajectory_temp[0].name;
    stop_point.position = trajectory_temp[trajectory_temp.size()-1].position;
    std::fill(stop_point.velocity.begin(), stop_point.velocity.end(), 0);
    stop_point.header.stamp = trajectory_temp[trajectory_temp.size()-1].header.stamp + (trajectory_temp[1].header.stamp  - trajectory_temp[0].header.stamp);

    // Add Stop-Point in the end of the trajectory
    trajectory_temp.push_back(stop_point);
    trajectory = &trajectory_temp;

    return stop_point;

}

void admittance_control::trajectory_debug_csv (std::vector<sensor_msgs::JointState> trajectory, std::string trajectory_name) {

    std::string package_path = ros::package::getPath("admittance_controller");
    std::string save_file = package_path + "/debug/" + trajectory_name + "_debug.csv";
    std::ofstream trajectory_debug = std::ofstream(save_file);

    trajectory_debug << "frame_id,seq,sec,nsec,     ,pos_joint1,pos_joint2,pos_joint2,pos_joint4,pos_joint5,pos_joint6,     ,vel_joint1,vel_joint2,vel_joint3,vel_joint4,vel_joint5,vel_joint6\n\n";
    
    for (unsigned int i = 0; i < trajectory.size(); i++) {
    
        trajectory_debug << trajectory[i].header.frame_id << "," << trajectory[i].header.seq << "," << trajectory[i].header.stamp.sec << "," << trajectory[i].header.stamp.nsec << ", ,";
    
        for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {trajectory_debug << trajectory[i].position[joint_n] << ",";} trajectory_debug << " ,";
    
        for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {trajectory_debug << trajectory[i].velocity[joint_n] << ",";} trajectory_debug << "\n";
    
    }
    
    trajectory_debug.close();
        
}


//------------------------------------------------- TRAJECTORY SCALING --------------------------------------------------//


std::vector<sensor_msgs::JointState> admittance_control::trajectory_scaling (admittance_controller::joint_trajectory trajectory) {

    std::vector<sensor_msgs::JointState> input_trajectory = trajectory.trajectory, scaled_trajectory;
    
    // ---- DEBUG ---- //
    if (simple_debug) {trajectory_debug_csv(input_trajectory,"input_trajectory");}
    
    // Percentage of Scaling [%]
    int velocity_scaling_percentage = trajectory.velocity_scaling_percentage;

    // ---- Check Requested Scaling ---- //
    bool no_scaling_requested = false, velocity_scaling_requested = false;
    check_requested_scaling (trajectory, &no_scaling_requested, &velocity_scaling_requested);


    // ---- NO-SCALING Requested ---- //  ->  No-Changes on Input Trajectory
    if (no_scaling_requested) {scaled_trajectory = input_trajectory;}
    

    // ---- TRAJECTORY SCALING Requested ---- //
    else if (velocity_scaling_requested) {

        // Creation of a Stop-Point (zero velocity) in the end of the trajectory
        sensor_msgs::JointState stop_point = add_stop_point(&input_trajectory);


        // ---- Q(s) SPLINE INTERPOLATION ---- //

        // Get Input Positions q(t)
        std::vector<Vector6d> input_positions;
        for (unsigned i = 0; i < input_trajectory.size() - 1; i++) {input_positions.push_back(Vector6d(input_trajectory[i].position.data()));}

        // Get Trajectory Registration Time & Sampling Time
        const double trajectory_time = (input_trajectory[input_trajectory.size()-1].header.stamp - input_trajectory[0].header.stamp).toSec();
        const double sampling_time = (input_trajectory[1].header.stamp - input_trajectory[0].header.stamp).toSec();
        ROS_INFO("Trajectory Time: %lf", trajectory_time);
        ROS_INFO("Sampling Time: %lf", sampling_time);

        // Spline Interpolation -> Q(s) = spline6d[joint_number](s) con s € [0,T]
        std::vector<tk::spline> q_spline6d = spline_interpolation (input_positions, trajectory_time, GET_VARIABLE_NAME(input_positions));


        // ---- COMPUTE SCALED VELOCITY ---- //

        // Registration Velocity    ->    If t ∈ [0,T] and s ∈ [0,T] -> ṡ_rec = 1
        double s_dot_rec = 1;

        // Scaled Velocity          ->    ṡ_des = gain * ṡ_rec
        double s_dot_des = compute_scaled_velocity (s_dot_rec, velocity_scaling_percentage);


        // ---- COMPUTE SCALED EVOLUTION ---- //

        // s_des[k] = s_des[k-1] + ṡ_des * τ, τ = sampling_time
        std::vector<double> s_des = compute_s_des (s_dot_des, trajectory_time, sampling_time);

        // q_des[k] = q_des[s[k]]
        std::vector<Vector6d> q_des = compute_desired_positions (s_des, q_spline6d);


        // ---- COMPUTE CONTROL VELOCITY ---- //

        // q̇_des[k] = (q_des[k+1] - q_des[k]) / τ
        std::vector<Vector6d> q_dot_des = compute_desired_velocities (q_des, sampling_time);


        // ---- SCALED TRAJECTORY CREATION ---- //

        // Create trajectory with computed positions and velocities
        scaled_trajectory = create_scaled_trajectory (input_trajectory, q_des, q_dot_des);


        // ---- LIMIT JOINT VELOCITY AND ACCELERATION ---- //

        // Check if the trajectory is feasable in respect with manipulator dynamics
        //TODO: scaled_trajectory = limit_joint_dynamics(scaled_trajectory);

    }

    return scaled_trajectory;

}


//-------------------------------------------- TRAJECTORY SCALING FUNCTIONS ---------------------------------------------//


void admittance_control::check_requested_scaling (admittance_controller::joint_trajectory trajectory, bool *no_scaling_requested, bool *velocity_scaling_requested) {

    int percentage_vel = trajectory.velocity_scaling_percentage;
    
     // ---- NO-SCALING Requested ---- //
    if (percentage_vel == 100 || percentage_vel == 0) {*no_scaling_requested = true; ROS_INFO("No-Scaling Requested");}

    // ---- PERCENTAGE-VELOCITY Requested ---- //
    else if (percentage_vel != 100 && percentage_vel != 0) {*velocity_scaling_requested = true; ROS_INFO("Velocity-Scaling Requested: %d%", percentage_vel);}

}

double admittance_control::compute_scaled_velocity (double s_dot_rec, int velocity_scaling_percentage) {

    // Compute Desired Gain
    double gain = velocity_scaling_percentage / 100;

    // Scaling Registration Velocity * Requested Gain
    double s_dot_des = s_dot_rec * gain;

    return s_dot_des;

}

std::vector<double> admittance_control::compute_s_des (double s_dot_des, double trajectory_time, double sampling_time) {

    std::vector<double> s_des;

    unsigned int k = 0;

    while (true) {

        double s_des_temp;

        if (k == 0) {s_des_temp = 0;}

        else if (k != 0) {

            // Compute s[k] = s[k-1] + ṡ * τ, τ = sampling_time
            s_des_temp = s_des[k-1] + s_dot_des * sampling_time;

            // Check if s_des[k] > T and Assign s_des[k] = T (last point)
            if (s_des_temp > trajectory_time) {s_des_temp = trajectory_time;}

        }
        
        s_des.push_back(s_des_temp);

        // ---- FINE TRAIETTORIA ---- //
        if (s_des[k] >= trajectory_time) {break;}
        
        k++;

    }

    // ---- DEBUG ---- //
    if (simple_debug) {
        std::string package_path = ros::package::getPath("admittance_controller");
        std::string save_file = package_path + "/debug/s_des_debug.csv";
        std::ofstream s_des_debug = std::ofstream(save_file);
        s_des_debug << "s_des\n";
        for (unsigned int i = 0; i < s_des.size(); i++) {s_des_debug << s_des[i] << "\n";}
        s_des_debug.close();
    }

    return s_des;

}

std::vector<Vector6d> admittance_control::compute_desired_positions (std::vector<double> s_des, std::vector<tk::spline> q_spline6d) {

    std::vector<Vector6d> q_des;

    // Compute Desired Joint Position from Geometric Position    ->    q_des[k] = q_des[s[k]]
    for (unsigned int i = 0; i < s_des.size(); i++) {q_des.push_back(get_spline_value(q_spline6d,s_des[i]));}

    // ---- DEBUG ---- //
    if (complete_debug) {
        std::string package_path = ros::package::getPath("admittance_controller");
        std::string save_file = package_path + "/debug/q_des_debug.csv";
        std::ofstream q_des_debug = std::ofstream(save_file);
        q_des_debug << "q_des\n";
        for (unsigned int i = 0; i < q_des.size(); i++) {
            for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {q_des_debug << q_des[i][joint_n] << ",";} q_des_debug << "\n";}
        q_des_debug.close();
    }

    return q_des;

}

std::vector<Vector6d> admittance_control::compute_desired_velocities (std::vector<Vector6d> q_des, double sampling_time) {

    std::vector<Vector6d> q_dot_des;

    // Add Stop-Point in the end of q_des
    q_des.push_back(q_des[q_des.size()-1]);

    // Compute q̇_des[k] = (q_des[k+1] - q_des[k]) / τ, τ = sampling_time
    for (unsigned int i = 0; i < q_des.size() - 1; i++) {q_dot_des.push_back((q_des[i+1].array() - q_des[i].array()) / sampling_time);}

    //TODO: check velocity and acceleration limits:
    // necessario controllare le velocità e se una eccede ridurla solo in quel calcolo, ripartendo però da s_des! 
    // poichè se rallento s_des[k] < s_des[k-1] + s_dot_des * sampling_time, poiche s_dot_lim < s_dot_des
    // fare il controllo su s_dot_des subito ? come ?

    // ---- DEBUG ---- //
    if (complete_debug) {
        std::string package_path = ros::package::getPath("admittance_controller");
        std::string save_file = package_path + "/debug/q_dot_des_debug.csv";
        std::ofstream q_dot_des_debug = std::ofstream(save_file);
        q_dot_des_debug << "q_dot_des\n";
        for (unsigned int i = 0; i < q_dot_des.size(); i++) {
            for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {q_dot_des_debug << q_dot_des[i][joint_n] << ",";} q_dot_des_debug << "\n";}
        q_dot_des_debug.close();
    }

    return q_dot_des;

}

std::vector<sensor_msgs::JointState> admittance_control::create_scaled_trajectory (std::vector<sensor_msgs::JointState> input_trajectory, std::vector<Vector6d> q_des, std::vector<Vector6d> q_dot_des) {

    std::vector<sensor_msgs::JointState> scaled_trajectory;

    ros::Time start_time = ros::Time::now();
    const double sampling_time = (input_trajectory[1].header.stamp - input_trajectory[0].header.stamp).toSec();

    // Create Scaled Trajectory
    for (unsigned int i = 0; i < q_dot_des.size(); i++) {
        
        // ---- DEBUG ---- //
        if (complete_debug) ROS_INFO_ONCE("q_dot_des size: %d", int(q_dot_des.size()));

        sensor_msgs::JointState temp;

        // Assign time to each point
        temp.header = input_trajectory[0].header;
        temp.header.seq = input_trajectory[0].header.seq + i;
        temp.header.stamp = start_time + ros::Duration(sampling_time * i);

        // Assign Joint Names
        temp.name = input_trajectory[0].name;

        // Assign Position and Velocities
        temp.position = std::vector<double>(q_des[i].data(), q_des[i].data() + q_des[i].size());
        temp.velocity = std::vector<double>(q_dot_des[i].data(), q_dot_des[i].data() + q_dot_des[i].size());

        scaled_trajectory.push_back(temp);
        
    }

    // ---- DEBUG ---- //
    if (simple_debug) {trajectory_debug_csv(scaled_trajectory,"scaled_trajectory");}

    return scaled_trajectory;

}


//-------------------------------------------- SPLINE INTERPOLATION FUNCTION --------------------------------------------//


std::vector<tk::spline> admittance_control::spline_interpolation (std::vector<Vector6d> data_vector, double spline_lenght, std::string output_file) {

    // Creation of Spline6d Vector -> Usage: spline6d[joint_number](s) = q(s)
    std::vector<tk::spline> spline6d;

    // Creation of s € [0,spline_lenght] vector
    std::vector<double> s;
    for (unsigned i = 0; i < data_vector.size(); i++) {
        double s_i = spline_lenght * (i / (double(data_vector.size()) - 1));
        s.push_back(s_i);
    }

    // Compute Spline for each Joint
    for (unsigned joint_number = 0; joint_number < 6; joint_number++) {

        // Create a Single-Joint Vector
        std::vector<double> waypoints_1d;
        for (unsigned i = 0; i < data_vector.size(); i++) {waypoints_1d.push_back(data_vector[i][joint_number]);}

        // Compute Cubic Spline [Q(s), s € [0,T]]
        tk::spline spline1d;
        spline1d.set_points(s, waypoints_1d);

        // Add Results to "spline6d" Vector
        spline6d.push_back(spline1d);

    }

    // ---- DEBUG ---- //
    if (complete_debug) {
        std::string package_path = ros::package::getPath("admittance_controller");
        std::string save_file = package_path + "/debug/" + output_file + "_spline6d_debug.csv";
        std::ofstream spline6d_debug = std::ofstream(save_file);
        spline6d_debug << "s, ,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6\n\n";
        for (unsigned int i = 0; i < data_vector.size(); i++) { 
            spline6d_debug << s[i] << ", ,";
            for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {spline6d_debug << spline6d[joint_n](s[i]) << ",";}
            spline6d_debug << "\n";
        }
        spline6d_debug.close();
    }
    
    return spline6d;

}

std::vector<tk::spline> admittance_control::spline_interpolation (std::vector<Array6d> data_vector, double spline_lenght, std::string output_file) {

    // Converting Array into Vector
    std::vector<Vector6d> data_vector_temp;
    for (unsigned int i = 0; i < data_vector.size(); i++) {data_vector_temp.push_back(data_vector[i].matrix());}

    return spline_interpolation(data_vector_temp, spline_lenght, output_file);

}

Vector6d admittance_control::get_spline_value (std::vector<tk::spline> spline6d, double s) {

    Vector6d spline_value;
    spline_value.resize(6);
    
    // Get spline1d value for each joint
    for (unsigned int i = 0; i < 6; i++) {spline_value[i] = spline6d[i](s);}

    return spline_value;

}


//-------------------------------------------------- CONTROL FUNCTIONS --------------------------------------------------//


void admittance_control::send_velocity_to_robot (Vector6d velocity) {

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

void admittance_control::send_position_to_robot (Vector6d position) {

    // Get available controller
    if (list_controllers_client.call(list_controllers_srv)) {ROS_INFO("Get Available Controllers");} else {ROS_ERROR("Failed to Call Service: \"/controller_manager/list_controllers\"");}
    std::vector<controller_manager_msgs::ControllerState> controller_list = list_controllers_srv.response.controller;

    switch_controller_srv.request.stop_controllers.resize(1);
    switch_controller_srv.request.start_controllers.resize(1);

    // Switch Controller (Velocity to Position)
    switch_controller_srv.request.stop_controllers[0]  = "joint_group_vel_controller";
    switch_controller_srv.request.start_controllers[0] = "scaled_pos_joint_traj_controller";
    switch_controller_srv.request.strictness = switch_controller_srv.request.STRICT;

    // Call Switch Controller Service
    if (switch_controller_client.call(switch_controller_srv)) {

        ROS_WARN("Controller Changed from \"%s\" to \"%s\"", switch_controller_srv.request.stop_controllers[0].c_str(), switch_controller_srv.request.start_controllers[0].c_str());
        ros::Duration(1).sleep();

    } else {
        
        ROS_ERROR("Failed to Call Service: \"/controller_manager/switch_controller\"");
        ROS_ERROR("Failed to Switch Controllers");
    }

    // Message Creation
    trajectory_msgs::JointTrajectory trajectory_temp;
    double execution_time = 10;

    trajectory_temp.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    trajectory_temp.points.resize(1);
    trajectory_temp.points[0].positions = {position[0], position[1], position[2], position[3], position[4], position[5]};
    trajectory_temp.points[0].time_from_start = ros::Duration(execution_time);

    // Publish Trajectory Position
    joint_trajectory_publisher.publish(trajectory_temp);
    ROS_INFO("GoTo Initial Position in %.2fs: %.2f %.2f %.2f %.2f %.2f %.2f", execution_time, position[0], position[1], position[2], position[3], position[4], position[5]);

    // Wait for Position Reached
    wait_for_position_reached(position, execution_time);

    // Switch Controller (Position to Velocity)
    switch_controller_srv.request.stop_controllers[0]  = "scaled_pos_joint_traj_controller";
    switch_controller_srv.request.start_controllers[0] = "joint_group_vel_controller";
    switch_controller_srv.request.strictness = switch_controller_srv.request.STRICT;

    // Call Switch Controller Service
    if (switch_controller_client.call(switch_controller_srv)) {

        ROS_WARN("Controller Changed from \"%s\" to \"%s\"", switch_controller_srv.request.stop_controllers[0].c_str(), switch_controller_srv.request.start_controllers[0].c_str());
        ros::Duration(1).sleep();
        
    } else {
        
        ROS_ERROR("Failed to Call Service: \"/controller_manager/switch_controller\"");
        ROS_ERROR("Failed to Switch Controllers");
    }

}

void admittance_control::wait_for_position_reached (Vector6d desired_position, double maximum_time) {

    joint_state_callback = false;

    // Wait for Joint State Callback
    while (!joint_state_callback) {ros::spinOnce();}

    Vector6d current_position(joint_state.position.data());

    ros::Time start_time = ros::Time::now();

    // Wait until desired_position and current_position are equal with a little tolerance
    while ((Eigen::abs(desired_position.array() - current_position.array()) > 0.0001).all() || ((ros::Time::now() - start_time).toSec() < maximum_time)) {

        ros::spinOnce();
        current_position = Vector6d(joint_state.position.data());
        
    }

}

void admittance_control::freedrive_mode (bool activation) {

    std_msgs::String freedrive_mode_command;

    if (activation) {

        // Turn ON FreedriveMode
        ROS_WARN("FreeDrive Mode Activated");
        freedrive_mode_command.data = "def prog(): freedrive_mode() while(1) end";

        // Deactivate Admittance Controller
        admittance_control_request = false;
    
    } else {

        // Turn OFF FreedriveMode
        ROS_WARN("FreeDrive Mode Dectivated");
        freedrive_mode_command.data = "def prog(): end";

        // Activate Admittance Controller
        admittance_control_request = true;
    
    }

    ur10e_script_command_publisher.publish(freedrive_mode_command);

}


//--------------------------------------------------- UTILS FUNCTIONS ---------------------------------------------------//


Vector6d admittance_control::low_pass_filter(Vector6d input_vec) {

    // Adding new element to filter vector    
    filter_elements.push_back(input_vec);

    // Keep only the last 100 values of the vector
    while (filter_elements.size() > 100) {filter_elements.erase(filter_elements.begin());}

    // Median Filter (media = sum / N_elements)
    Vector6d sum, median;
    for (unsigned int i = 0; i < filter_elements.size(); i++) {sum += filter_elements[i];}
    median = sum / filter_elements.size();
    
    return median;

}

void admittance_control::wait_for_callbacks_initialization (void) {

    ros::Duration(1).sleep();

    // Wait for the Callbacks
    while (ros::ok() && (!force_callback || !joint_state_callback)) {

        ros::spinOnce();
        
        if (!force_callback) {ROS_WARN_THROTTLE(3, "Wait for Force Sensor");}
        if (!joint_state_callback) {ROS_WARN_THROTTLE(3, "Wait for Joint State feedback");}

    }

}

int admittance_control::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void admittance_control::spinner (void) {

    ros::spinOnce();

    // FreeDrive Mode
    if (freedrive_mode_request) {ros::spinOnce();}
    
    // Admittance Controller
    else if (admittance_control_request) {

        compute_admittance();
        send_velocity_to_robot(q_dot);
        ros::spinOnce();
        loop_rate.sleep();

    }

}
