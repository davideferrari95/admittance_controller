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
    ROS_INFO_ONCE("Maximum Velocity:     %.2f %.2f %.2f %.2f %.2f %.2f", max_vel[0], max_vel[1], max_vel[2], max_vel[3], max_vel[4], max_vel[5]);
    ROS_INFO_ONCE("Maximum Acceleration: %.2f %.2f %.2f %.2f %.2f %.2f \n", max_acc[0], max_acc[1], max_acc[2], max_acc[3], max_acc[4], max_acc[5]);
    ROS_INFO_ONCE("Force Dead Zone:   %.2f", force_dead_zone);
    ROS_INFO_ONCE("Troque Dead Zone:  %.2f", torque_dead_zone);
    ROS_INFO_ONCE("Admittance Weight: %.2f \n", admittance_weight);
    ROS_INFO_STREAM_ONCE("Inertia Reduction: " << inertia_reduction << std::endl);
    ROS_INFO_STREAM_ONCE("Cycle Time: " << loop_rate.expectedCycleTime().toSec()*1000 << " ms" << std::endl);
    
    // ---- DEBUG OFSTREAM ---- //
    std::string package_path = ros::package::getPath("admittance_controller");
    ROS_INFO_STREAM_ONCE("Package Path:  " << package_path << std::endl);
    std::string save_file = package_path + "/debug/ft_sensor.txt";
    ft_sensor_debug = std::ofstream(save_file);
    
    // ---- WAIT FOR INITIALIZATION ---- //
    wait_for_callbacks_initialization();

    // ---- ZERO FT SENSOR ---- //
    while (use_ur_real_robot && !zero_ft_sensor_client.call(zero_ft_sensor_srv)) {ROS_WARN_THROTTLE(2,"Wait for Service: \"/ur_hardware_interface/zero_ftsensor\"");}

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
    
    for (int i = 0; i < 6; i++) {ft_sensor_debug << external_wrench[i] << " ";}
    ft_sensor_debug << "\n";

    ROS_DEBUG_THROTTLE(2, "Sensor Force/Torque  ->  Fx: %.2f  Fy: %.2f  Fz: %.2f  |  Tx: %.2f  Ty: %.2f  Tz: %.2f", external_wrench[0], external_wrench[1], external_wrench[2], external_wrench[3], external_wrench[4], external_wrench[5]);

    for (int i = 0; i < 3; i++) {if(fabs(external_wrench[i]) < fabs(force_dead_zone)) {external_wrench[i] = 0.0;}}
    for (int i = 3; i < 6; i++) {if(fabs(external_wrench[i]) < fabs(torque_dead_zone)) {external_wrench[i] = 0.0;}}
    
    ROS_INFO_THROTTLE(2, "Sensor Force/Torque Clamped  ->  Fx: %.2f  Fy: %.2f  Fz: %.2f  |  Tx: %.2f  Ty: %.2f  Tz: %.2f", external_wrench[0], external_wrench[1], external_wrench[2], external_wrench[3], external_wrench[4], external_wrench[5]);
    
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

    ROS_DEBUG_THROTTLE(2, "joint position: %.2f %.2f %.2f %.2f %.2f %.2f", joint_position[0], joint_position[1], joint_position[2], joint_position[3], joint_position[4], joint_position[5]);

    joint_state_callback = true;
    
}

void admittance_control::trajectory_execution_Callback (const admittance_controller::joint_trajectory::ConstPtr &msg) {

    admittance_controller::joint_trajectory t = *msg;

    t.trajectory = trajectory_scaling(t);

    ROS_WARN("Trajectory Execution Callback");

    trajectory_execution(t.trajectory);

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

    ROS_DEBUG_STREAM_THROTTLE(2, "Manipulator Jacobian: " << std::endl << std::endl << jacobian << std::endl);
    ROS_DEBUG_STREAM_THROTTLE(2, "Manipulator Inverse Jacobian: " << std::endl << std::endl << jacobian.inverse() << std::endl);

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

    // -- DEBUG OUTPUT -- //
    ROS_DEBUG_THROTTLE(2, "Translation Vector   ->   X: %.3f  Y: %.3f  Z: %.3f", end_effector_state.translation().x(), end_effector_state.translation().y(), end_effector_state.translation().z());
    ROS_DEBUG_THROTTLE(2, "Euler Angles         ->   R: %.3f  P: %.3f  Y: %.3f", euler_angles[0], euler_angles[1], euler_angles[2]);
    ROS_DEBUG_THROTTLE(2, "Rotation Quaternion  ->   X: %.3f  Y: %.3f  Z: %.3f  W: %.3f", rotation_quaternion.x(), rotation_quaternion.y(), rotation_quaternion.z(), rotation_quaternion.w());

    ROS_DEBUG_STREAM_THROTTLE(2, "Rotation Matrix from Model:" << std::endl << std::endl << end_effector_state.rotation() << std::endl);
    ROS_DEBUG_STREAM_THROTTLE(2, "Rotation Matrix 6x6:" << std::endl << std::endl << rotation_matrix << std::endl);

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
        ROS_DEBUG_STREAM_ONCE("Start Velocity: " << std::endl << std::endl << x_dot << std::endl);
    
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

    ROS_DEBUG_THROTTLE(2, "Desired Cartesian Velocity:  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f", x_dot[0], x_dot[1], x_dot[2], x_dot[3], x_dot[4], x_dot[5]);
    ROS_DEBUG_THROTTLE(2, "Desired  Joints   Velocity:  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f", q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5]);

}


//----------------------------------------------- LIMIT DYNAMICS FUNCTIONS ----------------------------------------------//


Vector6d admittance_control::limit_joint_dynamics (Vector6d joint_velocity) {

    double duration = loop_rate.expectedCycleTime().toSec();

    // Limit Joint Velocity

    for (int i = 0; i < joint_velocity.size(); i++) {

        if (fabs(joint_velocity[i]) > max_vel[i]) {

            ROS_DEBUG("Reached Maximum Velocity on Joint %d   ->   Velocity: %.3f   Limited at: %.3f", i, joint_velocity[i], sign(joint_velocity[i]) * max_vel[i]);
            joint_velocity[i] = sign(joint_velocity[i]) * max_vel[i];

        }

    }

    // Limit Joint Acceleration

    for (int i = 0; i < joint_velocity.size(); i++) {

        if (fabs(joint_velocity[i] - q_dot_last_cycle[i]) > max_acc[i] * duration) {

            ROS_DEBUG("Reached Maximum Acceleration on Joint %d   ->   Acceleration: %.3f   Limited at: %.3f", i, (joint_velocity[i] - q_dot_last_cycle[i]) / duration, q_dot_last_cycle[i] +  sign(joint_velocity[i] - q_dot_last_cycle[i]) * max_acc[i]);
            joint_velocity[i] = q_dot_last_cycle[i] + sign(joint_velocity[i] - q_dot_last_cycle[i]) * max_acc[i] * duration;
        
        }

    }

    q_dot_last_cycle = joint_velocity;
    return joint_velocity;

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
    sensor_msgs::JointState stop_point;
    stop_point.name = trajectory[0].name;
    stop_point.position = trajectory[trajectory.size()-1].position;
    std::fill(stop_point.velocity.begin(), stop_point.velocity.end(), 0);
    long double time_stamp = trajectory[trajectory.size()-1].header.stamp.sec + (trajectory[1].header.stamp.sec  - trajectory[0].header.stamp.sec) 
                           + trajectory[trajectory.size()-1].header.stamp.nsec * pow(10,-9) + (trajectory[1].header.stamp.nsec - trajectory[0].header.stamp.nsec);
    stop_point.header.stamp.sec  = std::floor(time_stamp);
    stop_point.header.stamp.nsec = (time_stamp - std::floor(time_stamp)) * pow(10,9);

    // Add Stop-Point in the end of the trajectory
    trajectory.push_back(stop_point);

    for (unsigned i = 0; i < trajectory.size() - 1; i++) {

        // Compute Trajectory Rate (Point [i+1] - Point [i])
        long double trajectory_rate = (trajectory[i+1].header.stamp.sec + (trajectory[i+1].header.stamp.nsec * pow(10,-9))) - (trajectory[i].header.stamp.sec + (trajectory[i].header.stamp.nsec * pow(10,-9)));

        // Command robot in velocity
        send_velocity_to_robot(Vector6d(trajectory[i].velocity.data()));

        // Sleep
        ros::Duration(trajectory_rate).sleep();

    }

    // End-Trajectory Stop-Point 
    send_velocity_to_robot(Vector6d(stop_point.velocity.data()));

}

std::vector<sensor_msgs::JointState> admittance_control::trajectory_scaling (admittance_controller::joint_trajectory trajectory) {

    std::vector<sensor_msgs::JointState> input_trajectory = trajectory.trajectory, scaled_trajectory;

    // Target Velocity [number]
    double target_velocity = trajectory.target_velocity;

    // Percentage of Scaling [%]
    int velocity_scaling_percentage = trajectory.velocity_scaling_percentage;


    // ---- NO-SCALING Requested ---- //
    if ((velocity_scaling_percentage == 100 || velocity_scaling_percentage == 0) && target_velocity == 0) {

        ROS_INFO("No-Scaling Requested");

        // No-Changes on Input Trajectory
        scaled_trajectory = input_trajectory;
    }

    // ---- SCALING Requested ---- //
    else {

        // Creation of a Stop-Point (zero velocity) in the end of the trajectory
        sensor_msgs::JointState stop_point;
        stop_point.name = input_trajectory[0].name;
        stop_point.position = input_trajectory[input_trajectory.size()-1].position;
        std::fill(stop_point.velocity.begin(), stop_point.velocity.end(), 0);
        long double time_stamp = input_trajectory[input_trajectory.size()-1].header.stamp.sec + (input_trajectory[1].header.stamp.sec  - input_trajectory[0].header.stamp.sec) 
                               + input_trajectory[input_trajectory.size()-1].header.stamp.nsec * pow(10,-9) + (input_trajectory[1].header.stamp.nsec - input_trajectory[0].header.stamp.nsec);
        stop_point.header.stamp.sec  = std::floor(time_stamp);
        stop_point.header.stamp.nsec = (time_stamp - std::floor(time_stamp)) * pow(10,9);

        // Add Stop-Point in the end of the trajectory
        input_trajectory.push_back(stop_point);

        std::vector<Vector6d> input_positions;

        // Get Input Positions q(t) and Velocities q̇(t)
        for (unsigned i = 0; i < input_trajectory.size() - 1; i++) {input_positions.push_back(Vector6d(input_trajectory[i].position.data()));}

        // Spline Interpolation -> Q(s) = spline6d[joint_number](s) con s € [0,1]
        std::vector<tk::spline> q_spline6d = spline_interpolation (input_positions);

        // Creation of s € [0,1] vector (s[0] = 0)
        std::vector<double> s;

        std::vector<Array6d> s_dot_rec, q_dot_rec, v_geom_rec;
        
        // Find Registration Velocity ṡ -> q̇ = dq/ds * ṡ, ṡ = ds/dt
        for (unsigned int i = 0; i < input_trajectory.size() - 1; i++) {
            
            // Creation of s € [0,1] vector
            s.push_back(double(i) * 1 / (double(input_trajectory.size() - 1) - 1));
            
            // dq = q[i+1] - q[i]
            Array6d dq, ds(s[i+1] - s[i]), q_dot_rec_i(input_trajectory[i].velocity.data());
            for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {dq[joint_n] = q_spline6d[joint_n](s[i+1]) - q_spline6d[joint_n](s[i]);}

            // geometric velocity (v_geom) = dq/ds = (q[i]-q[i-1])/(s[i]-s[i-1])
            Array6d v_geom = dq / ds;

            // joint velocity (q̇) = dq/ds * ṡ -> ṡ = q̇ * ds/dq = q̇ / v_geom
            Array6d s_dot_rec_i = q_dot_rec_i / v_geom;

            s_dot_rec.push_back(s_dot_rec_i);
            q_dot_rec.push_back(q_dot_rec_i);
            v_geom_rec.push_back(v_geom);

        }

        std::vector<Array6d> gain;
        std::vector<double> time_gain;

        // ---- TARGET FIXED-VELOCITY Requested ---- //
        if (target_velocity != 0) {

            ROS_INFO("Target-Velocity Scaling Requested");

            for (unsigned int i = 0; i < input_trajectory.size() - 1; i++) {

                // Variable Gain -> Fixed Velocity
                gain.push_back((1 / s_dot_rec[i]) * target_velocity);

                // TODO: se richiedo una v costante (quindi non doppio/metà v_rec) il time_gain è diverso per ogni giunto
                // time_gain.push_back();
            
            }
        }

        // ---- PERCENTAGE-VELOCITY Requested ---- //
        else if (velocity_scaling_percentage != 100 && velocity_scaling_percentage != 0) {

            ROS_INFO("Percentage-Velocity Scaling Requested");
            
            for (unsigned int i = 0; i < input_trajectory.size() - 1; i++) {
                
                // Fixed Gain
                Array6d vel_scaling(velocity_scaling_percentage);
                gain.push_back(vel_scaling / 100);

                // Time Gain
                time_gain.push_back(velocity_scaling_percentage / 100);

            }
        }

        std::vector<Array6d> s_dot_des, q_dot_des;
        s_dot_des.resize(s_dot_rec.size());
        q_dot_des.resize(q_dot_rec.size());

        bool time_scaling = true;

        // ---- TIME SCALING ---- //
        if (time_scaling) {

            // ---- Change ṡ and t, ds = cost, trajectory points = cost ---- //

            // Time Vectors Creation
            std::vector<ros::Time> time_rec, time_des;
            time_rec.resize(input_trajectory.size());
            time_des.resize(input_trajectory.size());

            // Get Registration Time Vector
            for (unsigned int i = 0; i < input_trajectory.size(); i++) {
                time_rec[i].sec  = input_trajectory[i].header.stamp.sec;
                time_rec[i].nsec = input_trajectory[i].header.stamp.nsec;
            }

            // Scaling Registration Velocity * Requested Gain
            for (unsigned int i = 0; i < input_trajectory.size() - 1; i++) {s_dot_des[i] = s_dot_rec[i] * gain[i];}

            // Ricompute New q̇ = dq/ds * ṡ
            for (unsigned int i = 0; i < input_trajectory.size() - 1; i++) {q_dot_des[i] = s_dot_des[i] * v_geom_rec[i];}

            // Scaling Registration Time / Requested Gain
            for (unsigned int i = 0; i < input_trajectory.size() - 1; i++) {
                
                long double dt_rec = (time_rec[i+1].sec + time_rec[i+1].nsec * pow(10,-9)) - (time_rec[i].sec + time_rec[i].nsec * pow(10,-9));
                long double dt_des = dt_rec / time_gain[i];

                // Assign Starting Time
                if (i = 0) {time_des[0] = time_rec[0];}

                // Assign Next Time (floor = get only integer part)
                long double t_des  = time_des[i].sec + time_des[i].nsec * pow(10,-9) + dt_des;
                time_des[i+1].sec  = std::floor(t_des);
                time_des[i+1].nsec = (t_des - std::floor(dt_des)) * pow(10,9);
                
            }
            
            // Create Scaled Trajectory
            for (unsigned int i = 0; i < input_trajectory.size(); i++) {
                
                sensor_msgs::JointState temp;
                temp.header.stamp = time_des[i];
                temp.name         = input_trajectory[i].name;
                temp.position     = input_trajectory[i].position;
                temp.velocity     = std::vector<double>(q_dot_des[i].data(), q_dot_des[i].data() + q_dot_des[i].size());

                scaled_trajectory.push_back(temp);
                
            }
            
        }

        // ---- SPACE SCALING ---- //
        else if (!time_scaling) {

            // ---- Change ṡ and ds, t = cost, trajectory points != cost ---- //

        }

    }

    // ---- LIMIT JOINTS DYNAMIC ---- //

    for (unsigned int i = 1; i < scaled_trajectory.size(); i++) {

        // Compute Sampling Time
        double sampling_time = (input_trajectory[i].header.stamp.sec + (input_trajectory[i].header.stamp.nsec * pow(10,-9))) - (input_trajectory[i-1].header.stamp.sec + (input_trajectory[i-1].header.stamp.nsec * pow(10,-9)));

        for (int joint_n = 0; joint_n < 6; joint_n++) {
            
            // Limit Joint Velocity
            if (fabs(scaled_trajectory[i].velocity[joint_n]) > max_vel[joint_n]) {

                ROS_DEBUG("Reached Maximum Velocity on Joint %d   ->   Velocity: %.3f   Limited at: %.3f", joint_n, scaled_trajectory[i].velocity[joint_n], sign(scaled_trajectory[i].velocity[joint_n]) * max_vel[joint_n]);
                scaled_trajectory[i].velocity[joint_n] = sign(scaled_trajectory[i].velocity[joint_n]) * max_vel[joint_n];

            }
            
            // Limit Joint Acceleration
            if (fabs(scaled_trajectory[i].velocity[joint_n] - scaled_trajectory[i-1].velocity[joint_n]) > max_acc[joint_n] * sampling_time) {

                ROS_DEBUG("Reached Maximum Acceleration on Joint %d   ->   Acceleration: %.3f   Limited at: %.3f", joint_n, (scaled_trajectory[i].velocity[joint_n] - scaled_trajectory[i-1].velocity[joint_n]) / sampling_time, scaled_trajectory[i-1].velocity[joint_n] +  sign(scaled_trajectory[i].velocity[joint_n] - scaled_trajectory[i-1].velocity[joint_n]) * max_acc[joint_n]);
                scaled_trajectory[i].velocity[joint_n] = scaled_trajectory[i-1].velocity[joint_n] + sign(scaled_trajectory[i].velocity[joint_n] - scaled_trajectory[i-1].velocity[joint_n]) * max_acc[joint_n] * sampling_time;

            }

        }
        
    }

    return scaled_trajectory;

}

std::vector<tk::spline> admittance_control::spline_interpolation (std::vector<Vector6d> data_vector) {

    std::vector<tk::spline> spline6d;

    // Creation of s € [0,1] vector
    std::vector<double> s;
    for (unsigned i = 0; i < data_vector.size(); i++) {
        double s_i = i * 1 / (double(data_vector.size()) - 1);
        s.push_back(s_i);
    }

    // Compute Spline for each Joint
    for (unsigned joint_number = 0; joint_number < 6; joint_number++) {

        // Create a Single-Joint Vector
        std::vector<double> waypoints_1d;
        for (unsigned i = 0; i < data_vector.size(); i++) {waypoints_1d.push_back(data_vector[i][joint_number]);}

        // Compute Cubic Spline [Q(s), s € [0,1]]
        tk::spline spline1d;
        spline1d.set_points(s, waypoints_1d);

        // Add Results to "spline6d" Vector
        spline6d.push_back(spline1d);

    }

    // ---- DEBUG ---- //
    for (unsigned int spline_number = 0; spline_number < 6; spline_number++) {
    
        tk::spline spline1d = spline6d[spline_number];
        std::string package_path = ros::package::getPath("admittance_controller");
        std::string save_file = package_path + "/debug/spline1d_joint" + std::to_string(spline_number+1) + "_debug.csv";
        std::ofstream spline1d_debug = std::ofstream(save_file);

        spline1d_debug << "Point,s\n";

        for (unsigned int i = 0; i < data_vector.size(); i++) {
            double x = i / (double(data_vector.size()) - 1);
            spline1d_debug << spline1d(x) << "," << x << "\n";
        }
    }

    // spline6d[joint_number](s) = q(s)

    return spline6d;

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
    wait_for_position_reached(position);

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

void admittance_control::wait_for_position_reached (Vector6d desired_position) {

    ros::spinOnce();

    Vector6d current_position(joint_state.position.data());

    // Wait until desired_position and current_position are equal with a little tolerance
    while ((Eigen::abs(desired_position.array() - current_position.array()) > 0.0001).all()) {

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
