#include <dwl_planners/ModeInvariantWholeBodyPlanner.h>


namespace dwl_planners
{

ModeInvariantWholeBodyPlanner::ModeInvariantWholeBodyPlanner(ros::NodeHandle node) : privated_node_(node),
		interpolation_time_(0.), computation_time_(0.), new_current_state_(true)//(false)
{
//	current_state_.base_pos(dwl::rbd::LZ) = 0.2;
	current_state_.joint_pos = Eigen::VectorXd::Zero(2);
	current_state_.joint_pos << 0.6, -1.5;
	current_state_.joint_vel = Eigen::VectorXd::Zero(2);
	current_state_.joint_acc = Eigen::VectorXd::Zero(2);
	current_state_.joint_eff = Eigen::VectorXd::Zero(2);
	current_state_.joint_eff << 9.33031, 27.6003;
	current_state_.contacts.resize(1);
	current_state_.contacts[0].position = Eigen::Vector3d::Zero();
	current_state_.contacts[0].velocity = Eigen::Vector3d::Zero();
	current_state_.contacts[0].acceleration = Eigen::Vector3d::Zero();
	current_state_.contacts[0].force << 0., 0., 106.555;//Eigen::Vector3d::Zero();
}


ModeInvariantWholeBodyPlanner::~ModeInvariantWholeBodyPlanner()
{

}


void ModeInvariantWholeBodyPlanner::init()
{
	// Setting publishers and subscribers
	motion_plan_pub_ = node_.advertise<dwl_msgs::WholeBodyTrajectory>("/hyl/constrained_operational_controller/plan", 1);
	robot_state_sub_ = node_.subscribe<sensor_msgs::JointState>("/hyl/joint_states", 1,
			&ModeInvariantWholeBodyPlanner::jointStateCallback, this);

	// Initializing the planning optimizer
	dwl::solver::IpoptNLP* ipopt_solver = new dwl::solver::IpoptNLP();
	dwl::solver::OptimizationSolver* solver = ipopt_solver;
	planning_.init(solver);


	// Initializing the dynamical system constraint
	std::string model_file = "/home/cmastalli/ros_workspace/src/dwl/thirdparty/rbdl/hyl.urdf";
	dwl::model::FullDynamicalSystem* system_constraint = new dwl::model::FullDynamicalSystem();
	dwl::model::DynamicalSystem* dynamical_system = system_constraint;

	dynamical_system->modelFromURDFFile(model_file, true);

	// Reading and setting the integration method of dynamical constraint
	std::string integration_method;
	privated_node_.param<std::string>("dynamical_system/time_integration/type", integration_method, "fixed");
	if (integration_method == "variable")
		dynamical_system->setStepIntegrationMethod(dwl::model::Variable);
	else
		dynamical_system->setStepIntegrationMethod(dwl::model::Fixed);

	// Reading the full-trajectory parameter
	bool full_trajectory_opt;
	privated_node_.param("dynamical_system/full_trajectory_optimization", full_trajectory_opt, false);
	if (full_trajectory_opt)
		dynamical_system->setFullTrajectoryOptimization();

	// Adding the dynamical system
	planning_.addDynamicalSystem(dynamical_system);
	dwl::model::FloatingBaseSystem system = planning_.getDynamicalSystem()->getFloatingBaseSystem();

	// Reading and setting the time integration step
	double step_time;
	privated_node_.param("dynamical_system/time_integration/step_time", step_time, 0.1);
	planning_.setStepIntegrationTime(step_time);


	// Adding the contact model constraint
	dwl::model::InelasticContactModelConstraint* contact_constraint = new dwl::model::InelasticContactModelConstraint();
	contact_constraint->modelFromURDFFile(model_file);
	planning_.addConstraint(contact_constraint);

	dwl::model::InelasticContactVelocityConstraint* contact_vel_constraint = new dwl::model::InelasticContactVelocityConstraint();
	contact_vel_constraint->modelFromURDFFile(model_file);
	planning_.addConstraint(contact_vel_constraint);


	// Initializing the desired state, integral and terminal cost weights
	desired_state_.setJointDoF(system.getJointDoF());
	dwl::LocomotionState integral_weights(system.getJointDoF());
	dwl::LocomotionState terminal_weights(system.getJointDoF());

	// Reading base information
	const char* base_names[] = {"AX", "AY", "AZ", "LX", "LY", "LZ"};
	for (unsigned int coord_idx = 0; coord_idx < 6; coord_idx++) {
		dwl::rbd::Coords6d coord = dwl::rbd::Coords6d(coord_idx);
		std::string name = base_names[coord_idx];
		double value;

		// Reading desired base positions
		privated_node_.param("desired_state/position/" + name, value, 0.0);
		desired_state_.base_pos(coord) = value;

		// Reading desired base velocities
		privated_node_.param("desired_state/velocity/" + name, value, 0.0);
		desired_state_.base_vel(coord) = value;

		// Reading desired base accelerations
		privated_node_.param("desired_state/acceleration/" + name, value, 0.0);
		desired_state_.base_acc(coord) = value;

		// Reading the weight value of the base positions
		privated_node_.param("integral_cost/state_tracking_energy/position/base/" + name,
				value, 0.0);
		integral_weights.base_pos(coord) = value;
		privated_node_.param("terminal_cost/state_tracking_energy/position/base/" + name,
				value, 0.0);
		terminal_weights.base_pos(coord) = value;


		// Reading the weight value of the base velocities
		privated_node_.param("integral_cost/state_tracking_energy/velocity/base/" + name,
				value, 0.0);
		integral_weights.base_vel(coord) = value;
		privated_node_.param("terminal_cost/state_tracking_energy/velocity/base/" + name,
				value, 0.0);
		terminal_weights.base_vel(coord) = value;

		// Reading the weight value of the base accelerations
		privated_node_.param("integral_cost/state_tracking_energy/acceleration/base/" + name,
				value, 0.0);
		integral_weights.base_acc(coord) = value;
		privated_node_.param("terminal_cost/state_tracking_energy/acceleration/base/" + name,
				value, 0.0);
		terminal_weights.base_acc(coord) = value;
	}

	// Reading joint information
	for (dwl::urdf_model::JointID::const_iterator jnt_it = system.getJoints().begin();
			jnt_it != system.getJoints().end(); jnt_it++) {
		unsigned int joint_idx =  jnt_it->second - system.getFloatingBaseDoF();
		std::string joint_name = jnt_it->first;
		double value;

		// Reading the weight value of the joint positions
		if (!privated_node_.getParam("integral_cost/state_tracking_energy/position/joints/" +
				joint_name, value))
			ROS_WARN("The position weight of integral cost in %s joint is not defined", joint_name.c_str());
		else
			integral_weights.joint_pos(joint_idx) = value;
		if (!privated_node_.getParam("terminal_cost/state_tracking_energy/position/joints/" +
				joint_name, value))
			ROS_WARN("The position weight of terminal cost in %s joint is not defined", joint_name.c_str());
		else
			terminal_weights.joint_pos(joint_idx) = value;

		// Reading the weight value of the joint velocities
		if (!privated_node_.getParam("integral_cost/state_tracking_energy/velocity/joints/" +
				joint_name, value))
			ROS_WARN("The velocity weight of integral cost in %s joint is not defined", joint_name.c_str());
		else
			integral_weights.joint_vel(joint_idx) = value;
		if (!privated_node_.getParam("terminal_cost/state_tracking_energy/velocity/joints/" +
				joint_name, value))
			ROS_WARN("The velocity weight of terminal cost in %s joint is not defined", joint_name.c_str());
		else
			terminal_weights.joint_vel(joint_idx) = value;

		// Reading the weight value of the joint accelerations
		if (!privated_node_.getParam("integral_cost/state_tracking_energy/acceleration/joints/" +
				joint_name, value))
			ROS_WARN("The acceleration weight of integral cost in %s joint is not defined", joint_name.c_str());
		else
			integral_weights.joint_acc(joint_idx) = value;
		if (!privated_node_.getParam("terminal_cost/state_tracking_energy/acceleration/joints/" +
				joint_name, value))
			ROS_WARN("The acceleration weight of terminal cost in %s joint is not defined", joint_name.c_str());
		else
			terminal_weights.joint_acc(joint_idx) = value;

		// Control weights
		if (!privated_node_.getParam("integral_cost/control_energy/" + joint_name, value))
			ROS_WARN("The control weight of integral const in %s joint is not defined", joint_name.c_str());
		else
			integral_weights.joint_eff(joint_idx) = value;
	}

	// Setting the cost functions
	dwl::model::Cost* integral_state_tracking_cost = new dwl::model::IntegralStateTrackingEnergyCost();
	integral_state_tracking_cost->setWeights(integral_weights);
	dwl::model::Cost* terminal_state_tracking_cost = new dwl::model::TerminalStateTrackingEnergyCost();
	terminal_state_tracking_cost->setWeights(terminal_weights);
	dwl::model::Cost* integral_control_cost = new dwl::model::IntegralControlEnergyCost();
	integral_control_cost->setWeights(integral_weights);

	// Adding the cost functions
	planning_.addCost(integral_state_tracking_cost);
	planning_.addCost(terminal_state_tracking_cost);
	planning_.addCost(integral_control_cost);


	// Reading the interpolation time
	privated_node_.param("interpolation_time", interpolation_time_, -1.);

	// Reading and setting the horizon value
	int horizon;
	privated_node_.param("horizon", horizon, 1);
	planning_.setHorizon(horizon);

	// Reading the allowed computation time
	privated_node_.param("computation_time", computation_time_, std::numeric_limits<double>::max());
}


bool ModeInvariantWholeBodyPlanner::compute()
{
	if (new_current_state_) {
		new_current_state_ = false;
		return planning_.compute(current_state_, desired_state_, computation_time_);
	} else
		return false;
}


void ModeInvariantWholeBodyPlanner::publishWholeBodyTrajectory()
{
	// Publishing the motion plan if there is at least one subscriber
	if (motion_plan_pub_.getNumSubscribers() > 0) {
		robot_trajectory_msg_.header.stamp = ros::Time::now();

		// Filling the current state
		writeWholeBodyStateMessage(robot_trajectory_msg_.actual,
								   current_state_);

		// Filling the trajectory message
		dwl::locomotion::LocomotionTrajectory trajectory;
		if (interpolation_time_ <= 0.)
			trajectory = planning_.getWholeBodyTrajectory();
		else
			trajectory = planning_.getInterpolatedWholeBodyTrajectory(interpolation_time_);

		robot_trajectory_msg_.trajectory.resize(trajectory.size());
		for (unsigned int i = 0; i < trajectory.size(); i++)
			writeWholeBodyStateMessage(robot_trajectory_msg_.trajectory[i], trajectory[i]);

		// Publishing the motion plan
		motion_plan_pub_.publish(robot_trajectory_msg_);
	}
}



void ModeInvariantWholeBodyPlanner::writeWholeBodyStateMessage(dwl_msgs::WholeBodyState& msg,
															   const dwl::LocomotionState& state)
{
	// Getting the floating-base system information
	dwl::model::FloatingBaseSystem system = planning_.getDynamicalSystem()->getFloatingBaseSystem();

	// Filling the time information
	msg.time = state.time;

	// Filling the base state
	msg.base.resize(system.getFloatingBaseDoF());
	unsigned int counter = 0;
	for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
		dwl::rbd::Coords6d base_coord = dwl::rbd::Coords6d(base_idx);
		dwl::model::FloatingBaseJoint base_joint = system.getFloatingBaseJoint(base_coord);

		if (base_joint.active) {
			msg.base[counter].id = base_idx;
			msg.base[counter].name = base_joint.name;

			msg.base[counter].position = state.base_pos(base_idx);
			msg.base[counter].velocity = state.base_vel(base_idx);
			msg.base[counter].acceleration = state.base_acc(base_idx);

			counter++;
		}
	}

	// Filling the joint state
	msg.joints.resize(system.getJointDoF());
	for (dwl::urdf_model::JointID::const_iterator jnt_it = system.getJoints().begin();
			jnt_it != system.getJoints().end(); jnt_it++) {
		unsigned int joint_idx =  jnt_it->second - system.getFloatingBaseDoF();
		std::string joint_name = jnt_it->first;

		msg.joints[joint_idx].name = joint_name;
		msg.joints[joint_idx].position = state.joint_pos(joint_idx);
		msg.joints[joint_idx].velocity = state.joint_vel(joint_idx);
		msg.joints[joint_idx].acceleration = state.joint_acc(joint_idx);
		msg.joints[joint_idx].effort = state.joint_eff(joint_idx);
	}

	// Filling the contact state
	msg.contacts.resize(system.getNumberOfEndEffectors());
	for (dwl::urdf_model::LinkID::const_iterator ee_it = system.getEndEffectors().begin();
			ee_it != system.getEndEffectors().end(); ee_it++) {
		unsigned int ee_idx =  ee_it->second;
		std::string ee_name = ee_it->first;

		msg.contacts[ee_idx].name = ee_name;

		// Positions
		msg.contacts[ee_idx].position.x = state.contacts[ee_idx].position(dwl::rbd::X);
		msg.contacts[ee_idx].position.y = state.contacts[ee_idx].position(dwl::rbd::Y);
		msg.contacts[ee_idx].position.z = state.contacts[ee_idx].position(dwl::rbd::Z);

		// Velocities
		msg.contacts[ee_idx].velocity.x = state.contacts[ee_idx].velocity(dwl::rbd::X);
		msg.contacts[ee_idx].velocity.y = state.contacts[ee_idx].velocity(dwl::rbd::Y);
		msg.contacts[ee_idx].velocity.z = state.contacts[ee_idx].velocity(dwl::rbd::Z);

		// Accelerations
		msg.contacts[ee_idx].acceleration.x = state.contacts[ee_idx].acceleration(dwl::rbd::X);
		msg.contacts[ee_idx].acceleration.y = state.contacts[ee_idx].acceleration(dwl::rbd::Y);
		msg.contacts[ee_idx].acceleration.z = state.contacts[ee_idx].acceleration(dwl::rbd::Z);

		// Forces
		msg.contacts[ee_idx].wrench.force.x = state.contacts[ee_idx].force(dwl::rbd::X);
		msg.contacts[ee_idx].wrench.force.y = state.contacts[ee_idx].force(dwl::rbd::Y);
		msg.contacts[ee_idx].wrench.force.z = state.contacts[ee_idx].force(dwl::rbd::Z);
	}
}


void ModeInvariantWholeBodyPlanner::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
	// Getting the floating system information
	dwl::model::FloatingBaseSystem system = planning_.getDynamicalSystem()->getFloatingBaseSystem();

	// Setting the base and joint information
	current_state_.base_pos.setZero();
	current_state_.base_vel.setZero();
	current_state_.base_acc.setZero();
	dwl::urdf_model::JointID joints = system.getJoints();
	for (unsigned int jnt_idx = 0; jnt_idx < system.getSystemDoF(); jnt_idx++) {
		std::string joint_name = msg->name[jnt_idx];

		dwl::urdf_model::JointID::iterator joint_it = joints.find(joint_name);
		if (joint_it != joints.end()) {
			unsigned int joint_id = joint_it->second - system.getFloatingBaseDoF();

			current_state_.joint_pos(joint_id) = msg->position[jnt_idx];
			current_state_.joint_vel(joint_id) = msg->velocity[jnt_idx];
			current_state_.joint_acc(joint_id) = 0.;
			current_state_.joint_eff(joint_id) = msg->effort[jnt_idx];
		} else {
			for (unsigned int base_idx = 0; base_idx < 6; base_idx++) {
				dwl::rbd::Coords6d base_coord = dwl::rbd::Coords6d(base_idx);
				dwl::model::FloatingBaseJoint base_joint = system.getFloatingBaseJoint(base_coord);

				if (base_joint.active) {
					if (base_joint.name == joint_name) {
						current_state_.base_pos(base_coord) = msg->position[jnt_idx];
						current_state_.base_vel(base_coord) = msg->velocity[jnt_idx];
						current_state_.base_acc(base_coord) = 0.;
					}
				}
			}
		}
	}

	new_current_state_ = true;
}

} //@namespace dwl_planners




int main(int argc, char **argv)
{
	ros::init(argc, argv, "mode_invariant_whole_body_planner");

	dwl_planners::ModeInvariantWholeBodyPlanner planner;

	planner.init();
	ros::spinOnce();

	try {
		ros::Rate loop_rate(0.25);

		while (ros::ok()) {
			if (planner.compute()) {
				planner.publishWholeBodyTrajectory();
				return 0;
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch (std::runtime_error& e) {
		ROS_ERROR("mode_invariant_whole_body_planner exception: %s", e.what());
		return -1;
	}

	return 0;
}
