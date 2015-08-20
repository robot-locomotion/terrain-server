#ifndef DWL_PLANNERS__CONSTRAINED_WHOLE_BODY_PLANNER__H
#define DWL_PLANNERS__CONSTRAINED_WHOLE_BODY_PLANNER__H

#include <ros/ros.h>

// Locomotion headers
#include <locomotion/WholeBodyTrajectoryOptimization.h>
#include <model/ConstrainedDynamicalSystem.h>
#include <model/TerminalStateTrackingEnergyCost.h>
#include <model/IntegralStateTrackingEnergyCost.h>
#include <model/IntegralControlEnergyCost.h>
#include <solver/IpoptNLP.h>

// Messages headers
#include <dwl_msgs/WholeBodyTrajectory.h>
#include <sensor_msgs/JointState.h>


namespace dwl_planners
{

class ConstrainedWholeBodyPlanner
{
	public:
		/** @brief Constructor function */
		ConstrainedWholeBodyPlanner(ros::NodeHandle node = ros::NodeHandle("~"));

		/** @brief Destructor function */
		~ConstrainedWholeBodyPlanner();

		/** @brief Initializes the constrained whole-body planner */
		void init();

		/** @brief Computes the whole-body trajectory */
		bool compute();

		/** @brief Publishes the planned whole-body trajectory */
		void publishWholeBodyTrajectory();


	private:
		/**
		 * @brief Writes the whole-body state message from a locomotion state
		 * @param dwl_msgs::WholeBodyState& Whole-body state message
		 * @param const dwl::LocomotionState& Locomotion state
		 */
		void writeWholeBodyStateMessage(dwl_msgs::WholeBodyState& msg,
										const dwl::LocomotionState& state);

		/**
		 * @brief Callback method when the joint state message arrives
		 * @param const sensor_msgs::JointStateConstPtr& Robot state message
		 */
		void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);


		/** @brief Ros node handle */
		ros::NodeHandle node_;

		/** @brief Privated Ros node handle */
		ros::NodeHandle privated_node_;

		/** Motion plan publisher */
		ros::Publisher motion_plan_pub_;

		/** @brief TF listener */
		ros::Subscriber robot_state_sub_;

		/** @brief Whole-body trajectory optimization */
		dwl::locomotion::WholeBodyTrajectoryOptimization planning_;

		/** @brief Current whole-body state */
		dwl::LocomotionState current_state_;

		/** @brief Desired whole-body state */
		dwl::LocomotionState desired_state_;

		/** @brief Interpolation time */
		double interpolation_time_;

		/** @brief Allowed computation time */
		double computation_time_;

		/** @brief Whole-body trajectory message */
		dwl_msgs::WholeBodyTrajectory robot_trajectory_msg_;

		/** @brief Returns true when a new current state message has arrived */
		bool new_current_state_;
};

} //@namespace dwl_planners

#endif
