#ifndef DWL_TERRAIN__REWARD_MAP_SUBSCRIBER__H
#define DWL_TERRAIN__REWARD_MAP_SUBSCRIBER__H

#include <ros/ros.h>
#include <dwl/utils/EnvironmentRepresentation.h>
#include <dwl_terrain/RewardMap.h>


namespace dwl_terrain
{

class RewardMapSubscriber
{
	public:
		/** @brief Constructor function */
		RewardMapSubscriber();

		/** @brief Destructor function */
		~RewardMapSubscriber();

		/**
		 * @brief Creates a real-time subscriber of reward map.
		 * The name of the topic is defined as node_ns/reward_map
		 * @param ros::NodeHandle ROS node handle used by the subscription
		 */
		void init(ros::NodeHandle node);

		/**
		 * @brief Updates the vector of reward cells
		 * @param dwl::RewardCells& Vector of reward cells
		 */
		void update(dwl::RewardCells& reward_map);


	private:
		/**
		 * @brief Callback method when the reward map message arrives
		 * @param const dwl_terrain::RewardMapConstPtr& Reward map message
		 */
		void callback(const dwl_terrain::RewardMapConstPtr& msg);

		/** @brief Reward map subscriber */
		ros::Subscriber sub_;

		/** @brief Realtime buffer for the motion plan message */
		realtime_tools::RealtimeBuffer<dwl_terrain::RewardMap> map_buffer_;

		/** @brief Reward map message */
		dwl_terrain::RewardMap reward_msg_;

		/** @brief Reward map (or cells) */
		dwl::RewardCells reward_map_;

		/** @brief Indicates if there is a new motion plan available */
		bool new_msg_;
};

} //@namespace dwl_terrain


#endif
