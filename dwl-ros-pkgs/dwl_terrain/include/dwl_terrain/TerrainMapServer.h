#ifndef DWL_TERRAIN__TERRAIN_MAP_SERVER___H
#define DWL_TERRAIN__TERRAIN_MAP_SERVER___H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/math/Utils.h>

#include <dwl/environment/SlopeFeature.h>
#include <dwl/environment/HeightDeviationFeature.h>
#include <dwl/environment/CurvatureFeature.h>
#include <dwl/utils/Orientation.h>

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <dwl_terrain/TerrainMap.h>
#include <dwl_terrain/TerrainCell.h>
#include <std_srvs/Empty.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <dwl/environment/TerrainMapping.h>


namespace dwl_terrain
{

/**
 * @class TerrainMapServer
 * @brief Class for building terrain map
 */
class TerrainMapServer
{
	public:
		/** @brief Constructor function */
		TerrainMapServer(ros::NodeHandle node = ros::NodeHandle("~"));

		/** @brief Destructor function */
		~TerrainMapServer();

		/** @brief Initialization of the terrain map server */
		bool init();

		/**
		 * @brief Callback function when it arrives a octomap message
		 * @param const octomap_msgs::Octomap::ConstPtr& msg Octomap message
		 */
		void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

		/** @brief Resets the terrain map */
		bool reset(std_srvs::Empty::Request& req,
				   std_srvs::Empty::Response& resp);

		/** @brief Publishes a terrain map */
		void publishTerrainMap();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Private ROS node handle */
		ros::NodeHandle private_node_;

		/** @brief Pointer to the terrain mapping class */
		dwl::environment::TerrainMapping terrain_map_;

		/** @brief Terrain map publisher */
		ros::Publisher map_pub_;

		/** @brief Octomap subscriber */
		message_filters::Subscriber<octomap_msgs::Octomap>* octomap_sub_;

		/** @brief TF and octomap subscriber */
		tf::MessageFilter<octomap_msgs::Octomap>* tf_octomap_sub_;

		/** @brief Reset service */
		ros::ServiceServer reset_srv_;

		/** @brief Terrain map message */
		dwl_terrain::TerrainMap map_msg_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

		/** @brief Base frame */
		std::string base_frame_;

		/** @brief World frame */
		std::string world_frame_;

		/** @brief Indicates if it was computed new information of the terrain map */
		bool new_information_;
};

} //@namespace dwl_terrain
#endif
