#ifndef TERRAIN_SERVER__TERRAIN_MAP_SERVER___H
#define TERRAIN_SERVER__TERRAIN_MAP_SERVER___H

#include <ros/ros.h>

#include <dwl/environment/SpaceDiscretization.h>
#include <dwl/utils/Orientation.h>

#include <terrain_server/TerrainMapping.h>
#include <terrain_server/feature/SlopeFeature.h>
#include <terrain_server/feature/HeightDeviationFeature.h>
#include <terrain_server/feature/CurvatureFeature.h>


#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <terrain_server/TerrainMap.h>
#include <terrain_server/TerrainCell.h>
#include <std_srvs/Empty.h>
#include <terrain_server/TerrainData.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>



namespace terrain_server
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

		/** @brief Gets the terrain data */
		bool getTerrainData(terrain_server::TerrainData::Request& req,
							terrain_server::TerrainData::Response& res);

		/** @brief Publishes a terrain map */
		void publishTerrainMap();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Private ROS node handle */
		ros::NodeHandle private_node_;

		/** @brief Pointer to the terrain mapping class */
		terrain_server::TerrainMapping terrain_map_;

		/**
		 *  @brief Object of the SpaceDiscretization class for defining the
		 *  conversion routines for the terrain cost-map */
		dwl::environment::SpaceDiscretization terrain_discretization_;

		/** @brief Terrain map publisher */
		ros::Publisher map_pub_;

		/** @brief Octomap subscriber */
		message_filters::Subscriber<octomap_msgs::Octomap>* octomap_sub_;

		/** @brief TF and octomap subscriber */
		tf::MessageFilter<octomap_msgs::Octomap>* tf_octomap_sub_;

		/** @brief Reset service */
		ros::ServiceServer reset_srv_;

		/** @bief Get the terrain data service */
		ros::ServiceServer terrain_data_srv_;

		/** @brief Terrain map message */
		terrain_server::TerrainMap map_msg_;

		/** @brief TF listener */
		tf::TransformListener tf_listener_;

		/** @brief Base frame */
		std::string base_frame_;

		/** @brief World frame */
		std::string world_frame_;

		/** @brief Indicates if it was computed an initial terrain map */
		bool initial_map_;
};

} //@namespace terrain_server
#endif
