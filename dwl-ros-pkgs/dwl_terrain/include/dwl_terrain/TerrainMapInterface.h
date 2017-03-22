#ifndef DWL_TERRAIN__TERRAIN_MAP_INTERFACE__H
#define DWL_TERRAIN__TERRAIN_MAP_INTERFACE__H

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>

#include <dwl/environment/TerrainMap.h>
#include <dwl/environment/SpaceDiscretization.h>
#include <dwl/utils/RigidBodyDynamics.h>
#include <dwl/utils/EnvironmentRepresentation.h>
#include <dwl_terrain/TerrainMap.h>
#include <dwl_terrain/TerrainData.h>


namespace dwl_terrain
{

class TerrainMapInterface
{
	public:
		/** @brief Constructor function */
		TerrainMapInterface();

		/** @brief Destructor function */
		~TerrainMapInterface();

		/**
		 * @brief Creates a real-time subscriber of terrain map.
		 * The name of the topic is defined as node_ns/terrain_map
		 * @param ros::NodeHandle ROS node handle	 used by the subscription
		 */
		void init(ros::NodeHandle node);

		void updateTerrainMap();

		/**
		 * @brief Gets the vector of terrain cells
		 * @param dwl::TerrainData& Vector of terrain cells
		 */
		bool getTerrainMap(dwl::TerrainData& map);

		/** @brief These methods allows us to call the terrain map service
		 * and get the desired terrain data. Note that returns false
		 * if there is not available data, and in that case a default value is
		 * assigned */
		const dwl::TerrainCell& requestTerrainData(const Eigen::Vector2d& position);
		const double& requestTerrainCost(const Eigen::Vector2d& position);
		const double& requestTerrainHeight(const Eigen::Vector2d& position);
		const Eigen::Vector3d& requestTerrainNormal(const Eigen::Vector2d& position);

		/** @brief These methods allows us to get the data from the updated
		 * terrain map and get the desired terrain data. Note that returns false
		 * if there is not available data, and in that case a default value is
		 * assigned */
		bool getTerrainData(dwl::TerrainCell& cell,
							const Eigen::Vector2d& position) const;
		const dwl::TerrainCell& getTerrainData(const Eigen::Vector2d& position) const;
		bool getTerrainCost(double& cost,
							const Eigen::Vector2d& position) const;
		const double& getTerrainCost(const Eigen::Vector2d& position) const;
		bool getTerrainHeight(double& height,
							  const Eigen::Vector2d& position) const;
		double getTerrainHeight(const Eigen::Vector2d& position) const;
		bool getTerrainNormal(Eigen::Vector3d& normal,
							  const Eigen::Vector2d& position) const;
		const Eigen::Vector3d& getTerrainNormal(const Eigen::Vector2d& position) const;


	private:
		/**
		 * @brief Callback method when the terrain map message arrives
		 * @param const dwl_terrain::TerrainMapConstPtr& Terrain map message
		 */
		void callback(const dwl_terrain::TerrainMapConstPtr& msg);

		/** @brief Terrain map subscriber */
		ros::Subscriber sub_;

		/** @brief Realtime buffer for the terrain map message */
		realtime_tools::RealtimeBuffer<dwl_terrain::TerrainMap> map_buffer_;

		/** @brief The terrain map client */
		ros::ServiceClient terrain_clt_;

		/** @brief Terrain map message */
		dwl_terrain::TerrainMap map_msg_;

		/** @brief Terrain map (or cells) */
		std::shared_ptr<dwl::environment::TerrainMap> terrain_map_;
		dwl::TerrainCell terrain_cell_;
		dwl::TerrainData terrain_data_;

		/** @brief Indicates if there is a new terrain map available */
		bool new_msg_;

		/** @brief Indicates if there is a terrain map available */
		bool is_terrain_data_;
};

} //@namespace dwl_terrain


#endif
