#ifndef TERRAIN_SERVER__TERRAIN_MAPPING__H
#define TERRAIN_SERVER__TERRAIN_MAPPING__H

#include <dwl/environment/TerrainMap.h>
#include <dwl/environment/Feature.h>
#include <dwl/utils/utils.h>

#include <octomap/octomap.h>


namespace terrain_server
{

/**
 * @class TerrainMapping
 * @brief Abstract class for building the terrain map
 */
class TerrainMapping : public dwl::environment::TerrainMap
{
	public:
		/** @brief Constructor function */
		TerrainMapping();

		/** @brief Destructor function */
		~TerrainMapping();

		/**
		 * @brief Adds a feature of the terrain map
		 * @param Feature* the pointer of the feature to add
		 */
		void addFeature(dwl::environment::Feature* feature);

		/**
		 * @brief Removes a feature of the terrain map
		 * @param std::string the name of the feature to remove
		 */
		void removeFeature(std::string feature_name);

		/**
		 * @brief Abstract method for computing the terrain map according
		 * the robot position and model of the terrain
		 * @param octomap::OcTree* The model of the environment
		 * @param const Eigen::Vector4d& The position of the robot and the yaw angle
		 */
		void compute(octomap::OcTree* model,
					 const Eigen::Vector4d& robot_state);

		/**
		 * @brief Computes the terrain data given the voxel map
		 * and the key of the topmost cell of a certain position of the grid
		 * @param octomap::OcTree* Pointer to the octomap model of the environment
		 * @param const octomap::OcTreeKey& The key of the topmost cell of a
		 * certain position of the grid
		 */
		void computeTerrainData(octomap::OcTree* octomap,
								const octomap::OcTreeKey& heightmap_key);

		/**
		 * @brief Removes terrain values outside the interest region
		 * @param const Eigen::Vector3d& State of the robot, i.e. 3D position
		 * and yaw orientation
		 */
		void removeTerrainOutsideInterestRegion(const Eigen::Vector3d& robot_state);

		/**
		 * @brief Sets a interest region
		 * @param double Radius along the x-axis
		 * @param double Radius along the x-axis
		 */
		void setInterestRegion(double radius_x,
							   double radius_y);

		/**
		 * @brief Adds a new search area around the current position of the robot
		 * @param double Minimum Cartesian position along the x-axis
		 * @param double Maximum Cartesian position along the x-axis
		 * @param double Minimum Cartesian position along the y-axis
		 * @param double Maximum Cartesian position along the y-axis
		 * @param double Minimum Cartesian position along the z-axis
		 * @param double Maximum Cartesian position along the z-axis
		 * @param double Resolution of the grid
		 */
		void addSearchArea(double min_x, double max_x,
						   double min_y, double max_y,
						   double min_z, double max_z,
						   double grid_size);

		/**
		 * @brief Sets the neighboring area for computing physical properties
		 * of the terrain
		 * @param int Number of left neighbors
		 * @param int Number of right neighbors
		 * @param int Number of left neighbors
		 * @param int Number of right neighbors
		 * @param int Number of bottom neighbors
		 * @param int Number of top neighbors
		 */
		void setNeighboringArea(int back_neighbors, int front_neighbors,
								int left_neighbors, int right_neighbors,
								int bottom_neighbors, int top_neighbors);


	private:
		/** @brief Vector of pointers to the Feature class */
		std::vector<dwl::environment::Feature*> features_;

		/** @brief Terrain information */
		dwl::Terrain terrain_info_;

		/** @brief Indicates if it was added a feature */
		bool is_added_feature_;

		/** @brief Indicates if it was added a search area */
		bool is_added_search_area_;

		/** @brief Interest area */
		double interest_radius_x_, interest_radius_y_;

		/** @brief Vector of search areas */
		std::vector<dwl::SearchArea> search_areas_;

		/** @brief Object of the NeighboringArea struct that defines the
		 *  neighboring area */
		dwl::NeighboringArea neighboring_area_;

		/** @brief Defines if it is using the mean of the cloud */
		bool using_cloud_mean_;

		/** @brief Depth of the octomap */
		int depth_;
};

} //@namespace terrain_server

#endif
