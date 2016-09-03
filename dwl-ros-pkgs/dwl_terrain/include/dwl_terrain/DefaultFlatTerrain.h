#ifndef DWL_TERRAIN__DEFAULT_FLAT_TERRAIN___H
#define DWL_TERRAIN__DEFAULT_FLAT_TERRAIN___H

#include <dwl/utils/RigidBodyDynamics.h>
#include <dwl/utils/Orientation.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <iostream>


namespace dwl_terrain
{

struct Rectangle
{
	Rectangle() : center_x(0.), center_y(0.), width(0.), length(0.),
			yaw(0.), resolution(0.), height(0.) {}

	double center_x;
	double center_y;
	double width;
	double length;
	double yaw;
	double resolution;
	double height;
};

/**
 * @class DefaultFlatTerrain
 * @brief Class for building default flat terrain around the robot
 */
class DefaultFlatTerrain
{
	public:
		/** @brief Constructor function */
		DefaultFlatTerrain(ros::NodeHandle node);

		/** @brief Destructor function */
		~DefaultFlatTerrain();

		/** @brief Sets the flat terrain given the desired properties */
		void setFlatTerrain();


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Private ROS node handle */
		ros::NodeHandle private_node_;

		/** @brief Flat terrain point cloud publisher */
		ros::Publisher flat_terrain_pub_;

		/** @brief Number of elements */
		int n_rectangles_;

		std::vector<Rectangle> rectangles_;

		/** @brief World frame name */
		std::string world_frame_;

		/** @brief Position of the terrain w.r.t. the world frame */
		Eigen::Vector3d position_;

		/** Point cloud data that defines the flat terrain */
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
};

} //@namespace dwl_terrain

#endif
