#include <dwl_terrain/DefaultFlatTerrain.h>



namespace dwl_terrain
{

DefaultFlatTerrain::DefaultFlatTerrain(ros::NodeHandle node) : node_(node),
		center_x_(0.), center_y_(0.), width_(0.), length_(0.), yaw_(0.),
		resolution_(0.), height_(0.), world_frame_("world")
{
	flat_terrain_pub_  = private_node_.advertise<sensor_msgs::PointCloud2>("topic_output", 1);
	node_.param("world_frame", world_frame_, world_frame_);
	node_.param("center_x", center_x_, center_x_);
	node_.param("center_y", center_y_, center_y_);
	node_.param("width", width_, width_);
	node_.param("length", length_, length_);
	node_.param("yaw", yaw_, yaw_);
	node_.param("height", height_, height_);
	node_.param("resolution", resolution_, resolution_);
}


DefaultFlatTerrain::~DefaultFlatTerrain()
{

}


void DefaultFlatTerrain::setFlatTerrain()
{
	PointCloud pcl_msg;
	pcl_msg.header.frame_id = world_frame_;
	for (double xi = 0; xi < width_ / 2; xi += resolution_) {
		for (int sx = -1; sx <= 1; sx += 2) {
			for (double yi = 0; yi < length_ / 2; yi += resolution_) {
				for (int sy = -1; sy <= 1; sy += 2) {
					double x = sx * xi * cos(yaw_) - sy * yi * sin(yaw_) + center_x_;
					double y = sx * xi * sin(yaw_) + sy * yi * cos(yaw_) + center_y_;

					pcl_msg.push_back(pcl::PointXYZ(x, y, height_));
				}
			}
		}
	}

	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg(pcl_msg, cloud);

	flat_terrain_pub_.publish(cloud);
}

} //@namespace dwl_terrain



int main(int argc, char **argv)
{
	ros::init(argc, argv, "default_flat_terrain");

	ros::NodeHandle node("~");
	dwl_terrain::DefaultFlatTerrain default_flat_terrain(node);

	ros::spinOnce();

	try {
		ros::Rate loop_rate(100);
		while(ros::ok()) {
			default_flat_terrain.setFlatTerrain();
			ros::spinOnce();
			loop_rate.sleep();
		}
	} catch(std::runtime_error& e) {
		ROS_ERROR("reward_map_server exception: %s", e.what());
		return -1;
	}

	return 0;
}
