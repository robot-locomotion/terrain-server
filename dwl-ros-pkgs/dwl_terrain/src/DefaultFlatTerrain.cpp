#include <dwl_terrain/DefaultFlatTerrain.h>


namespace dwl_terrain
{

DefaultFlatTerrain::DefaultFlatTerrain(ros::NodeHandle node) : node_(node),
		n_rectangles_(1), world_frame_("world")
{
	flat_terrain_pub_  = private_node_.advertise<sensor_msgs::PointCloud2>("topic_output", 1);

	node_.param("position/x", position_(dwl::rbd::X), 0.);
	node_.param("position/y", position_(dwl::rbd::Y), 0.);
	node_.param("position/z", position_(dwl::rbd::Z), 0.);

	// Reading the number of elements
	node_.param("rectangles", n_rectangles_, n_rectangles_);

	// Reading the rectangles properties
	rectangles_.resize(n_rectangles_);
	for (int k = 0; k < n_rectangles_; k++) {
		std::string ns_name = "rectangle_" + std::to_string(k+1);

		node_.param(ns_name + "/center_x",
					rectangles_[k].center_x,
					rectangles_[k].center_x);
		node_.param(ns_name + "/center_y",
					rectangles_[k].center_y,
					rectangles_[k].center_y);
		node_.param(ns_name + "/width",
					rectangles_[k].width,
					rectangles_[k].width);
		node_.param(ns_name + "/length",
					rectangles_[k].length,
					rectangles_[k].length);
		node_.param(ns_name + "/yaw",
					rectangles_[k].yaw,
					rectangles_[k].yaw);
		node_.param(ns_name + "/height",
					rectangles_[k].height,
					rectangles_[k].height);
		node_.param(ns_name + "/resolution",
					rectangles_[k].resolution,
					rectangles_[k].resolution);
	}
}


DefaultFlatTerrain::~DefaultFlatTerrain()
{

}


void DefaultFlatTerrain::setFlatTerrain()
{
	PointCloud pcl_msg;
	pcl_msg.header.frame_id = world_frame_;

	for (int k = 0; k < n_rectangles_; k++) {
		for (double xi = 0; xi < rectangles_[k].length / 2;
				xi += rectangles_[k].resolution) {
			for (int sx = -1; sx <= 1; sx += 2) {
				for (double yi = 0; yi < rectangles_[k].width / 2;
						yi += rectangles_[k].resolution) {
					for (int sy = -1; sy <= 1; sy += 2) {
						double x = sx * xi * cos(rectangles_[k].yaw) -
								sy * yi * sin(rectangles_[k].yaw) +
								rectangles_[k].center_x + position_(dwl::rbd::X);
						double y = sx * xi * sin(rectangles_[k].yaw) +
								sy * yi * cos(rectangles_[k].yaw) +
								rectangles_[k].center_y + position_(dwl::rbd::Y);

						pcl_msg.push_back(pcl::PointXYZ(x, y, rectangles_[k].height + position_(dwl::rbd::Z)));
					}
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
