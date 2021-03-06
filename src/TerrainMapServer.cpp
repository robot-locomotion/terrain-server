#include <terrain_server/TerrainMapServer.h>


namespace terrain_server
{

TerrainMapServer::TerrainMapServer(ros::NodeHandle node) : private_node_(node),
		terrain_discretization_(0.04, 0.04, M_PI / 200),
		octomap_sub_(NULL),	tf_octomap_sub_(NULL), base_frame_("base_link"),
		world_frame_("world"), initial_map_(false)
{

}


TerrainMapServer::~TerrainMapServer()
{
	if (tf_octomap_sub_) {
		delete tf_octomap_sub_;
		tf_octomap_sub_ = NULL;
	}

	if (octomap_sub_) {
		delete octomap_sub_;
		octomap_sub_ = NULL;
	}
}


bool TerrainMapServer::init()
{
	// Getting the names of search areas
	XmlRpc::XmlRpcValue area_names;
	if (!private_node_.getParam("search_areas", area_names)) {
		ROS_ERROR("No search areas given in the namespace: %s.",
				private_node_.getNamespace().c_str());
	} else {
		// Evaluating the fetching information of the search areas
		if (area_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
			ROS_ERROR("Malformed search area specification.");
			return false;
		}

		double min_x, max_x, min_y, max_y, min_z, max_z, resolution;
		for (int i = 0; i < area_names.size(); i++) {
			private_node_.getParam((std::string) area_names[i] + "/min_x", min_x);
			private_node_.getParam((std::string) area_names[i] + "/max_x", max_x);
			private_node_.getParam((std::string) area_names[i] + "/min_y", min_y);
			private_node_.getParam((std::string) area_names[i] + "/max_y", max_y);
			private_node_.getParam((std::string) area_names[i] + "/min_z", min_z);
			private_node_.getParam((std::string) area_names[i] + "/max_z", max_z);
			private_node_.getParam((std::string) area_names[i] + "/resolution", resolution);

			// Adding the search areas
			terrain_map_.addSearchArea(min_x, max_x, min_y, max_y, min_z, max_z, resolution);
		}
	}

	// Getting the interest region, i.e. the information outside this region will be deleted
	double radius_x = 1, radius_y = 1;
	private_node_.getParam("interest_region/radius_x", radius_x);
	private_node_.getParam("interest_region/radius_y", radius_y);
	terrain_map_.setInterestRegion(radius_x, radius_y);

	// Getting the feature information
	bool enable_slope, enable_height_dev, enable_curvature;
	double weight;
	double default_weight = 1;
	private_node_.getParam("features/slope/enable", enable_slope);
	private_node_.getParam("features/height_deviation/enable", enable_height_dev);
	private_node_.getParam("features/curvature/enable", enable_curvature);

	// Adding the slope feature if it's enable
	if (enable_slope) {
		// Setting the weight feature
		private_node_.param("features/slope/weight", weight, default_weight);
		dwl::environment::Feature* slope_ptr = new terrain_server::feature::SlopeFeature();
		slope_ptr->setWeight(weight);

		// Adding the feature
		terrain_map_.addFeature(slope_ptr);
	}

	// Adding the height deviation feature if it's enable
	if (enable_height_dev) {
		// Setting the weight feature
		private_node_.param("features/height_deviation/weight",
							weight,	default_weight);
		double flat_height_deviation, max_height_deviation, min_allowed_height;
		private_node_.param("features/height_deviation/flat_height_deviation",
							flat_height_deviation, 0.01);
		private_node_.param("features/height_deviation/max_height_deviation",
							 max_height_deviation, 0.3);
		private_node_.param("features/height_deviation/min_allowed_height",
							 min_allowed_height, -std::numeric_limits<double>::max());
		dwl::environment::Feature* height_dev_ptr =
				new terrain_server::feature::HeightDeviationFeature(flat_height_deviation,
																	max_height_deviation,
																	min_allowed_height);
		height_dev_ptr->setWeight(weight);

		// Setting the neighboring area
		double size, resolution;
		private_node_.param("features/height_deviation/neighboring_area/square_size",
							size, 0.1);
		private_node_.param("features/height_deviation/neighboring_area/resolution",
							resolution, 0.04);
		height_dev_ptr->setNeighboringArea(-size, size, -size, size, resolution);

		// Adding the feature
		terrain_map_.addFeature(height_dev_ptr);
	}

	// Adding the curvature feature if it's enable
	if (enable_curvature) {
		// Setting the weight feature
		private_node_.param("features/curvature/weight", weight, default_weight);
		dwl::environment::Feature* curvature_ptr = new terrain_server::feature::CurvatureFeature();
		curvature_ptr->setWeight(weight);

		// Adding the feature
		terrain_map_.addFeature(curvature_ptr);
	}

	// Getting the base and world frame
	private_node_.param("base_frame", base_frame_, base_frame_);
	private_node_.param("world_frame", world_frame_, world_frame_);
	map_msg_.header.frame_id = world_frame_;

	// Declaring the subscriber to octomap and tf messages
	octomap_sub_ =
			new message_filters::Subscriber<octomap_msgs::Octomap>(
					node_, "octomap_binary", 5);
	tf_octomap_sub_ =
			new tf::MessageFilter<octomap_msgs::Octomap>(
					*octomap_sub_, tf_listener_, world_frame_, 5);
	tf_octomap_sub_->registerCallback(
			boost::bind(&TerrainMapServer::octomapCallback, this, _1));

	// Declaring the publisher of terrain map
	map_pub_ = node_.advertise<terrain_server::TerrainMap>("terrain_map", 1);

	reset_srv_ = private_node_.advertiseService("reset", &TerrainMapServer::reset, this);
	terrain_data_srv_ =
			private_node_.advertiseService("data", &TerrainMapServer::getTerrainData, this);


	return true;
}


void TerrainMapServer::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	// Creating a octree
	octomap::OcTree* octomap = NULL;
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

	if (tree) {
		octomap = dynamic_cast<octomap::OcTree*>(tree);
	}

	if (!octomap) {
		ROS_WARN("Failed to create octree structure");
		return;
	}

	// Setting the resolution of the gridmap
	terrain_map_.setResolution(octomap->getResolution(), false);

	// Getting the transformation between the world to robot frame
	tf::StampedTransform tf_transform;
	try {
		tf_listener_.lookupTransform(world_frame_,
									 base_frame_,
									 msg->header.stamp,
									 tf_transform);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	// Getting the robot state (3D position and yaw angle)
	Eigen::Vector4d robot_position = Eigen::Vector4d::Zero();
	robot_position(0) = tf_transform.getOrigin()[0];
	robot_position(1) = tf_transform.getOrigin()[1];
	robot_position(2) = tf_transform.getOrigin()[2];

	// Computing the yaw angle
	tf::Quaternion q = tf_transform.getRotation();
	double yaw =
			dwl::math::getYaw(dwl::math::getRPY(Eigen::Quaterniond(q.getW(),
																   q.getX(),
																   q.getY(),
																   q.getZ())));
	robot_position(3) = yaw;

	// Computing the terrain map
	timespec start_rt, end_rt;
	clock_gettime(CLOCK_REALTIME, &start_rt);
	terrain_map_.compute(octomap, robot_position);
	initial_map_ = true;
	publishTerrainMap();
	clock_gettime(CLOCK_REALTIME, &end_rt);
	double duration =
			(end_rt.tv_sec - start_rt.tv_sec) + 1e-9*(end_rt.tv_nsec - start_rt.tv_nsec);
	ROS_INFO("The duration of computation of terrain map is %f seg.", duration);
}


bool TerrainMapServer::reset(std_srvs::Empty::Request& req,
							std_srvs::Empty::Response& resp)
{
	initial_map_ = false;
	terrain_map_.reset();

	ros::ServiceClient client = 
		private_node_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

	if (!client.call(req, resp)) {
		ROS_ERROR("Failed to call service /octomap_server/reset");
		return false;
	}

	ROS_INFO("Reset terrain map");	
	return true;
}


bool TerrainMapServer::getTerrainData(terrain_server::TerrainData::Request& req,
									  terrain_server::TerrainData::Response& res)
{
	if (initial_map_) {
		Eigen::Vector2d position(req.position.x, req.position.y);
		dwl::TerrainCell cell = terrain_map_.getTerrainData(position);

		res.cost = cell.cost;
		res.height = cell.height;
		res.normal.x = cell.normal(dwl::rbd::X);
		res.normal.y = cell.normal(dwl::rbd::Y);
		res.normal.z = cell.normal(dwl::rbd::Z);

		return true;
	}
	
	return false;
}


void TerrainMapServer::publishTerrainMap()
{
	// Publishing the terrain map if there is at least one subscriber
	if (map_pub_.getNumSubscribers() > 0) {
		map_msg_.header.stamp = ros::Time::now();

		dwl::TerrainDataMap terrain_gridmap = terrain_map_.getTerrainDataMap();

		// Getting the terrain map resolutions
		map_msg_.plane_size = terrain_map_.getResolution(true);
		map_msg_.height_size = terrain_map_.getResolution(false);

		// Getting the number of cells
		unsigned int num_cells = terrain_gridmap.size();
		map_msg_.cell.resize(num_cells);

		// Converting the vertexes into a cell message
		terrain_server::TerrainCell cell;
		unsigned int idx = 0;
		for (dwl::TerrainDataMap::iterator vertex_iter = terrain_gridmap.begin();
				vertex_iter != terrain_gridmap.end();
				vertex_iter++)
		{
			dwl::TerrainCell terrain_cell = vertex_iter->second;

			cell.key_x = terrain_cell.key.x;
			cell.key_y = terrain_cell.key.y;
			cell.key_z = terrain_cell.key.z;
			cell.cost = terrain_cell.cost;
			cell.normal.x = terrain_cell.normal(dwl::rbd::X);
			cell.normal.y = terrain_cell.normal(dwl::rbd::Y);
			cell.normal.z = terrain_cell.normal(dwl::rbd::Z);
			map_msg_.cell[idx] = cell;

			idx++;
		}

		map_pub_.publish(map_msg_);

		// Deleting old information
		map_msg_.cell.clear();
	}
}

} //@namespace terrain_server



int main(int argc, char **argv)
{
	ros::init(argc, argv, "terrain_map_server");

	terrain_server::TerrainMapServer terrain_server;
	if (!terrain_server.init())
		return -1;

	ros::spin();

	return 0;
}
