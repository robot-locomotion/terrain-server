#include <dwl_terrain/TerrainMapInterface.h>


namespace dwl_terrain
{

TerrainMapInterface::TerrainMapInterface() : new_msg_(false),
		is_terrain_data_(false)
{
	ros::NodeHandle node;
	terrain_clt_ =
			node.serviceClient<dwl_terrain::TerrainData>("/terrain_map/data");
	reset_clt_ =
			node.serviceClient<std_srvs::Empty>("/terrain_map/reset");
	terrain_map_.reset(new dwl::environment::TerrainMap());
}


TerrainMapInterface::~TerrainMapInterface()
{

}


void TerrainMapInterface::init(ros::NodeHandle node)
{
	sub_ = node.subscribe<dwl_terrain::TerrainMap> ("/terrain_map", 1,
			&TerrainMapInterface::callback, this, ros::TransportHints().tcpNoDelay());
}


void TerrainMapInterface::updateTerrainMap()
{
	// Checks if there is a new terrain map message
	if (new_msg_) {
		// Setting the terrain map to be updated
		map_msg_ = *map_buffer_.readFromRT();
		new_msg_ = false;

		// Getting the number of cells
		unsigned int num_cells = map_msg_.cell.size();
		terrain_data_.data.resize(num_cells);

		// Setting up the terrain resolution
		terrain_data_.plane_size = map_msg_.plane_size;
		terrain_data_.height_size = map_msg_.height_size;

		// Converting the messages to dwl::TerrainMap format
		dwl::TerrainCell cell;
		for (unsigned int i = 0; i < num_cells; i++) {
			// Filling the terrain values per every cell
			cell.key.x = map_msg_.cell[i].key_x;
			cell.key.y = map_msg_.cell[i].key_y;
			cell.key.z = map_msg_.cell[i].key_z;
			cell.cost = map_msg_.cell[i].cost;
			cell.normal =
					Eigen::Vector3d(map_msg_.cell[i].normal.x,
									map_msg_.cell[i].normal.y,
									map_msg_.cell[i].normal.z);

			// Adding the terrain cell to the queue
			terrain_data_.data[i] = cell;
		}

		terrain_map_->setTerrainMap(terrain_data_);

		// We have an initial map
		if (!is_terrain_data_)
			is_terrain_data_ = true;
	}
}


bool TerrainMapInterface::resetTerrainMap()
{
	std_srvs::Empty srv;
	if (!reset_clt_.call(srv)) {
		ROS_ERROR("Failed to call service /terrain_map/reset");
		return false;
	}

	return true;
}


bool TerrainMapInterface::getTerrainMap(dwl::TerrainData& map)
{
	updateTerrainMap();
	if (is_terrain_data_) {
		map = terrain_data_;
		return true;
	} else
		return false;
}


const dwl::TerrainCell& TerrainMapInterface::requestTerrainData(const Eigen::Vector2d& position)
{
	dwl_terrain::TerrainData srv;
	srv.request.position.x = position(dwl::rbd::X);
	srv.request.position.y = position(dwl::rbd::Y);

	if (terrain_clt_.call(srv)) {
		terrain_cell_.cost = srv.response.cost;
		terrain_cell_.height = srv.response.height;
		terrain_cell_.normal =
				Eigen::Vector3d(srv.response.normal.x,
								srv.response.normal.y,
								srv.response.normal.z);
	} else {
		ROS_ERROR("Failed to call service terrain_map/data");
		terrain_cell_.cost = 0.;
		terrain_cell_.height = 0.;
		terrain_cell_.normal = Eigen::Vector3d::UnitZ();
	}

	return terrain_cell_;
}


const double& TerrainMapInterface::requestTerrainCost(const Eigen::Vector2d& position)
{
	return requestTerrainData(position).cost;
}


const double& TerrainMapInterface::requestTerrainHeight(const Eigen::Vector2d& position)
{
	return requestTerrainData(position).height;
}


const Eigen::Vector3d& TerrainMapInterface::requestTerrainNormal(const Eigen::Vector2d& position)
{
	return requestTerrainData(position).normal;
}


bool TerrainMapInterface::getTerrainData(dwl::TerrainCell& cell,
										 const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainData(cell, position);
}


const dwl::TerrainCell& TerrainMapInterface::getTerrainData(const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainData(position);
}


bool TerrainMapInterface::getTerrainCost(double& cost,
										 const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainCost(cost, position);
}


const double& TerrainMapInterface::getTerrainCost(const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainCost(position);
}


bool TerrainMapInterface::getTerrainHeight(double& height,
										   const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainHeight(height, position);
}


double TerrainMapInterface::getTerrainHeight(const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainHeight(position);
}


bool TerrainMapInterface::getTerrainNormal(Eigen::Vector3d& normal,
										   const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainNormal(normal, position);
}


const Eigen::Vector3d& TerrainMapInterface::getTerrainNormal(const Eigen::Vector2d& position) const
{
	return terrain_map_->getTerrainNormal(position);
}


void TerrainMapInterface::callback(const dwl_terrain::TerrainMapConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	map_buffer_.writeFromNonRT(*msg);

	new_msg_ = true;
}

} //@namespace dwl_terrain
