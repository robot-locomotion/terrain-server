#include <dwl_terrain/TerrainMapInterface.h>


namespace dwl_terrain
{

TerrainMapInterface::TerrainMapInterface() : new_msg_(false)
{
	ros::NodeHandle node;
	terrain_clt_ =
			node.serviceClient<dwl_terrain::TerrainData>("/terrain_data");
}


TerrainMapInterface::~TerrainMapInterface()
{

}


void TerrainMapInterface::init(ros::NodeHandle node)
{
	sub_ = node.subscribe<dwl_terrain::TerrainMap> ("/terrain_map", 1,
			&TerrainMapInterface::callback, this, ros::TransportHints().tcpNoDelay());
}


bool TerrainMapInterface::getTerrainMap(dwl::TerrainData& map)
{
	// Checks if there is a new terrain map message
	if (new_msg_) {
		// Setting the terrain map to be updated
		map_msg_ = *map_buffer_.readFromRT();
		new_msg_ = false;

		// Getting the number of cells
		unsigned int num_cells = map_msg_.cell.size();
		terrain_map_.data.resize(num_cells);

		// Setting up the terrain resolution
		terrain_map_.plane_size = map_msg_.plane_size;
		terrain_map_.height_size = map_msg_.height_size;
		dwl::environment::SpaceDiscretization model(terrain_map_.plane_size);
		model.setEnvironmentResolution(terrain_map_.height_size, false);

		// Converting the messages to dwl::TerrainMap format
		dwl::TerrainCell cell;
		for (unsigned int i = 0; i < num_cells; i++) {
			// Filling the terrain values per every cell
			cell.key.x = map_msg_.cell[i].key_x;
			cell.key.y = map_msg_.cell[i].key_y;
			cell.key.z = map_msg_.cell[i].key_z;
			cell.cost = map_msg_.cell[i].cost;
			model.keyToCoord(cell.height, cell.key.z, false);
			cell.normal =
					Eigen::Vector3d(map_msg_.cell[i].normal.x,
									map_msg_.cell[i].normal.y,
									map_msg_.cell[i].normal.z);

			// Adding the terrain cell to the queue
			terrain_map_.data[i] = cell;
		}
		map = terrain_map_;

		return true;
	}

	map = terrain_map_;

	return false;
}


const dwl::TerrainCell& TerrainMapInterface::getTerrainData(const Eigen::Vector2d& position)
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
		ROS_ERROR("Failed to call service terrain_data");
		terrain_cell_.cost = 0.;
		terrain_cell_.height = 0.;
		terrain_cell_.normal = Eigen::Vector3d::UnitZ();
		return terrain_cell_;
	}

	return terrain_cell_;
}


double TerrainMapInterface::getTerrainCost(const Eigen::Vector2d& position)
{
	return getTerrainData(position).cost;
}


double TerrainMapInterface::getTerrainHeight(const Eigen::Vector2d& position)
{
	return getTerrainData(position).height;
}


Eigen::Vector3d TerrainMapInterface::getTerrainNormal(const Eigen::Vector2d& position)
{
	return getTerrainData(position).normal;
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
