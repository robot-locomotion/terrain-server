#include <dwl_terrain/TerrainMapSubscriber.h>


namespace dwl_terrain
{

TerrainMapSubscriber::TerrainMapSubscriber() : new_msg_(false)
{

}


TerrainMapSubscriber::~TerrainMapSubscriber()
{

}


void TerrainMapSubscriber::init(ros::NodeHandle node)
{
	sub_ = node.subscribe<dwl_terrain::TerrainMap> ("/terrain_map", 1,
			&TerrainMapSubscriber::callback, this, ros::TransportHints().tcpNoDelay());
}


bool TerrainMapSubscriber::getTerrainMap(dwl::TerrainData& map)
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

		// Converting the messages to dwl::TerrainMap format
		dwl::TerrainCell cell;
		for (unsigned int i = 0; i < num_cells; i++) {
			// Filling the terrain values per every cell
			cell.key.x = map_msg_.cell[i].key_x;
			cell.key.y = map_msg_.cell[i].key_y;
			cell.key.z = map_msg_.cell[i].key_z;
			cell.cost = map_msg_.cell[i].cost;

			// Adding the terrain cell to the queue
			terrain_map_.data[i] = cell;
		}
		map = terrain_map_;

		return true;
	}

	map = terrain_map_;

	return false;
}


void TerrainMapSubscriber::callback(const dwl_terrain::TerrainMapConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	map_buffer_.writeFromNonRT(*msg);

	new_msg_ = true;
}


} //@namespace dwl_terrain
