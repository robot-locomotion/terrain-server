#include <dwl_terrain/RewardMapSubscriber.h>


namespace dwl_terrain
{

RewardMapSubscriber::RewardMapSubscriber() : new_msg_(false)
{

}


RewardMapSubscriber::~RewardMapSubscriber()
{

}


void RewardMapSubscriber::init(ros::NodeHandle node)
{
	sub_ = node.subscribe<dwl_terrain::RewardMap> ("reward_map", 1,
			&RewardMapSubscriber::callback, this, ros::TransportHints().tcpNoDelay());
}


bool RewardMapSubscriber::getRewardMap(dwl::RewardCells& reward_map)
{
	// Checks if there is a new reward message
	if (new_msg_) {
		// Setting the reward map to be updated
		reward_msg_ = *map_buffer_.readFromRT();
		new_msg_ = false;

		// Getting the number of cells
		unsigned int num_cells = reward_msg_.cell.size();
		reward_map_.resize(num_cells);

		// Converting the messages to reward_map format
		dwl::RewardCell reward_cell;
		for (unsigned int i = 0; i < num_cells; i++) {
			// Filling the reward per every cell
			reward_cell.key.x = reward_msg_.cell[i].key_x;
			reward_cell.key.y = reward_msg_.cell[i].key_y;
			reward_cell.key.z = reward_msg_.cell[i].key_z;
			reward_cell.reward = reward_msg_.cell[i].reward;
			reward_cell.plane_size = reward_msg_.plane_size;
			reward_cell.height_size = reward_msg_.height_size;

			// Adding the reward cell to the queue
			reward_map_[i] = reward_cell;
		}

		return true;
	}

	return false;
}


void RewardMapSubscriber::callback(const dwl_terrain::RewardMapConstPtr& msg)
{
	// the writeFromNonRT can be used in RT, if you have the guarantee that
	// no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	// there is only one single rt thread
	map_buffer_.writeFromNonRT(*msg);

	new_msg_ = true;
}


} //@namespace dwl_terrain
