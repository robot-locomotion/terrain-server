#ifndef TERRAIN_SERVER__FEATURE__HEIGHT_DEVIATION_FEATURE__H
#define TERRAIN_SERVER__FEATURE__HEIGHT_DEVIATION_FEATURE__H

#include <dwl/environment/Feature.h>


namespace terrain_server
{

namespace feature
{

/**
 * @class HeightDeviationFeature
 * @brief Class for solving the reward value of a height deviation feature
 */
class HeightDeviationFeature : public dwl::environment::Feature
{
	public:
		/** @brief Constructor function */
		HeightDeviationFeature(double flat_height_deviation, double max_height_deviation,
				double min_allowed_height = -std::numeric_limits<double>::max());

		/** @brief Destructor function */
		~HeightDeviationFeature();

		/**
		 * @brief Compute the cost value given a terrain information
		 * @param double& Cost value
		 * @param const Terrain& Information of the terrain
		 */
		void computeCost(double& cost_value,
						 const dwl::Terrain& terrain_info);


	private:
		/** @brief Flat height deviation */
		double flat_height_deviation_;

		/** @brief Maximum height deviation */
		double max_height_deviation_;

		/** @brief Minimum allowed height */
		double min_allowed_height_;
};

} //@namespace feature
} //@namespace terrain_server

#endif
