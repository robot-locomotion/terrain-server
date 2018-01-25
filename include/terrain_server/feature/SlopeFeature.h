#ifndef TERRAIN_SERVER__FEATURE__SLOPE_FEATURE__H
#define TERRAIN_SERVER__FEATURE__SLOPE_FEATURE__H

#include <dwl/environment/Feature.h>


namespace terrain_server
{

namespace feature
{

/**
 * @class SlopeFeature
 * @brief Class for computing the cost value of the slope feature
 */
class SlopeFeature : public dwl::environment::Feature
{
	public:
		/** @brief Constructor function */
		SlopeFeature();

		/** @brief Destructor function */
		~SlopeFeature();

		/**
		 * @brief Compute the cost value given a terrain information
		 * @param double& Cost value
		 * @param const Terrain& Information of the terrain
		 */
		void computeCost(double& cost_value,
						 const dwl::Terrain& terrain_info);

	private:
		/** @brief Threshold that specify the flat condition */
		double flat_threshold_;

		/** @brief Threshold that indicates a very (bad) steep condition */
		double steep_threshold_;
};


} //@namespace feature
} //@namespace terrain_server

#endif
