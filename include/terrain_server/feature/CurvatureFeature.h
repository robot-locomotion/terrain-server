#ifndef TERRAIN_SERVER__FEATURE__CURVATURE_FEATURE__H
#define TERRAIN_SERVER__FEATURE__CURVATURE_FEATURE__H

#include <dwl/environment/Feature.h>


namespace terrain_server
{

namespace feature
{

/**
 * @class CurvatureFeature
 * @brief Class for computing the cost value of the curvature feature
 */
class CurvatureFeature : public dwl::environment::Feature
{
	public:
		/** @brief Constructor function */
		CurvatureFeature();

		/** @brief Destructor function */
		~CurvatureFeature();

		/**
		 * @brief Compute the cost value given a terrain information
		 * @param double& Cost value
		 * @param const Terrain& Information of the terrain
		 */
		void computeCost(double& cost_value,
						 const dwl::Terrain& terrain_info);

	private:
		/** @brief Threshold that specify the positive condition */
		double positive_threshold_;

		/** @brief Threshold that indicates a very (bad) condition */
		double negative_threshold_;
};

} //@namespace feature
} //@namespace terrain_server

#endif
