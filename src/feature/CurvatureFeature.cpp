#include <terrain_server/feature/CurvatureFeature.h>
#include <Eigen/Dense>


namespace terrain_server
{

namespace feature
{

CurvatureFeature::CurvatureFeature() :
		positive_threshold_(6.0), negative_threshold_(-6.0)
{
	name_ = "Curvature";
}

CurvatureFeature::~CurvatureFeature()
{

}


void CurvatureFeature::computeCost(double& cost_value,
								   const dwl::Terrain& terrain_info)
{
	double curvature = terrain_info.curvature;

	// The worse condition
	if (curvature * 10000 > 9) {
		cost_value = max_cost_;
		return;
	}

	if (curvature > positive_threshold_)
		cost_value = 0.;
	else if (curvature < negative_threshold_)
		cost_value = max_cost_;
	else
		cost_value = max_cost_
				- log((curvature - negative_threshold_)
								/ (positive_threshold_ - negative_threshold_));
}

} //@namespace feature
} //@namespace terrain
