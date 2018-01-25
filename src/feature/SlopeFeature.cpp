#include <terrain_server/feature/SlopeFeature.h>
#include <Eigen/Dense>


namespace terrain_server
{

namespace feature
{


SlopeFeature::SlopeFeature() : flat_threshold_(1.0 * (M_PI / 180.0)),
		steep_threshold_(70.0 * (M_PI / 180.0))
{
	name_ = "Slope";
}


SlopeFeature::~SlopeFeature()
{

}


void SlopeFeature::computeCost(double& cost_value,
							   const dwl::Terrain& terrain_info)
{
	double slope = fabs(acos((double) terrain_info.surface_normal(2)));

	if (slope < flat_threshold_)
		cost_value = 0.;
	else if (slope < steep_threshold_) {
		cost_value = -log(1 - (slope - flat_threshold_) / (steep_threshold_ - flat_threshold_));
		if (max_cost_ < cost_value)
			cost_value = max_cost_;
	} else
		cost_value = max_cost_;
}

} //@namespace feature
} //@namespace terrain_server
