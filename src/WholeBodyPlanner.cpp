#include <dwl_planners/WholeBodyPlanner.h>

//#include <tests/model/HS071Constraint.cpp>
//#include <tests/model/HS071Cost.cpp>


namespace dwl_planners
{

WholeBodyPlanner::WholeBodyPlanner() : planning_ptr_(NULL), solver_(NULL), environment_(NULL)
{

}


WholeBodyPlanner::~WholeBodyPlanner()
{

}


void WholeBodyPlanner::init()
{
	solver_ = new dwl::solver::IpoptNLP();

//	dwl::model::BenchmarkCost cost;
//	dwl::model::BenchmarkConstraint constraint;
//	solver_->addCost(&cost);
//	solver_->addConstraint(&constraint);

//	planning_ptr_->reset(&robot_, solver_, environment_);
	solver_->init();

	solver_->compute();
}


bool WholeBodyPlanner::compute()
{

	return true;
}

} //@namespace dwl_planners



int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamic_wholebody_planner");

	dwl_planners::WholeBodyPlanner planner;

	planner.init();
	ros::spinOnce();

//	try {
//		ros::Rate loop_rate(100);
//
//		while (ros::ok()) {
//			if (planner.compute()) {
//				planner.publishBodyPath();
//				planner.publishContactSequence();
//				planner.publishContactRegions();
//			}
//			ros::spinOnce();
//			loop_rate.sleep();
//		}
//	} catch (std::runtime_error& e) {
//		ROS_ERROR("hierarchical_planner exception: %s", e.what());
//		return -1;
//	}

	return 0;
}
