#include "ros/ros.h"
#include "ros/console.h"
#include <rtt_rosparam/rosparam.h>

#include "math.h"
#include <urdf/model.h>
//#include <kdl_parser/kdl_parser.hpp>
#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chain_utils.hpp>


class Irb140{

public:
	Irb140();

private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};


Irb140::Irb140()
{

}

int main(int argc, char** argv){
  ros::init(argc, argv, "irb140_kdl");
	if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  const std::string urdf_file = argv[1];
	std::string robot_description;
	ros::param::get("/robot_description", robot_description);

  urdf::Model my_model;
	KDL::Tree kdl_tree;
	KDL::Chain 	kdl_chain;
  /*if (!my_model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  KDL::Tree my_tree;
	if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
  }
	*/

	rtt_ros_kdl_tools::initChainFromString(robot_description, "base_link", "Link6", kdl_tree, kdl_chain);
	rtt_ros_kdl_tools::printChain(kdl_chain);
  rtt_ros_kdl_tools::ChainUtils chainutils;
	chainutils.init();
  chainutils.printChain();
	chainutils.updateModel();
  KDL::Jacobian J = chainutils.getJacobian();

	std::cout<< J.data <<std::endl;

	Irb140 my_irb140;
	//ros::spin();
}
