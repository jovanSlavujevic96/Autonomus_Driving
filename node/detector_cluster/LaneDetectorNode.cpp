#include <bachelor/DetectorNode.hpp>
#include <bachelor/ImageProcessor/LaneProcessor.hpp>

int main(int argc, char **argv)
{
	const std::string nodeName = "LaneDetector_Node";
	ros::init(argc, argv, nodeName);

	DetectorNode detector(std::make_unique<LaneProcessor>() );	
    std::cout << nodeName << " successfully initialized." << std::endl;

	detector.runProgram();

	return EXIT_SUCCESS;
}
