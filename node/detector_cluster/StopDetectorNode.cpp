#include <bachelor/DetectorNode.hpp>
#include <bachelor/ImageProcessor/StopProcessor.hpp>

int main(int argc, char **argv)
{
	const std::string nodeName = "StopDetector_Node";
	ros::init(argc, argv, nodeName);

	DetectorNode detector(std::make_unique<StopProcessor>() );	
    std::cout << nodeName << " successfully initialized." << std::endl;

	detector.runProgram();

	return EXIT_SUCCESS;
}
