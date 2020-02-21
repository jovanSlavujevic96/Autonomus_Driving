#include <bachelor/DetectorNode.hpp>
#include <bachelor/ImageProcessor/LimitProcessor.hpp>

int main(int argc, char **argv)
{
	const std::string nodeName = "LimitDetector_Node";
	ros::init(argc, argv, nodeName);

	DetectorNode detector(std::make_unique<LimitProcessor>() );	
    std::cout << nodeName << " successfully initialized." << std::endl;

	detector.runProgram();

	return EXIT_SUCCESS;
}