#include <bachelor/Detector/DetectorNode.hpp>
#include <bachelor/ImageProcessor/LaneProcessor.hpp>

int main(int argc, char **argv)
{
	const std::string nodeName = "LaneDetector_Node";
	ros::init(argc, argv, nodeName);

	std::unique_ptr<DetectorNode> detector;
	if(argc < 2 || (argc >=2 && !strcmp(argv[1], "solution1") ) )
	{
		detector = std::make_unique<DetectorNode>(std::make_unique<LaneProcessor>(CamCalSolution1));	
	}
	else if(argc >= 2 && !strcmp(argv[1], "solution2") )
	{
		detector = std::make_unique<DetectorNode>(std::make_unique<LaneProcessor>(CamCalSolution2));	
	}
	else
	{
		return EXIT_FAILURE;
	}
	std::cout << nodeName << " successfully initialized." << std::endl;

	detector->runProgram();

	return EXIT_SUCCESS;
}
