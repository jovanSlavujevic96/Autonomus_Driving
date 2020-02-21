#ifndef BACHELOR_DETECTORNODE_HPP_
#define BACHELOR_DETECTORNODE_HPP_

#include <bachelor/Detector.hpp>
#include <bachelor/DataReceiver/DataReceiver.hpp>
#include <bachelor/ImageProcessor/IImageProcessor.hpp>

class DetectorNode
{
    std::unique_ptr<Detector> m_Detector;
    std::unique_ptr<IDataReceiver<sensor_msgs::Image>> m_FrameRcv;

    void init(void);
public:
    DetectorNode(std::unique_ptr<IImageProcessor> processor);
    DetectorNode(std::unique_ptr<Detector> detector);

    void runProgram(void);

    ~DetectorNode() = default;
};

#endif //BACHELOR_DETECTORNODE_HPP_