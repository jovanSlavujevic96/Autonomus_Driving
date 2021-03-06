#ifndef BACHELOR_DETECTOR_DETECTORNODE_HPP_
#define BACHELOR_DETECTOR_DETECTORNODE_HPP_

#include "Detector.hpp"
#include <bachelor/DataProtocol/Receiver.hpp>
#include <bachelor/ImageProcessor/IImageProcessor.hpp>

class DetectorNode
{
    std::unique_ptr<Detector> m_Detector;
    std::unique_ptr<IReceiver> m_FrameRcv;

    void init(void);
public:
    DetectorNode(std::unique_ptr<IImageProcessor> processor);
    DetectorNode(std::unique_ptr<Detector> detector);

    void runProgram(void);

    ~DetectorNode() = default;
};

#endif //BACHELOR_DETECTOR_DETECTORNODE_HPP_