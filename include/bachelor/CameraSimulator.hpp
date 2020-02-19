#ifndef BACHELOR_CAMERASIMULATOR_HPP_
#define BACHELOR_CAMERASIMULATOR_HPP_

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>

#include <opencv2/opencv.hpp>
#include <memory>

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

class CameraSimulator : public IObserver<std_msgs::Bool>
{
private:
	std::unique_ptr<IDataSender<sensor_msgs::Image>> m_FrameEmiter;
	std::unique_ptr<IDataSender<std_msgs::Bool>> m_WatchdogEmiter;
	bool m_NodeMSG;
	cv::VideoCapture m_Video;
	bool m_IgnoreDetection = false, m_PauseVideo = false;

	void checkMsgs(void);
public:
	CameraSimulator();
	virtual ~CameraSimulator() = default;

	void setVideo(cv::VideoCapture& video);

	void update(const std_msgs::Bool& msg, Topics subjTopic) override;
	bool doStuff(void) override;
};

#endif //BACHELOR_CAMERASIMULATOR_HPP_