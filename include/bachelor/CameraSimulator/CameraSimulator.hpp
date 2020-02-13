#ifndef BACHELOR_CAMERASIMULATOR_CAMERASIMULATOR_HPP
#define BACHELOR_CAMERASIMULATOR_CAMERASIMULATOR_HPP

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>

#include <opencv2/opencv.hpp>
#include <memory>

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

template class IObserver<std_msgs::Bool>;
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
	virtual ~CameraSimulator();

	void setVideo(cv::VideoCapture &_video);

	void update(std_msgs::Bool &_data, Topics _subjTopic) override;
	bool doStuff(void) override;
};

#endif //BACHELOR_CAMERASIMULATOR_CAMERASIMULATOR_HPP