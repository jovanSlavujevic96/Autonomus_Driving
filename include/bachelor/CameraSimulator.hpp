#ifndef BACHELOR_CAMERASIMULATOR_HPP_
#define BACHELOR_CAMERASIMULATOR_HPP_

#include <bachelor/IObserver.hpp>
#include <bachelor/DataProtocol/ISender.hpp>

#include <opencv2/opencv.hpp>
#include <memory>

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

class IPlatformRcv;

class CameraSimulator : 
	public IObserver
{
private:
	std::unique_ptr<ISender> m_FrameEmiter;
	std::unique_ptr<ISender> m_WatchdogEmiter;
	cv::VideoCapture m_Video;
	bool m_PauseVideo = false;

	void checkMsgs(void);
public:
	CameraSimulator();
	virtual ~CameraSimulator() = default;

	void setVideo(cv::VideoCapture& video);

	void update(const IPlatformRcv* receiver) override;
	bool doStuff(void);
};

#endif //BACHELOR_CAMERASIMULATOR_HPP_