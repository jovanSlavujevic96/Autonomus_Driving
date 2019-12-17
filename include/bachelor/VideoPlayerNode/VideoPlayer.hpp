#ifndef BACHELOR_VIDEOPLAYERNODE_VIDEOPLAYER_HPP_
#define BACHELOR_VIDEOPLAYERNODE_VIDEOPLAYER_HPP_

#include <bachelor/Observer/IObserver.hpp>
#include <bachelor/DataProtocol/IDataSender.hpp>

#include <opencv2/opencv.hpp>
#include <memory>

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

template class IObserver<bool>;
class VideoPlayer : public IObserver<bool>
{
private:
	std::unique_ptr<IDataSender<sensor_msgs::Image,sensor_msgs::Image> > m_FrameEmiter;
	std::unique_ptr<IDataSender<bool,std_msgs::Bool> > m_TimerStartsEmiter, m_WatchdogEmiter;
	bool m_NodeMSG[3];
	cv::VideoCapture m_Video;
	bool m_IgnoreDetection = false, m_PauseVideo = false;

	void checkMsgs(void);
public:
	VideoPlayer();
	virtual ~VideoPlayer();

	void setVideo(cv::VideoCapture &_video);

	virtual void update(bool &_data, Topics _subjTopic) override;
	virtual bool doStuff(void) override;
};

#endif //BACHELOR_VIDEOPLAYERNODE_VIDEOPLAYER_HPP_