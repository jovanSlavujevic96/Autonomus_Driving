#ifndef BACHELOR_VIDEOPLAYER_NODE_VIDEOPLAYER_HPP_
#define BACHELOR_VIDEOPLAYER_NODE_VIDEOPLAYER_HPP_

#include <opencv2/opencv.hpp>

#include <bachelor/Observer/IBoolObserver.hpp>
#include <bachelor/DataProtocol/IBoolDataEmiter.hpp>
#include <bachelor/DataProtocol/IFrameDataEmiter.hpp>
#include <bachelor/Topics.h>

#include <memory>

class VideoPlayer : public IBoolObserver
{
private:
	std::unique_ptr<IFrameDataEmiter> m_FrameEmiter;
	std::unique_ptr<IBoolDataEmiter> m_TimerStartsEmiter, m_WatchdogEmiter;
	bool m_NodeMSG[3];
	cv::VideoCapture m_Video;
	bool m_IgnoreDetection = false, m_PauseVideo = false;
public:
	VideoPlayer();
	virtual ~VideoPlayer();

	void setVideo(cv::VideoCapture &_video);
	bool Cycle(void);
	void checkMsgs(void);
	virtual void update(bool _data, Topics _subjTopic) override;
};

#endif //BACHELOR_VIDEOPLAYER_NODE_VIDEOPLAYER_HPP_