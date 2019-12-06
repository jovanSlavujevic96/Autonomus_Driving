#ifndef BACHELOR_VIDEOPLAYERNODE_VIDEOPLAYER_HPP_
#define BACHELOR_VIDEOPLAYERNODE_VIDEOPLAYER_HPP_

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

	void checkMsgs(void);
public:
	VideoPlayer();
	virtual ~VideoPlayer();

	void setVideo(cv::VideoCapture &_video);
	bool SendFrame(void);
	virtual void update(bool _data, Topics _subjTopic) override;
};

#endif //BACHELOR_VIDEOPLAYERNODE_VIDEOPLAYER_HPP_