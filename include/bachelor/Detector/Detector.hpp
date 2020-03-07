#ifndef BACHELOR_DETECTOR_DETECTOR_HPP_
#define BACHELOR_DETECTOR_DETECTOR_HPP_

#include <memory>

#include <bachelor/IObserver.hpp> 
#include <bachelor/DataProtocol/ISender.hpp>
#include <bachelor/ImageProcessor/IImageProcessor.hpp>

class Detector : 
	public IObserver
{
private:
	std::unique_ptr<ISender> m_DataEmiterWatchdog;
	std::unique_ptr<ISender> m_CoordSender;
	std::unique_ptr<ISender> m_ToECU;
	std::unique_ptr<IImageProcessor> m_ImgProc;

public:
	Detector(std::unique_ptr<IImageProcessor> procType);
	virtual ~Detector() = default;
	
	void update(const IPlatformRcv* receiver) override;	//observer method
	bool doStuff(void) override;
};

#endif //BACHELOR_DETECTOR_DETECTOR_HPP_
