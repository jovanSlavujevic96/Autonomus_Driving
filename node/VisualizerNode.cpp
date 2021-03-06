#include <ros/ros.h>
#include <string>
#include <map>

#include <bachelor/Display.hpp>
#include <bachelor/DataProtocol/Receiver.hpp>
#include <bachelor/DataProtocol/PlatformRcvCoord.hpp>
#include <bachelor/DataProtocol/PlatformRcvLog.hpp>
#include <bachelor/DataProtocol/PlatformRcvImage.hpp>

#include <bachelor/Visualizer/LaneVisualizer.hpp>
#include <bachelor/Visualizer/ObjectVisualizer.hpp>
#include <bachelor/Visualizer/LogVisualizer.hpp>

int main(int argc, char **argv)
{
    bool draw[3] = {false};
    std::unique_ptr<IVisualizer> visualizer[3];
    if(argc >= 2)
    {
        for(int i=1; i<argc; ++i)
        {
            if(!strcmp(argv[i], "yes"))
            {
                draw[i-1] = true;
                if((i-1)==0)
                {
                    visualizer[i-1] = std::make_unique<LaneVisualizer>();
                }
                else if((i-1)==1)
                {
                    visualizer[i-1] = std::make_unique<ObjectVisualizer>(LimitVizType);
                }
                else if((i-1)==2)
                {
                    visualizer[i-1] = std::make_unique<ObjectVisualizer>(StopVizType);
                }
            }
        }
    }
    const std::string nodeName = "Visualizer_Node";
    ros::init(argc, argv, nodeName);

    Display display;
    std::unique_ptr<IReceiver> coordinatesRcv[3];
    {
        const std::string names[3] = {"lanes", "limit rects", "stop rects"};
        const Topic topics[3] = {Coord_LaneDet, Coord_LimDet, Coord_StopDet};
        for(int i=0; i<3; ++i)
        {
            std::cout << "draw "<< names[i] << ": " << std::boolalpha << draw[i] << std::endl;
            if(draw[i])
            {
                coordinatesRcv[i] = std::make_unique<Receiver>(std::make_unique<PlatformRcvCoord>(topics[i]));
                coordinatesRcv[i]->registerObserver(&display);
                display.addVisualizer(visualizer[i].get(), topics[i]);
            }
        }
    }
    std::unique_ptr<IVisualizer> LogViz = std::make_unique<LogVisualizer>();
    display.addVisualizer(LogViz.get(), LogFromECU);

    std::unique_ptr<IReceiver> framesRcv;
    framesRcv = std::make_unique<Receiver>(std::make_unique<PlatformRcvImage>(RawFrame));
    framesRcv->registerObserver(&display);

    std::unique_ptr<IReceiver> logRcv = std::make_unique<Receiver>(std::make_unique<PlatformRcvLog>(LogFromECU));
    logRcv->registerObserver(&display);
    
    std::cout << nodeName << " successfully initialized." << std::endl; 

    while(ros::ok() && display.doStuff() )
    {
        ros::spinOnce();
    }

    return 0;
}