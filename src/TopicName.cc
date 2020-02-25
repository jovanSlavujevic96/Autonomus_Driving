#include <bachelor/Topic.h>

std::map<Topic, std::string> TopicName =
{
    {ImHere_CamSim, "/Im_here/CAM"},       {ImHere_Visual, "/Im_here/VISUAL"},  {ImHere_LaneDet, "/Im_here/LANE_DET"},
    {ImHere_StopDet, "/Im_here/STOP_DET"}, {ImHere_LimDet, "/Im_here/LIM_DET"}, {ImHere_ECU, "/Im_here/ECU"},
    {PauseOrPlay, "/pause/play"},          {RawFrame, "/raw_frame"},            {Coord_LaneDet, "/Coord/LANE_DET"},
    {Coord_StopDet, "/Coord/STOP_DET"},    {Coord_LimDet, "/Coord/LIM_DET"},    {ECU_LaneDet, "/ECU/LANE_DET"},
    {ECU_StopDet, "/ECU/STOP_DET"},        {ECU_LimDet, "/ECU/LIM_DET"},        {LogFromECU, "/LogFromECU"}    
};