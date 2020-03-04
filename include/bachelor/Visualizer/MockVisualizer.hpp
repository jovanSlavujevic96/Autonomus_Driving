#ifndef BACHELOR_VISUALIZER_MOCKVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_MOCKVISUALIZER_HPP_

#include "IVisualizer.hpp"
#include <gmock/gmock.h>

class MockVisualizer : public IVisualizer
{
public:
    MOCK_METHOD1(draw, bool(Frame& frame));
    MOCK_CONST_METHOD0(getVisualizerType, VisualizerType(void));
};

#endif //BACHELOR_VISUALIZER_MOCKVISUALIZER_HPP_