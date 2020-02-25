#ifndef BACHELOR_VISUALIZER_OBJECTVISUALIZER_HPP_
#define BACHELOR_VISUALIZER_OBJECTVISUALIZER_HPP_

#include "IVisualizer.hpp"

class ObjectVisualizer : 
    public IVisualizer
{
    cv::Scalar m_RectColor, m_TextColor;
    std::string m_SignName;
    const VisualizerType m_VisualizerType;

public:
    ObjectVisualizer(VisualizerType type);
    virtual ~ObjectVisualizer() = default;

    void draw(Frame* frame) override;
    VisualizerType getVisualizerType(void) override;
};

#endif //BACHELOR_VISUALIZER_OBJECTVISUALIZER_HPP_