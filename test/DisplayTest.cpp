#include <gmock/gmock.h>
#include <memory>
#include <bachelor/Visualizer/MockVisualizer.hpp>
#include <bachelor/Display.hpp>

struct DisplayTest : testing::Test
{
    std::unique_ptr<MockVisualizer> mockVis;
    std::unique_ptr<Display> display;
    DisplayTest()
    {
        mockVis = std::make_unique<MockVisualizer>();
        display = std::make_unique<Display>();
    };
    virtual ~DisplayTest()
    {

    };
};

TEST_F(DisplayTest, DisplayTest1)
{
    display->addVisualizer(mockVis.get(), Coord_LaneDet );
}

int main(int argc, char** argv)
{
    testing::InitGoogleMock(&argc,argv);
    return RUN_ALL_TESTS();
} 