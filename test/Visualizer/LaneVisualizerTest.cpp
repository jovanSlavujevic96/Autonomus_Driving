#include <gtest/gtest.h>
#include <memory>
#include <bachelor/Visualizer/LaneVisualizer.hpp>

#define TestVideoPath "/home/rtrk/Videos/testVideos/LimitTest3.mp4"

struct LaneVisualizerTest : testing::Test
{
    std::unique_ptr<LaneVisualizer> m_Test;
    cv::VideoCapture cap; 
    LaneVisualizerTest()
    {
        m_Test = std::make_unique<LaneVisualizer>(); 
        cap = cv::VideoCapture(TestVideoPath); //testVideo
    };
    virtual ~LaneVisualizerTest()
    {
        cap.release();
    };
};

TEST_F(LaneVisualizerTest, draw_lines_with_nullptr_on_image)
{
    Frame _frame;
    std::vector<std::vector<cv::Point>> points(4, {cv::Point(1,1), cv::Point(2,2)});
    _frame.Dots = &points;
    _frame.MatFrame = nullptr;

    ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::KilledBySignal(SIGSEGV),".*");
    //expect segmentation fault
}

TEST_F(LaneVisualizerTest, draw_lines_with_nullptr_on_pointsVector)
{
    cv::Mat image;
    cap >> image;
    Frame _frame;
    _frame.Dots = nullptr;
    _frame.MatFrame = &image;

    ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::KilledBySignal(SIGSEGV),".*");
    //expect segmentation fault
}

TEST_F(LaneVisualizerTest, draw_lines_with_nullptr_on_textVector)
//there is no textVector in use
{
    cv::Mat image;
    cap >> image;
    std::vector<std::vector<cv::Point>> points(4, {(cv::Point(1,1), cv::Point(2,2))});
    Frame _frame;
    _frame.Dots = &points;
    _frame.MatFrame = &image;
    _frame.Text = nullptr;

    //ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::ExitedWithCode(0),".*");
    //expect normal exit
    EXPECT_TRUE(m_Test->draw(_frame));
}

TEST_F(LaneVisualizerTest, draw_lines_with_empty_mat)
{
    cv::Mat image; //empty Mat
    Frame _frame;
    std::vector<std::vector<cv::Point>> points(4, {(cv::Point(1,1), cv::Point(2,2))});

    _frame.Dots = &points;
    _frame.MatFrame = &image;

    //ASSERT_EXIT((m_Test->draw(_frame),std::exit(0)),::testing::ExitedWithCode(0),".*"); 
    //expect normal exit
    EXPECT_FALSE(m_Test->draw(_frame));
}

TEST_F(LaneVisualizerTest, draw_lines_with_empty_pointsVector)
{
    cv::Mat image; 
    cap >> image;
    std::vector<std::vector<cv::Point>> points; //empty vector

    Frame _frame;
    _frame.Dots = &points;
    _frame.MatFrame = &image;
    
    //ASSERT_EXIT((m_Test->draw(_frame),std::exit(0)),::testing::ExitedWithCode(0),".*"); 
    //expect normal exit
    EXPECT_FALSE(m_Test->draw(_frame));
}

TEST_F(LaneVisualizerTest, draw_lines_with_onePointInPointsVector)
{
    cv::Mat image; 
    cap >> image;
    std::vector<std::vector<cv::Point>> points(3, {(cv::Point(0,0), cv::Point(1,1))});
    {
        std::vector<cv::Point> pt(1);
        pt[0] = (cv::Point(2,2));
        points.push_back(pt);
    }
    Frame _frame;
    _frame.Dots = &points;
    _frame.MatFrame = &image;
    
    EXPECT_TRUE(m_Test->draw(_frame));
}

TEST_F(LaneVisualizerTest, draw_lines_with_lessThen4lines)
{
    cv::Mat image; 
    cap >> image;
    std::vector<std::vector<cv::Point>> points(2, {(cv::Point(1,1), cv::Point(2,2))});

    Frame _frame;
    _frame.Dots = &points;
    _frame.MatFrame = &image;
    
    //ASSERT_EXIT((m_Test->draw(_frame),std::exit(0)),::testing::ExitedWithCode(0),".*"); 
    //expect normal exit
    EXPECT_FALSE(m_Test->draw(_frame));

    points.push_back({cv::Point(0,0), cv::Point(image.cols, image.rows)});  //increase size by 1 (size 3)

    //ASSERT_EXIT((m_Test->draw(_frame),std::exit(0)),::testing::ExitedWithCode(0),".*"); 
    //expect normal exit
    EXPECT_FALSE(m_Test->draw(_frame));
}

TEST_F(LaneVisualizerTest, draw_lines_with_points_bigger_than_image)
{
    cv::Mat image;
    cap >> image;
    std::vector<std::vector<cv::Point>> points(4);
    points[0] = {cv::Point(image.cols, image.rows), cv::Point(0,0)};
    points[1] = {cv::Point(0, image.rows), cv::Point(image.cols/2, image.rows/2)};
    points[2] = {cv::Point(image.cols/2, 0), cv::Point(image.cols/2, image.rows)};
    points[3] = {cv::Point(image.cols/2, image.rows/2), cv::Point(image.cols,image.rows/2)};
    Frame _frame;
    _frame.MatFrame = &image;
    _frame.Dots = &points;
    cv::resize(image, image, cv::Size(image.cols/2, image.rows/2) );
    
    ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::ExitedWithCode(0),".*");
    //expect normal exit
    EXPECT_TRUE(m_Test->draw(_frame));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 