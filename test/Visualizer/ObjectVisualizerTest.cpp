#include <gtest/gtest.h>

#include <memory>
#include <bachelor/Visualizer/ObjectVisualizer.hpp>

#define TestVideoPath "/home/rtrk/Videos/testVideos/LimitTest3.mp4"

struct ObjectVisualizerTest : testing::Test
{
    std::unique_ptr<ObjectVisualizer> m_Test;
    cv::VideoCapture cap;
    ObjectVisualizerTest()
    {
        m_Test = std::make_unique<ObjectVisualizer>(LimitVizType);
        cap = cv::VideoCapture(TestVideoPath);
    };
    virtual ~ObjectVisualizerTest()
    {
        cap.release();
    };
};

TEST_F(ObjectVisualizerTest, draw_rects_with_nullptr_on_image)
{
    std::vector<std::vector<cv::Point>> points(4, {cv::Point(1,1), cv::Point(2,2)});
    std::vector<std::string> text(1, "50");
    Frame _frame;
    _frame.Dots = &points;
    _frame.MatFrame = nullptr;
    _frame.Text = &text;

    ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::KilledBySignal(SIGSEGV),".*");
    //expect segmentation fault
}

TEST_F(ObjectVisualizerTest, draw_rects_with_nullptr_pointsVector)
{
    cv::Mat image;
    cap >> image;
    std::vector<std::string> text(1, "50");
    Frame _frame;
    _frame.Dots = nullptr;
    _frame.MatFrame = &image;
    _frame.Text = &text;

    ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::KilledBySignal(SIGSEGV),".*");
    //expect segmentation fault
}

TEST_F(ObjectVisualizerTest, draw_rects_with_nullptr_textVector)
{
    cv::Mat image;
    cap >> image;
    std::vector<std::vector<cv::Point>> points(4, {cv::Point(1,1), cv::Point(2,2)});
    Frame _frame;
    _frame.Dots = &points;
    _frame.MatFrame = &image;
    _frame.Text = nullptr;

    ASSERT_EXIT( (m_Test->draw(_frame),std::exit(0)), ::testing::KilledBySignal(SIGSEGV),".*");
    //expect segmentation fault
}

TEST_F(ObjectVisualizerTest, draw_rects_with_empty_mat)
{
    cv::Mat image; //empty Mat
    Frame _frame;
    std::vector<std::vector<cv::Point>> points(4, {(cv::Point(1,1), cv::Point(2,2))});
    std::vector<std::string> text(1, "50");

    _frame.Dots = &points;
    _frame.MatFrame = &image;
    _frame.Text = &text;

    EXPECT_FALSE(m_Test->draw(_frame));
}

TEST_F(ObjectVisualizerTest, draw_rects_with_empty_textVector)
{
    cv::Mat image; 
    cap >> image;
    Frame _frame;
    std::vector<std::vector<cv::Point>> points(4, {(cv::Point(1,1), cv::Point(2,2))});
    std::vector<std::string> text;

    _frame.Dots = &points;
    _frame.MatFrame = &image;
    _frame.Text = &text;

    EXPECT_TRUE(m_Test->draw(_frame));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    testing::FLAGS_gtest_death_test_style="threadsafe";
    return RUN_ALL_TESTS();
}