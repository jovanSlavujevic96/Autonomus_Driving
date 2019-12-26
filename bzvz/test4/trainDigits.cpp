#include <opencv2/opencv.hpp>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#define ProjectPath "/home/rtrk/myROSworkspace/src/bachelor/bzvz/test4/"

void trainDigitClassifier(void);
std::string getDigits(cv::Mat image);

int main(int argc, char **argv)
{
    ///trainDigitClassifier();
    ///*
    cv::Mat input;
    {
        std::stringstream ss;
        ss << ProjectPath << argv[1];
        input = cv::imread(ss.str() );
        bool info = input.empty();
        if(info)
            return -1;
    }
    std::cout << getDigits(input) << std::endl;
    //*/
    return 0;
}

/*
 *  train the classifier for digit recognition, this could be done only one time, this method save the result in a file and
 *  it can be used in the next executions
 *  in order to train user must enter manually the corrisponding digit that the program shows, press space if the red box is just a point (false positive)
 */
void trainDigitClassifier(void)
{
    cv::Mat thr,gray,con;
    cv::Mat src;
    {
        std::stringstream ss;
        ss << ProjectPath << "all_numbers.png";
        src = cv::imread(ss.str(), 1);
        bool info = src.empty();
        if(info )
        {
            std::cout << "\"all_numbers.png\"" << " -> is there any image? -> " <<  std::boolalpha << !info << std::endl;
            std::exit(-1);
        }
    }
    cvtColor(src, gray, CV_BGR2GRAY);
    threshold(gray,thr, 125, 255, cv::THRESH_BINARY_INV); //Threshold to find contour
    //cv::imshow("ci",thr);
    //cv::waitKey(0);
    thr.copyTo(con);

    // Create sample and label data
    std::vector< std::vector <cv::Point> > contours; // Vector for storing contour
    std::vector< cv::Vec4i > hierarchy;
    cv::Mat sample;
    cv::Mat response_array;
    findContours( con, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); //Find contour

    for( int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through first hierarchy level contours
    {
        cv::Rect r= boundingRect(contours[i]); //Find bounding rect for each contour
        rectangle(src, cv::Point(r.x,r.y), cv::Point(r.x+r.width,r.y+r.height), cv::Scalar(0,0,255),2,8,0);
        cv::Mat ROI = thr(r); //Crop the image
        cv::Mat tmp1, tmp2;
        resize(ROI, tmp1, cv::Size(10,10), 0,0, cv::INTER_LINEAR ); //resize to 10X10
        tmp1.convertTo(tmp2, CV_32FC1); //convert to float

        cv::imshow("src",src);
        int c = cv::waitKey(); // Read corresponding label for contour from keyoard
        
        c-=0x30;     // Convert ascii to intiger value
        std::cout << "key pressed: " << c << std::endl;
        response_array.push_back(c); // Store label to a mat
        
        rectangle(src, cv::Point(r.x,r.y), cv::Point(r.x+r.width,r.y+r.height), cv::Scalar(0,255,0),2,8,0);
        sample.push_back(tmp2.reshape(1,1)); // Store  sample data
    }

    // Store the data to file
    cv::Mat response, tmp;
    tmp = response_array.reshape(1,1); //make continuous
    tmp.convertTo(response,CV_32FC1); // Convert  to float

    cv::FileStorage Data;
    {
        std::stringstream ss;
        ss << ProjectPath << "TrainingData.yml"; 
        Data = cv::FileStorage(ss.str(), cv::FileStorage::WRITE); // Store the sample data in a file
    }
    Data << "data" << sample;
    Data.release();

    cv::FileStorage Label; 
    {
        std::stringstream ss;
        ss << ProjectPath << "LabelData.yml"; 
        Label = cv::FileStorage(ss.str(), cv::FileStorage::WRITE); // Store the label data in a file
    }
    Label << "label" << response;
    Label.release();

    std::cout<<"Training and Label data created successfully....!! " << std::endl;

    cv::imshow("src",src);
    cv::waitKey();
}

/*
 *  get digit from the image given in param, using the classifier trained before
 */
std::string getDigits(cv::Mat image)
{
    cv::Mat thr1,gray1,con1;
    cv::Mat src1 = image.clone();
    cvtColor(src1,gray1,CV_BGR2GRAY);
    threshold(gray1,thr1,125,255,cv::THRESH_BINARY_INV); // Threshold to create input
    thr1.copyTo(con1);

    // Read stored sample and label for training
    cv::Mat sample1;
    cv::Mat response1,tmp1;
    cv::FileStorage Data1;
    {
        std::stringstream ss;
        ss << ProjectPath << "TrainingData.yml";
        Data1 = cv::FileStorage(ss.str(), cv::FileStorage::READ); // Read traing data to a Mat
    }
    Data1["data"] >> sample1;
    Data1.release();

    cv::FileStorage Label1;
    {
        std::stringstream ss;
        ss << ProjectPath << "LabelData.yml";
        Label1 = cv::FileStorage(ss.str(), cv::FileStorage::READ); // Read label data to a Mat
    }
    Label1["label"] >> response1;
    Label1.release();

    cv::Ptr<cv::ml::KNearest>  knn(cv::ml::KNearest::create());

    knn->train(sample1, cv::ml::ROW_SAMPLE,response1); // Train with sample and responses
    std::cout << "Training completed.....!!" << std::endl;

    std::vector< std::vector <cv::Point> > contours1; // Vector for storing contour
    std::vector< cv::Vec4i > hierarchy1;

    //Create input sample by contour finding and cropping
    findContours( con1, contours1, hierarchy1, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    cv::Mat dst1(src1.rows, src1.cols, CV_8UC3, cv::Scalar::all(0));
    std::string result;

    for( int i = 0; i< contours1.size(); i=hierarchy1[i][0] ) // iterate through each contour for first hierarchy level .
    {
        cv::Rect r= boundingRect(contours1[i]);
        cv::Mat ROI = thr1(r);
        cv::Mat tmp1, tmp2;
        resize(ROI,tmp1, cv::Size(10,10), 0, 0, cv::INTER_LINEAR );
        tmp1.convertTo(tmp2,CV_32FC1);
        cv::Mat bestLabels;
        float p=knn -> findNearest(tmp2.reshape(1,1),4, bestLabels);
        char name[4];
        sprintf(name,"%d",(int)p);
        std::cout << "num = " << (int)p << std::endl;
        result = result + std::to_string((int)p);

        putText( dst1, name, cv::Point(r.x,r.y+r.height), 0, 1, cv::Scalar(0, 255, 0), 2, 8 );
    }
    {
        std::stringstream ss;
        ss << ProjectPath << "dest.jpg";
        cv::imwrite(ss.str(), dst1);
    }
    return  result ;
}