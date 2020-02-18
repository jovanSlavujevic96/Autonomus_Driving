#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

#define image_SemiCircle "/home/rtrk/Pictures/test/"

cv::Mat SaveThenLoadImage(const cv::Mat &image)
{
    cv::imwrite("/home/rtrk/Pictures/test/tmp.png", image);
    return (cv::imread("/home/rtrk/Pictures/test/tmp.png"));
}

cv::Mat approximateCircle(cv::Mat redHueBinaryMask, int dilation_elem = 0)
{
    cv::cvtColor(redHueBinaryMask, redHueBinaryMask, cv::COLOR_BGR2GRAY);
    int dilation_type = 0;
    switch(dilation_elem)
    {
        case 0:
            dilation_type = cv::MORPH_RECT; 
        break;

        case 1:
            dilation_type = cv::MORPH_CROSS; 
        break;

        case 2:
            dilation_type = cv::MORPH_ELLIPSE;
        break;
    }

    int size = 1;
    cv::Mat element = getStructuringElement(dilation_type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
    cv::morphologyEx(redHueBinaryMask, redHueBinaryMask, cv::MORPH_OPEN, element);
    std::vector<cv::Point2f> points;
    for (int x = 0; x < redHueBinaryMask.cols; x++)
    {
        for (int y = 0; y < redHueBinaryMask.rows; y++)
        {
            if (redHueBinaryMask.at<uchar>(y, x) > 0)
            {
                points.push_back(cv::Point2f(x, y));
            }
        }
    }
    float xn = 0, xsum = 0;
    float yn = 0, ysum = 0;
    float n = points.size();

    for (int i = 0; i < n; i++)
    {
        xsum = xsum + points[i].x;
        ysum = ysum + points[i].y;
    }
    xn = xsum / n;
    yn = ysum / n;

    float ui = 0;
    float vi = 0;
    float suu = 0, suuu = 0;
    float svv = 0, svvv = 0;
    float suv = 0;
    float suvv = 0, svuu = 0;

    for (int i = 0; i < n; i++)
    {
        ui = points[i].x - xn;
        vi = points[i].y - yn;

        suu = suu + (ui * ui);
        suuu = suuu + (ui * ui * ui);

        svv = svv + (vi * vi);
        svvv = svvv + (vi * vi * vi);

        suv = suv + (ui * vi);

        suvv = suvv + (ui * vi * vi);
        svuu = svuu + (vi * ui * ui);
    }

    cv::Mat A = (cv::Mat_<float>(2, 2) << suu, suv, suv, svv);
    cv::Mat B = (cv::Mat_<float>(2, 1) << 0.5*(suuu + suvv),0.5*(svvv + svuu));
    cv::Mat abc;
    cv::solve(A, B, abc);

    float u = abc.at<float>(0);
    float v = abc.at<float>(1);

    float x = u + xn;
    float y = v + yn;

    float alpha = u * u + v * v + ((suu + svv) / n);
    float r = sqrt(alpha);

    cv::cvtColor(redHueBinaryMask, redHueBinaryMask, cv::COLOR_GRAY2BGR);
    if(x>0 && y>0 && r>0)
    {
        cv::circle(redHueBinaryMask, cv::Point(x, y), r, cv::Scalar(255, 0, 0), 1, 8, 0);
    }

    return redHueBinaryMask;
}

std::vector<cv::Rect> getContours(const cv::Mat &image)
{
    std::vector<cv::Rect> rects;

    cv::Mat blured;
	cv::GaussianBlur(image, blured, cv::Size(9,9), 2,2);
    std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(blured, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
    {
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);
    }
    for(int i = 0; i < contours_poly.size(); ++i)
	{
    	if (cv::contourArea(contours_poly[i]) >= 250 && contours_poly[i].size() <= 50 )
        {
            rects.push_back(cv::boundingRect(contours_poly[i]) );
        }
    }
    return rects;
}

int main(int argc, char **argv)
{
    std::string imageName = image_SemiCircle + std::string("semicircle.png");
    std::string imageName2;
    if (argc >= 2)
    {
        int num = std::atoi(argv[1]);

        imageName = image_SemiCircle + std::string("blackWhiteCrops/crop") + std::to_string(num) + std::string(".jpg");
        imageName2 = image_SemiCircle + std::string("blackWhiteCrops/original") + std::to_string(num) + std::string(".jpg");
    }
    cv::Mat binaryMaskCrop, originalCrop;
    binaryMaskCrop = cv::imread(imageName);
    originalCrop = cv::imread(imageName2);
    if (binaryMaskCrop.empty() || originalCrop.empty() )
    {
        std::cout << "Could not open image..." << std::endl;
        return -1;
    }
    cv::imshow("original black white", binaryMaskCrop);
    cv::imshow("original", originalCrop);
    
    auto approxCircle = approximateCircle(binaryMaskCrop, 2);
    cv::imshow("approx circle", approxCircle);
    
    std::vector<cv::Rect> contours;
    cv::Mat ROI;
    {
        cv::Mat1b mask;
        cv::inRange(approxCircle, cv::Scalar(255,0,0), cv::Scalar(255,0,0), mask );
        //cv::imshow("mask", mask);
        contours = getContours(mask);
        ROI = mask.clone();
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5f, 1));
    cv::floodFill(ROI, cv::Point(0,0), cv::Scalar(255));
    cv::erode(ROI, ROI, element);

    auto invertedROI = ROI.clone();
    invertedROI = ~invertedROI;
    invertedROI = SaveThenLoadImage(invertedROI);
    //cv::imshow("final ROI", invertedROI);

    ///cv::Mat res;
    ///img2.copyTo(res, invertedROI);
    ///cv::imshow("result", res);

    cv::Mat foreground =  originalCrop.clone();
    cv::Mat background = cv::Mat(foreground.size().height, foreground.size().width, CV_8UC3, cv::Scalar(255,255,255) ); //white background
    cv::Mat alpha = invertedROI.clone();
     
    foreground.convertTo(foreground, CV_32FC3);
    background.convertTo(background, CV_32FC3);
    alpha.convertTo(alpha, CV_32FC3, 1.0f/255); // 
    cv::Mat ouImage = cv::Mat::zeros(foreground.size(), foreground.type());
    cv::multiply(alpha, foreground, foreground); 
    cv::multiply(cv::Scalar::all(1.0)-alpha, background, background); 
    cv::add(foreground, background, ouImage);    

    //ouImage = ouImage/255;
    cv::Mat FINAL = SaveThenLoadImage(ouImage);

    if(!contours.empty() )
    {
        FINAL = FINAL(contours[0]);
    }
    cv::imshow("final1", FINAL);

    ///cv::Mat gray;
    ///cv::cvtColor(FINAL,gray,CV_BGR2GRAY);
    ///cv::Mat mask;
    ///double grayThres = cv::threshold(gray, mask, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    ///cv::imshow("MASK", mask);

    const float factor = 0.2f;
    FINAL = FINAL(cv::Rect(cv::Point( std::round(FINAL.cols*factor), std::round(FINAL.rows*factor) ), cv::Point( std::round(FINAL.cols*(1-factor)), std::round(FINAL.rows*(1-factor)) ) ) );
    cv::imshow("final2", FINAL);
    
    cv::Ptr<cv::text::OCRTesseract> ocr = cv::text::OCRTesseract::create(NULL, "eng", "012345789", 1, 6);
    std::string word;
    ocr->run(FINAL, word, NULL, NULL, NULL, cv::text::OCR_LEVEL_WORD );

    std::cout << "word: " << word << std::endl;
    cv::waitKey();
    return 0;
}