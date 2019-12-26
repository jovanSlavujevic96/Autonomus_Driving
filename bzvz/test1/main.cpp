#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <vector>

class ExpMovingAverage 
{
	double alpha; // [0;1] less = more stable, more = less stable
    double oldValue;
	bool unset;
public:
    ExpMovingAverage() 
	{
        this->alpha = 0.2;
		this->unset = true;
    }

	void clear() 
	{
		this->unset = true;
	}

    void add(double value) 
	{
        if (unset) 
		{
            this->oldValue = value;
			this->unset = false;
        }
        double newValue = this->oldValue + this->alpha * (value - this->oldValue);
        this->oldValue = newValue;
    }

	double get() 
	{
		return oldValue;
	}
};

cv::Point sub(cv::Point b, cv::Point a) 
{ 
	return cv::Point2d(b.x-a.x, b.y-a.y); 
}

cv::Point mul(cv::Point b, cv::Point a) 
{ 
	return cv::Point(b.x*a.x, b.y*a.y); 
}

cv::Point add(cv::Point b, cv::Point a) 
{ 
	return cv::Point(b.x + a.x, b.y + a.y);
}

cv::Point mul(cv::Point b, float t) 
{ 
	return cv::Point(b.x*t, b.y*t); 
}

float dot(cv::Point a, cv::Point b) 
{ 
	return (b.x*a.x + b.y*a.y); 
}

float dist(cv::Point v) 
{ 
	return sqrtf(v.x*v.x + v.y*v.y); 
}

cv::Point point_on_segment(cv::Point line0, cv::Point line1, cv::Point pt)
{
	CvPoint2D32f v = sub(pt, line0);
	CvPoint2D32f dir = sub(line1, line0);
	float len = dist(dir);
	float inv = 1.0f/(len+1e-6f);
	dir.x *= inv;
	dir.y *= inv;

	float t = dot(dir, v);
	if(t >= len) 
		return line1;
	else if(t <= 0) 
		return line0;

	return add(line0, mul(dir,t));
}

float dist2line(cv::Point line0, cv::Point line1, cv::Point pt)
{
	return dist(sub(point_on_segment(line0, line1, pt), pt));
}

void crop(cv::Mat src,  cv::Mat &dest, cv::Rect rect) 
{
	dest = src(rect).clone();
}

struct Lane 
{
	Lane(){}
	Lane(cv::Point a, cv::Point b, float angle, float kl, float bl): p0(a),p1(b),angle(angle),
		votes(0),visited(false),found(false),k(kl),b(bl) { }

	cv::Point p0, p1;
	int votes;
	bool visited, found;
	float angle, k, b;
};

struct Status 
{
	Status():reset(true),lost(0){}
	ExpMovingAverage k, b;
	bool reset;
	int lost;
};

Status laneR, laneL;

enum{
    SCAN_STEP = 5,			  // in pixels
	LINE_REJECT_DEGREES = 10, // in degrees
    BW_TRESHOLD = 250,		  // edge response strength to recognize for 'WHITE'
    BORDERX = 10,			  // px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,	  // px
	
	CANNY_MIN_TRESHOLD = 1,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 100, // edge detector maximum hysteresis threshold

	HOUGH_TRESHOLD = 50,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 50,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 100,   // join lines to one with smaller than this gaps

	CAR_DETECT_LINES = 4,    // minimum lines for a region to pass validation as a 'CAR'
	CAR_H_LINE_LENGTH = 10,  // minimum horizontal line length from car body in px

	MAX_VEHICLE_SAMPLES = 30,      // max vehicle detection sampling history
	CAR_DETECT_POSITIVE_SAMPLES = MAX_VEHICLE_SAMPLES-2, // probability positive matches for valid car
	MAX_VEHICLE_NO_UPDATE_FREQ = 15 // remove car after this much no update frames
};

#define K_VARY_FACTOR 0.2f
#define B_VARY_FACTOR 20
#define MAX_LOST_FRAMES 30

void FindResponses(cv::Mat img, int startX, int endX, int y, std::vector<int>& list)
{
	const int row = y * img.size().width * img.channels();
	unsigned char* ptr = (unsigned char*)img.data;

    int step = (endX < startX) ? -1: 1;
    int range = (endX > startX) ? endX-startX+1 : startX-endX+1;

    for(int x = startX; range>0; x += step, range--)
    {
        if(ptr[row + x] <= BW_TRESHOLD) 
			continue;

        int idx = x + step;

        while(range > 0 && ptr[row+idx] > BW_TRESHOLD)
		{
            idx += step;
            range--;
        }

        if(ptr[row+idx] <= BW_TRESHOLD) 
            list.push_back(x);

        x = idx; 
    }
}

unsigned char pixel(cv::Mat img, int x, int y) 
{
	return (unsigned char)img.data[(y*img.size().width+x)*img.channels()];
}

int findSymmetryAxisX(cv::Mat half_frame, cv::Point bmin, cv::Point bmax) {
  
  float value = 0;
  int axisX = -1;
  
  int xmin = bmin.x;
  int ymin = bmin.y;
  int xmax = bmax.x;
  int ymax = bmax.y;
  int half_width = half_frame.size().width/2;
  int maxi = 1;

  for(int x=xmin, j=0; x<xmax; x++, j++) 
  {
	float HS = 0;
    for(int y=ymin; y<ymax; y++) 
	{
		int row = y*half_frame.size().width * half_frame.channels();
        for(int step=1; step<half_width; step++) 
		{
          int neg = x-step;
          int pos = x+step;
		  unsigned char Gneg = (neg < xmin) ? 0 : (unsigned char)half_frame.data[row+neg*half_frame.channels() ];
          unsigned char Gpos = (pos >= xmax) ? 0 : (unsigned char)half_frame.data[row+pos*half_frame.channels() ];
          HS += abs(Gneg-Gpos);
        }
    }

	if (axisX == -1 || value > HS) 
	{ 		
		axisX = x;
		value = HS;
	}
  }

  return axisX;
}

bool hasVertResponse(cv::Mat &edges, int x, int y, int ymin, int ymax) {
	bool has = (pixel(edges, x, y) > BW_TRESHOLD);
	if (y-1 >= ymin) has &= (pixel(edges, x, y-1) < BW_TRESHOLD);
	if (y+1 < ymax) has &= (pixel(edges, x, y+1) < BW_TRESHOLD);
	return has;
}

int horizLine(cv::Mat &edges, int x, int y, cv::Point bmin, cv::Point bmax, int maxHorzGap) 
{
	int right = 0;
	int gap = maxHorzGap;
	for (int xx=x; xx<bmax.x; xx++) 
	{
		if (hasVertResponse(edges, xx, y, bmin.y, bmax.y)) 
		{
			right++;
			gap = maxHorzGap; // reset
		} 
		else 
		{
			gap--;
			if (gap <= 0) 
				break;
		}
	}

	int left = 0;
	gap = maxHorzGap;
	for (int xx=x-1; xx>=bmin.x; xx--) 
	{
		if (hasVertResponse(edges, xx, y, bmin.y, bmax.y)) 
		{
			left++;
			gap = maxHorzGap; // reset
		} 
		else 
		{
			gap--;
			if (gap <= 0) 
				break;
		}
	}
	return left+right;
}

void processSide(std::vector<Lane> &lanes, cv::Mat &edges, bool right) {

	Status* side = right ? &laneR : &laneL;

	// response search
	int w = edges.size().width;
	int h = edges.size().height;
	const int BEGINY = 0;
	const int ENDY = h-1;
	const int ENDX = right ? (w-BORDERX) : BORDERX;
	int midx = w/2;
	int midy = edges.size().height/2;
	unsigned char* ptr = (unsigned char*)edges.data;

	// show responses
	int* votes = new int[lanes.size()];
	for(int i=0; i<lanes.size(); i++) votes[i++] = 0;

	for(int y=ENDY; y>=BEGINY; y-=SCAN_STEP) {
		std::vector<int> rsp;
		FindResponses(edges, midx, ENDX, y, rsp);

		if (rsp.size() > 0) {
			int response_x = rsp[0]; // use first reponse (closest to screen center)

			float dmin = 9999999;
			float xmin = 9999999;
			int match = -1;
			for (int j=0; j<lanes.size(); j++) {
				// compute response point distance to current line
				float d = dist2line(
						cvPoint2D32f(lanes[j].p0.x, lanes[j].p0.y), 
						cvPoint2D32f(lanes[j].p1.x, lanes[j].p1.y), 
						cvPoint2D32f(response_x, y));

				// point on line at current y line
				int xline = (y - lanes[j].b) / lanes[j].k;
				int dist_mid = abs(midx - xline); // distance to midpoint

				// pick the best closest match to line & to screen center
				if (match == -1 || (d <= dmin && dist_mid < xmin)) {
					dmin = d;
					match = j;
					xmin = dist_mid;
					break;
				}
			}

			// vote for each line
			if (match != -1) {
				votes[match] += 1;
			}
		}
	}

	int bestMatch = -1;
	int mini = 9999999;
	for (int i=0; i<lanes.size(); i++) {
		int xline = (midy - lanes[i].b) / lanes[i].k;
		int dist = abs(midx - xline); // distance to midpoint

		if (bestMatch == -1 || (votes[i] > votes[bestMatch] && dist < mini)) {
			bestMatch = i;
			mini = dist;
		}
	}

	if (bestMatch != -1) {
		Lane* best = &lanes[bestMatch];
		float k_diff = fabs(best->k - side->k.get());
		float b_diff = fabs(best->b - side->b.get());

		bool update_ok = (k_diff <= K_VARY_FACTOR && b_diff <= B_VARY_FACTOR) || side->reset;

		printf("side: %s, k vary: %.4f, b vary: %.4f, lost: %s\n", 
			(right?"RIGHT":"LEFT"), k_diff, b_diff, (update_ok?"no":"yes"));
		
		if (update_ok) 
		{
			// update is in valid bounds
			side->k.add(best->k);
			side->b.add(best->b);
			side->reset = false;
			side->lost = 0;
		} 
		else 
		{
			// can't update, lanes flicker periodically, start counter for partial reset!
			side->lost++;
			if (side->lost >= MAX_LOST_FRAMES && !side->reset) 
				side->reset = true;
		}
	} 
	else 
	{
		printf("no lanes detected - lane tracking lost! counter increased\n");
		side->lost++;
		if (side->lost >= MAX_LOST_FRAMES && !side->reset) 
		{
			// do full reset when lost for more than N frames
			side->reset = true;
			side->k.clear();
			side->b.clear();
		}
	}
	delete[] votes;
}

void processLanes(std::vector<cv::Vec4i> &lines, cv::Mat &edges, cv::Mat &temp_frame) 
{
	std::vector<Lane> left, right;
/*
	for(int i = 0; i < lines->total; i++ )
    {
        cv::Point* line = (cv::Point*)lines[i];//cvGetSeqElem(lines,i);
		int dx = line[1].x - line[0].x;
		int dy = line[1].y - line[0].y;
		float angle = atan2f(dy, dx) * 180/CV_PI;

		if (fabs(angle) <= LINE_REJECT_DEGREES)  // reject near horizontal lines
			continue;

		// assume that vanishing point is close to the image horizontal center
		// calculate line parameters: y = kx + b;
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0!  
		float k = dy/(float)dx;
		float b = line[0].y - k*line[0].x;

		// assign lane's side based by its midpoint position 
		int midx = (line[0].x + line[1].x) / 2;
		if (midx < temp_frame.size().width/2) 
			left.push_back(Lane(line[0], line[1], angle, k, b));
		else if (midx > temp_frame.size().width/2) 
			right.push_back(Lane(line[0], line[1], angle, k, b));
    }
*/
	// show Hough lines
	for	(int i=0; i<right.size(); i++) 
	{
		cv::line(temp_frame, right[i].p0, right[i].p1, CV_RGB(0, 0, 255), 2);
	}

	for	(int i=0; i<left.size(); i++) 
	{
		cv::line(temp_frame, left[i].p0, left[i].p1, CV_RGB(255, 0, 0), 2);
	}

	processSide(left, edges, false);
	processSide(right, edges, true);

	// show computed lanes
	int x = temp_frame.size().width * 0.55f;
	int x2 = temp_frame.size().width;
	cv::line(temp_frame, cvPoint(x, laneR.k.get()*x + laneR.b.get()), 
		cvPoint(x2, laneR.k.get() * x2 + laneR.b.get()), CV_RGB(255, 0, 255), 2);

	x = temp_frame.size().width * 0;
	x2 = temp_frame.size().width * 0.45f;
	cv::line(temp_frame, cvPoint(x, laneL.k.get()*x + laneL.b.get()), 
		cvPoint(x2, laneL.k.get() * x2 + laneL.b.get()), CV_RGB(255, 0, 255), 2);
}
/*
int main(void)
{
	cv::VideoCapture inputVideo;
	{
		std::stringstream ss;
		ss << "/home/rtrk/Videos/" << "LaneDetTest.mp4";
		inputVideo = cv::VideoCapture(ss.str() );
		if(!inputVideo.isOpened() )
		{
			std::cerr << "Error: Can't open video\n";
			return -1;
		}
	}

    cv::Mat input,gray;
    inputVideo >> input;

	#define play 30
	#define pause 0
	int state = play;
	int key_pressed = 0;
	
	while(key_pressed != 27 && !input.empty() )
	{
		cv::cvtColor(input,gray,CV_BGR2GRAY);

		// threshold because of loaded jpeg artifacts
		cv::Mat edges = gray > 100;

		cv::dilate(edges, edges, cv::Mat());
		cv::dilate(edges, edges, cv::Mat());
		cv::dilate(edges, edges, cv::Mat());

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(edges, lines, 1, 2*CV_PI/180, 100,1000, 10 );

		for( size_t i = 0; i < lines.size(); i++ )
		{
			cv::Vec4i l = lines[i];

			cv::line( input, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3);

		}

		cv::resize(input, input, cv::Size(), 0.1, 0.1);
		cv::imshow("input",input);
		key_pressed = cv::waitKey(state);
		if(key_pressed == 32)
		{
			if(state)	state = pause;
			else		state = play;
		}
		inputVideo >> input;
	}
    return 0;
}
*/

int main(void)
{
	cv::VideoCapture inputVideo;
	{
		std::stringstream ss;
		ss << "/home/rtrk/Videos/" << "LaneDetTest.mp4";
		inputVideo = cv::VideoCapture(ss.str() );
		if(!inputVideo.isOpened() )
		{
			std::cerr << "Error: Can't open video\n";
			return -1;
		}
	}

	cv::Mat temp_frame, edges, gray, half_frame;
	CvMemStorage* houghStorage = cvCreateMemStorage(0);

	#define play 30
	#define pause 0
	int state = play;
	int key_pressed = 0;

	cv::Mat frame;
	inputVideo >> frame;
	while(key_pressed != 27 && !frame.empty() ) 
	{
		cv::pyrDown(frame, half_frame, cv::Size(frame.cols/2, frame.rows/2) );

		crop(frame, temp_frame, cv::Rect(0,0, frame.size().width, frame.size().height));
		
		cv::cvtColor(temp_frame, gray, CV_BGR2GRAY);
    	cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
		cv::Canny(gray, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);

		std::vector<cv::Vec4i> lines;
		{
			double rho = 1, theta = CV_PI/180;
			cv::HoughLinesP(edges, lines, rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP );
			//cv::HoughLinesP(dst, lines, 1, 2*CV_PI/180, 100,1000, 50 );
		}

		processLanes(lines, edges, temp_frame);
		
		cv::imshow("Half-frame", half_frame);

		cv::line(temp_frame, cv::Point(frame.size().width/2,0), 
			cv::Point(frame.size().width/2,frame.size().height), CV_RGB(255, 255, 0), 1);

		cv::imshow("Grey", gray);
		cv::imshow("Edges", edges);
		cv::imshow("Color", temp_frame);

		key_pressed = cv::waitKey(state);
		if(key_pressed == 32)
		{
			if(state) state = 0;
			else state = 30;
		}
		
		inputVideo >> frame;
	}
	cvReleaseMemStorage(&houghStorage);
	return 0;
}
