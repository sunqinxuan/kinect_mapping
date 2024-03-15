#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/calib3d/calib3d.hpp"

#include <vector>
#include <iterator>
#include <string>
#include <time.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

typedef struct PointRansac {
	float x;
	float y;
	float z;
} PointRansac;

class KeyPointEsti
{
public:
	KeyPointEsti();
	~KeyPointEsti();
	void Extract();
	int RansacEsti();
	Mat Rf;	
	Mat Tf;
private:
	vector<KeyPoint> keyPoints1,keyPoints2;
	vector<DMatch> matches;	
	vector<PointRansac> points1,points2;
	PointRansac Point;
};