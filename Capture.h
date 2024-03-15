#include <iostream>
#include <windows.h>
#include <tchar.h>
#include <strsafe.h>
#include <OpenNI.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define WIN32_LEAN_AND_MEAN
#define uchar unsigned char 

using namespace std;
using namespace openni;
using namespace cv;

class Capture
{
public:
	Capture();
	~Capture();
	int capture(void);
	int initial(void);
	int close(void);
	Mat cImageBGR;
private:
	int width, height;
	Device mDevice;
	VideoStream mDepthStream;
	VideoStream mColorStream;
};