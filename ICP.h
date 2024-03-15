#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <ostream>
#include <fstream>
#include <vector>
#include <string.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "ANN/ANN.h"
#include "cv.h"

using namespace std;
using namespace cv;

#define M_PI 3.14159265358979323846
#define uchar unsigned char
#define SIGN(x) ( (x)<0 ? -1:((x)>0?1:0 ) )
#define icp_width 640
#define icp_height 480


typedef struct Point3D {
	float x;
	float y;
	float z;
	uchar r;
	uchar g;
	uchar b;
} Point3D;

typedef struct Point3DSet {
	Point3D *point;
	int number;
	int invalid_point_number;
} Point3DSet;

typedef struct Rotation {
	float q0;
	float q1;
	float q2;
	float q3;
} Rotation;

typedef struct Translation {
	float t0;
	float t1;
	float t2;
} Translation;

typedef struct Transformation {
	Rotation R;
	Translation T;
} Transformation;

typedef struct Pose {
	float orgx;
	float orgy;
	float orgz;
	float x1;
	float x2;
	float x3;
	float y1;
	float y2;
	float y3;
	float z1;
	float z2;
	float z3;
} Pose;


class ICP
{
public:
	ICP();
	~ICP();
	void ReleasePoint3DSet(Point3DSet *pset);
	Point3DSet *loadDataSet(void);
	Point3DSet *loadModelSet(void);
	void icp(Point3DSet *data_set, Point3DSet *model_set, Mat Rf, Mat Tf, int flag_qr);
	vector<Pose> traj;
	vector<Point3DSet> pointcloud;
private:
	void Selection(Point3DSet *data_set,Point3DSet *pset);
	void Matching(Point3DSet *sel_data_set, Point3DSet *model_set, Point3DSet *yset);
	Point3D GetMeanOfPointSet(Point3DSet *pset);
	void SetRotationMatrix(CvMat *Matrix, Rotation R);
	Transformation GetOptimalRotation(Point3DSet *dset, Point3DSet *mset);
	void TransformPoint3DSet(Transformation RT, Point3DSet *pset, Point3DSet *yset);
	Transformation updatetransform(Transformation t1,Transformation t2);
	Transformation Matrix2Quat(CvMat *R,CvMat *T);
	Transformation updatetransform_qr(Transformation t1,Transformation t2);
	vector<int> index;
	Pose pose;
	Transformation trans;
	Transformation transqr;
};