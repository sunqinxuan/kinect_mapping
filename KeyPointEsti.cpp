#include "KeyPointEsti.h"

extern float xyzdata[480][640][3];
extern float xyzdata2[480][640][3];
extern Mat img1,img2;

KeyPointEsti::KeyPointEsti()
{
}

KeyPointEsti::~KeyPointEsti()
{
}

void KeyPointEsti::Extract()
{
	clock_t start,end;	

	//feature detection
	//start=clock();
	FastFeatureDetector fast(15);//越大，点数越少！15：1500；30：450
	fast.detect(img1,keyPoints1);
	fast.detect(img2,keyPoints2);
	//end=clock();
	//cout<<"feature detecting time:"<<double(end-start)<<endl;

	//drawKeypoints(img1, keyPoints1, img1, Scalar::all(255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
	//drawKeypoints(img2, keyPoints2, img2, Scalar::all(255), DrawMatchesFlags::DRAW_OVER_OUTIMG);	
	//imshow("FAST feature1", img1);
	//imshow("FAST feature2", img2);		
	//cout<<"points number:"<<keyPoints1.size()<<","<<keyPoints2.size()<<endl;

	//descriptors extraction
	//start=clock();
	BriefDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(img1, keyPoints1, descriptors1);
	extractor.compute(img2, keyPoints2, descriptors2);
	//end=clock();
	//printf("descriptors computing time:%f\n",(double)(end-start));

	//matching
	//start=clock();
	BruteForceMatcher<Hamming> matcher;
    matcher.match(descriptors1, descriptors2, matches);
	//cout<<"matched points:"<<matches.size()<<endl;
}

int KeyPointEsti::RansacEsti()
{
	points1.clear();
	points2.clear();
	int m1,n1,m2,n2;
	for(vector<DMatch>::iterator it=matches.begin();it!=matches.end();++it)
	{
		m1=keyPoints1.at((*it).queryIdx).pt.x;
		n1=keyPoints1.at((*it).queryIdx).pt.y;
		m2=keyPoints2.at((*it).trainIdx).pt.x;
		n2=keyPoints2.at((*it).trainIdx).pt.y;
		if(xyzdata[n1][m1][0]+xyzdata[n1][m1][1]+xyzdata[n1][m1][2]==0||xyzdata2[n2][m2][0]+xyzdata2[n2][m2][1]+xyzdata2[n2][m2][2]==0)
			continue;
		else
		{
			Point.x=-xyzdata[n1][m1][0];
			Point.y=xyzdata[n1][m1][1];
			Point.z=xyzdata[n1][m1][2];
			points1.push_back(Point);
			Point.x=-xyzdata2[n2][m2][0];
			Point.y=xyzdata2[n2][m2][1];
			Point.z=xyzdata2[n2][m2][2];
			points2.push_back(Point);//points1和points2是匹配好的特征点集
		}
	}
	//cout<<"3d points:"<<points1.size()<<","<<points2.size()<<endl;

	//FILE* rt;
	//rt=fopen("rt.txt","wt");

	Mat p11=Mat(3,1,CV_32FC1);
	Mat p12=Mat(3,1,CV_32FC1);
	Mat p13=Mat(3,1,CV_32FC1);
	Mat p21=Mat(3,1,CV_32FC1);
	Mat p22=Mat(3,1,CV_32FC1);
	Mat p23=Mat(3,1,CV_32FC1);
	Mat x1=Mat(3,1,CV_32FC1);
	Mat y1=Mat(3,1,CV_32FC1);
	Mat z1=Mat(3,1,CV_32FC1);
	Mat x2=Mat(3,1,CV_32FC1);
	Mat y2=Mat(3,1,CV_32FC1);
	Mat z2=Mat(3,1,CV_32FC1);
	Mat M1=Mat(3,3,CV_32FC1);
	Mat M2=Mat(3,3,CV_32FC1);
	Mat R=Mat(3,3,CV_32FC1);
	Mat T=Mat(3,1,CV_32FC1);
	Mat p1_bar=Mat(3,1,CV_32FC1);
	Mat p2_bar=Mat(3,1,CV_32FC1);
	Mat test1=Mat(3,1,CV_32FC1);
	Mat test2=Mat(3,1,CV_32FC1);
	Mat test3=Mat(3,1,CV_32FC1);
	int k1,k2,k3;
	vector<PointRansac> points1_in,points2_in,points1_final,points2_final;
	srand((int)time(0));
		double dist=0,pre_dist=9999999999;
		int count=0;
		int count_pre=0;


	Rf=Mat(3,3,CV_32FC1);
	Tf=Mat(3,1,CV_32FC1);
	//cout<<"ransac start"<<endl;
	for(int j=0;j<400;j++)
	//while(count<300)
	{
		while(1)
		{
			k1=rand()%points1.size();
			k2=rand()%points1.size();
			k3=rand()%points1.size();
			if(k1!=k2&&k1!=k3&&k2!=k3)
				break;
		}
		//cout<<"k1:"<<k1<<endl;
		//cout<<"k2:"<<k2<<endl;
		//cout<<"k3:"<<k3<<endl;
		//fprintf(rt,"k1:%d\nk2:%d\nk3:%d\n",k1,k2,k3);
		p11.at<float>(0,0)=points1.at(k1).x;
		p11.at<float>(1,0)=points1.at(k1).y;
		p11.at<float>(2,0)=points1.at(k1).z;
		p12.at<float>(0,0)=points1.at(k2).x;
		p12.at<float>(1,0)=points1.at(k2).y;
		p12.at<float>(2,0)=points1.at(k2).z;
		p13.at<float>(0,0)=points1.at(k3).x;
		p13.at<float>(1,0)=points1.at(k3).y;
		p13.at<float>(2,0)=points1.at(k3).z;
		p21.at<float>(0,0)=points2.at(k1).x;
		p21.at<float>(1,0)=points2.at(k1).y;
		p21.at<float>(2,0)=points2.at(k1).z;
		p22.at<float>(0,0)=points2.at(k2).x;
		p22.at<float>(1,0)=points2.at(k2).y;
		p22.at<float>(2,0)=points2.at(k2).z;
		p23.at<float>(0,0)=points2.at(k3).x;
		p23.at<float>(1,0)=points2.at(k3).y;
		p23.at<float>(2,0)=points2.at(k3).z;
	
		x1=p12-p11;
		x1=x1/sqrt(x1.at<float>(0,0)*x1.at<float>(0,0)+x1.at<float>(1,0)*x1.at<float>(1,0)+x1.at<float>(2,0)*x1.at<float>(2,0));
		x2=p22-p21;
		x2=x2/sqrt(x2.at<float>(0,0)*x2.at<float>(0,0)+x2.at<float>(1,0)*x2.at<float>(1,0)+x2.at<float>(2,0)*x2.at<float>(2,0));
		y1=(p13-p11)-x1.dot(p13-p11)*x1;
		y1=y1/sqrt(y1.at<float>(0,0)*y1.at<float>(0,0)+y1.at<float>(1,0)*y1.at<float>(1,0)+y1.at<float>(2,0)*y1.at<float>(2,0));
		y2=(p23-p21)-x2.dot(p23-p21)*x2;
		y2=y2/sqrt(y2.at<float>(0,0)*y2.at<float>(0,0)+y2.at<float>(1,0)*y2.at<float>(1,0)+y2.at<float>(2,0)*y2.at<float>(2,0));
		z1=x1.cross(y1);
		z2=x2.cross(y2);

		M1.at<float>(0,0)=x1.at<float>(0,0);
		M1.at<float>(1,0)=x1.at<float>(1,0);
		M1.at<float>(2,0)=x1.at<float>(2,0);
		M1.at<float>(0,1)=y1.at<float>(0,0);
		M1.at<float>(1,1)=y1.at<float>(1,0);
		M1.at<float>(2,1)=y1.at<float>(2,0);
		M1.at<float>(0,2)=z1.at<float>(0,0);
		M1.at<float>(1,2)=z1.at<float>(1,0);
		M1.at<float>(2,2)=z1.at<float>(2,0);
		M2.at<float>(0,0)=x2.at<float>(0,0);
		M2.at<float>(1,0)=x2.at<float>(1,0);
		M2.at<float>(2,0)=x2.at<float>(2,0);
		M2.at<float>(0,1)=y2.at<float>(0,0);
		M2.at<float>(1,1)=y2.at<float>(1,0);
		M2.at<float>(2,1)=y2.at<float>(2,0);
		M2.at<float>(0,2)=z2.at<float>(0,0);
		M2.at<float>(1,2)=z2.at<float>(1,0);
		M2.at<float>(2,2)=z2.at<float>(2,0);
		R=M2*M1.t();

		p1_bar=(p11+p12+p13)/3;
		p2_bar=(p21+p22+p23)/3;
		T=p2_bar-R*p1_bar;

		count=0;
		dist=0;
		for(int i=0;i<points1.size();i++)
		{
			test1.at<float>(0,0)=points1.at(i).x;
			test1.at<float>(1,0)=points1.at(i).y;
			test1.at<float>(2,0)=points1.at(i).z;
			test1=R*test1+T;
			test2.at<float>(0,0)=points2.at(i).x;
			test2.at<float>(1,0)=points2.at(i).y;
			test2.at<float>(2,0)=points2.at(i).z;
			dist=+norm(test1,test2);
			if(dist<50)
			{
				count++;
				points1_in.push_back(points1.at(i));
				points2_in.push_back(points2.at(i));
			}
			//cout<<i<<":  "<<dist<<endl;
			//fprintf(rt,"%d: %f\n",i+1,dist);
		}
		
		if(count>count_pre)
		{
			for(vector<PointRansac>::iterator it=points1_in.begin();it!=points1_in.end();++it)
			{
				points1_final.push_back(*it);
			}
			for(vector<PointRansac>::iterator it=points2_in.begin();it!=points2_in.end();++it)
			{
				points2_final.push_back(*it);
			}
			Rf=R.clone();
			Tf=T.clone();
			//cout<<"count_pre: "<<count_pre<<endl;
			//cout<<"count: "<<count<<endl<<endl;
			count_pre=count;
		}		
	}

	//if(count<300&&num!=53)
	//{
	//	Rf=Rf_pre.clone();
	//	Tf=Tf_pre.clone();
	//}
	return count_pre;
}

