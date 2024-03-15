#include "ICP.h"

extern float xyzdata[480][640][3];
extern float xyzdata2[480][640][3];
extern uchar texture[480][640][3];
extern uchar texture2[480][640][3];
extern Mat img1,img2;

ICP::ICP()
{
	trans.R.q0=1;
	trans.R.q1=0;
	trans.R.q2=0;
	trans.R.q3=0;
	trans.T.t0=0;
	trans.T.t1=0;
	trans.T.t2=0;
	transqr.R.q0=1;
	transqr.R.q1=0;
	transqr.R.q2=0;
	transqr.R.q3=0;
	transqr.T.t0=0;
	transqr.T.t1=0;
	transqr.T.t2=0;
}

ICP::~ICP()
{
}

void ICP::ReleasePoint3DSet(Point3DSet *pset)
{
	free(pset->point);
	free(pset);
}

Point3DSet* ICP::loadDataSet(void)
{
	Point3DSet *pset =(Point3DSet*) malloc(sizeof(Point3DSet));	
	pset->point =(Point3D*) malloc(icp_width*icp_height*sizeof(Point3D));
	pset->invalid_point_number = 0;
	int num=0;
	//int i=200;
	for(int i=0;i<icp_height;i+=4)
	{
		for(int j=0;j<icp_width;j+=4)
		{	
			if(xyzdata[i][j][0]+xyzdata[i][j][1]+xyzdata[i][j][2]==0)//||xyzdata[i][j][2]>4000)
				continue;
			else
			{
				pset->point[num].x=-xyzdata[i][j][0];
				pset->point[num].y=xyzdata[i][j][1];
				pset->point[num].z=xyzdata[i][j][2];
				pset->point[num].b=texture[i][j][0];
				pset->point[num].g=texture[i][j][1];
				pset->point[num].r=texture[i][j][2];
				num++;
			}
		}
	}	
	pset->number = num;
	//FILE* fp;
	//fp=fopen("dataset.txt","wt");
	//for(int i=0;i<pset->number;i++)
	//	//fprintf(fp,"%f  %f\n",pset->point[i].x,pset->point[i].z);
	//	fprintf(fp,"%f %f %f\n",pset->point[i].r,pset->point[i].g,pset->point[i].b);
	//fprintf(fp,"%d\n",pset->number);
	//fclose(fp);
	return pset;
}

Point3DSet* ICP::loadModelSet(void)
{
	Point3DSet *pset =(Point3DSet*) malloc(sizeof(Point3DSet));	
	pset->point =(Point3D*) malloc(icp_width*icp_height*sizeof(Point3D));
	pset->invalid_point_number = 0;
	int num=0;
	//int i=200;
	for(int i=0;i<icp_height;i+=4)
	{
		for(int j=0;j<icp_width;j+=4)
		{	
			if(xyzdata2[i][j][0]+xyzdata2[i][j][1]+xyzdata2[i][j][2]==0)//||xyzdata2[i][j][2]>4000)
				continue;
			else
			{
				pset->point[num].x=-xyzdata2[i][j][0];
				pset->point[num].y=xyzdata2[i][j][1];
				pset->point[num].z=xyzdata2[i][j][2];
				pset->point[num].b=texture2[i][j][0];
				pset->point[num].g=texture2[i][j][1];
				pset->point[num].r=texture2[i][j][2];
				num++;
			}
		}
	}	
	pset->number = num;
	//FILE* fp;
	//fp=fopen("modelset.txt","wt");
	//for(int i=0;i<pset->number;i++)
	//	//fprintf(fp,"%f  %f\n",pset->point[i].x,pset->point[i].z);
	//	fprintf(fp,"%f %f %f\n",pset->point[i].r,pset->point[i].g,pset->point[i].b);
	//fprintf(fp,"%d\n",pset->number);
	//fclose(fp);
	return pset;
}

void ICP::Selection(Point3DSet *data_set,Point3DSet *pset)
{
	//Point3DSet *pset =(Point3DSet*) malloc(sizeof(Point3DSet));
	//pset->point =(Point3D*) malloc((height*width)*sizeof(Point3D));
	//pset->invalid_point_number = 0;

	for(int i=0;i<data_set->number;i++)
	{
		pset->point[i].x=data_set->point[i].x;
		pset->point[i].y=data_set->point[i].y;//0;//
		pset->point[i].z=data_set->point[i].z;
		pset->point[i].r=data_set->point[i].r;
		pset->point[i].g=data_set->point[i].g;
		pset->point[i].b=data_set->point[i].b;
	}
	pset->number=data_set->number;
	//return pset;
}

void ICP::Matching(Point3DSet *sel_data_set, Point3DSet *model_set, Point3DSet *yset) 
{
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;	
	queryPt = annAllocPt(3);					// allocate query point
	dataPts = annAllocPts(model_set->number, 3);			// allocate data points
	nnIdx = new ANNidx[1];						// allocate near neigh indices
	dists = new ANNdist[1];						// allocate near neighbor dists
	for(int i=0;i<model_set->number;i++)
	{
		dataPts[i][0]=model_set->point[i].x;
		dataPts[i][1]=model_set->point[i].y;
		dataPts[i][2]=model_set->point[i].z;
	}
	kdTree = new ANNkd_tree(dataPts,model_set->number,3);
					//dataPts,					// the data points
					//nPts,						// number of points
					//dim);						// dimension of space

	/* return closest point set with the same size as selected data_set */
	//Point3DSet *yset =(Point3DSet*) malloc(sizeof(Point3DSet));
	//yset->number = sel_data_set->number;
	//yset->point =(Point3D*) malloc((sel_data_set->number)*sizeof(Point3D));
	//yset->invalid_point_number = 0;

	/* Find closest point set Y for data set */
	int i, j;
	int closest_index = 0;
	float distance = 0;
	//float minimum_distance = 0;
	int num=0;
	index.clear();
	//FILE* fp=fopen("distance.txt","wt");
	for (i=0; i < sel_data_set->number; i++) //sel_data_set中每一点i
	{
		closest_index = 0;
		//minimum_distance = 1000000000;

		queryPt[0]=sel_data_set->point[i].x;
		queryPt[1]=sel_data_set->point[i].y;
		queryPt[2]=sel_data_set->point[i].z;
		kdTree->annkSearch(queryPt,1,nnIdx,dists,0);		

		//for (j=0; j < model_set->number; j++) //寻找点i在model_set中的最近点（认为是在同一个坐标系下）
		//{
		//	if ((model_set->point[j].x == 0)&&(model_set->point[j].y == 0)) {continue;}
		//	distance = (sel_data_set->point[i].x - model_set->point[j].x)*(sel_data_set->point[i].x - model_set->point[j].x) +
		//				(sel_data_set->point[i].y - model_set->point[j].y)*(sel_data_set->point[i].y - model_set->point[j].y) +
		//				(sel_data_set->point[i].z - model_set->point[j].z)*(sel_data_set->point[i].z - model_set->point[j].z) ;
		//	if (distance < minimum_distance) {
		//		minimum_distance = distance;
		//		closest_index = j;
		//	}
		//}
		
		if(dists[0]<10000)
		{
			yset->point[num].x = model_set->point[nnIdx[0]].x;
			yset->point[num].y = model_set->point[nnIdx[0]].y;
			yset->point[num].z = model_set->point[nnIdx[0]].z;
			yset->point[num].r = model_set->point[nnIdx[0]].r;
			yset->point[num].g = model_set->point[nnIdx[0]].g;
			yset->point[num].b = model_set->point[nnIdx[0]].b;
			//sel_data_set->point[num].x=sel_data_set->point[i].x;
			//sel_data_set->point[num].y=sel_data_set->point[i].y;
			//sel_data_set->point[num].z=sel_data_set->point[i].z;
			//sel_data_set->point[num].r=sel_data_set->point[i].r;
			//sel_data_set->point[num].g=sel_data_set->point[i].g;
			//sel_data_set->point[num].b=sel_data_set->point[i].b;
			index.push_back(i);
			//fprintf(fp,"%f\n",dists[0]);
			//fprintf(fp,"%f,%f,%f\n",sel_data_set->point[num].x,sel_data_set->point[num].y,sel_data_set->point[num].z);
			//fprintf(fp,"%f,%f,%f\n",yset->point[num].x,yset->point[num].y,yset->point[num].z);
			num++;
		}
	}

	//FILE* idx;
	//idx=fopen("index.txt","wt");
	//for(vector<int>::iterator it=index.begin();it!=index.end();++it)
	//{
	//	fprintf(idx,"%d\n",(*it));
	//}
	//fclose(idx);

	yset->number=num;
	sel_data_set->number=num;
	delete nnIdx;
	delete dists;
	delete kdTree;
	//fprintf(fp,"%d\n",num);
	//fclose(fp);
	//return yset;//由model_set中与sel_data_set每一个点对应的最近点组成
}

Point3D ICP::GetMeanOfPointSet(Point3DSet *pset) {
	int i;
	Point3D p_bar;
	p_bar.x = 0;
	p_bar.y = 0;
	p_bar.z = 0;
	for (i=0; i < pset->number; i++) 
	{
		p_bar.x += pset->point[i].x;
		p_bar.y += pset->point[i].y;
		p_bar.z += pset->point[i].z;
	}
	p_bar.x /= pset->number;
	p_bar.y /= pset->number;
	p_bar.z /= pset->number;

	return p_bar;
}

void ICP::SetRotationMatrix(CvMat *Matrix, Rotation R) //由四元数生成旋转矩阵
{
	CvScalar s;//图像像素数据，若图像是一通道的，val[0]中存储数据
				//RGB图像：B:val[0],G:val[1],R:val[2],alphe:val[3]

	s.val[0] = R.q0*R.q0 + R.q1*R.q1 - R.q2*R.q2 - R.q3*R.q3;
	cvSet2D(Matrix, 0, 0, s);//给Matrix的点（0,0）赋值s

	s.val[0] = 2*(R.q1*R.q2 - R.q0*R.q3);
	cvSet2D(Matrix, 0, 1, s);

	s.val[0] = 2*(R.q1*R.q3 + R.q0*R.q2);
	cvSet2D(Matrix, 0, 2, s);

	s.val[0] = 2*(R.q2*R.q1 + R.q0*R.q3);
	cvSet2D(Matrix, 1, 0, s);

	s.val[0] = R.q0*R.q0 - R.q1*R.q1 + R.q2*R.q2 - R.q3*R.q3;
	cvSet2D(Matrix, 1, 1, s);

	s.val[0] = 2*(R.q2*R.q3 - R.q0*R.q1);
	cvSet2D(Matrix, 1, 2, s);

	s.val[0] = 2*(R.q3*R.q1 - R.q0*R.q2);
	cvSet2D(Matrix, 2, 0, s);

	s.val[0] = 2*(R.q3*R.q2 + R.q0*R.q1);
	cvSet2D(Matrix, 2, 1, s);

	s.val[0] = R.q0*R.q0 - R.q1*R.q1 - R.q2*R.q2 + R.q3*R.q3;
	cvSet2D(Matrix, 2, 2, s);
}

Transformation ICP::GetOptimalRotation(Point3DSet *dset, Point3DSet *mset) 
{
	//#define GetOptimalRotation_DEBUG_MESSAGE
	Point3D dbar, mbar;
	//dbar = GetMeanOfPointSet(dset);//点云的平均值，重心？？
	dbar.x=0;
	dbar.y=0;
	dbar.z=0;
	for (int i=0;i<mset->number;i++)
	{
		dbar.x+=dset->point[index[i]].x;
		dbar.y+=dset->point[index[i]].y;
		dbar.z+=dset->point[index[i]].z;
	}
	dbar.x /= mset->number;
	dbar.y /= mset->number;
	dbar.z /= mset->number;

	mbar = GetMeanOfPointSet(mset);
	//cout<<"dbar:"<<dbar.x<<","<<dbar.y<<","<<dbar.z<<endl;
	//cout<<"mbar:"<<mbar.x<<","<<mbar.y<<","<<mbar.z<<endl;
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	PrintPoint3D(dbar);
	PrintPoint3D(mbar);
	#endif

	CvMat *P = cvCreateMat(4,4,CV_32FC1);
	float Array[]={0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0};
	cvSetData(P, Array, P->step);
	//PrintMatrix(P, P->rows, P->cols);

	float Sxx=0, Sxy=0, Sxz=0, Syx=0, Syy=0, Syz=0, Szx=0, Szy=0, Szz=0;
	float Mcx, Mcy, Mcz, Dcx, Dcy, Dcz;
	int i;
	//cout<<"index:"<<index.size()<<endl;
	for (i=0; i<mset->number; i++) {
		Mcx = mset->point[i].x - mbar.x;//sel_data_set和model中最近点分别减去均值
		Mcy = mset->point[i].y - mbar.y;
		Mcz = mset->point[i].z - mbar.z;
		Dcx = dset->point[index[i]].x - dbar.x;
		Dcy = dset->point[index[i]].y - dbar.y;
		Dcz = dset->point[index[i]].z - dbar.z;
		Sxx += (Dcx*Mcx);
		Sxy += (Dcx*Mcy);
		Sxz += (Dcx*Mcz);
		Syx += (Dcy*Mcx);
		Syy += (Dcy*Mcy);
		Syz += (Dcy*Mcz);
		Szx += (Dcz*Mcx);
		Szy += (Dcz*Mcy);
		Szz += (Dcz*Mcz);
	}
	//printf("Sxx = %f\n", Sxx);

	/* Set P */
	CvScalar s;

	s.val[0] = Sxx + Syy + Szz;//矩阵P赋值，P干嘛用？？
	cvSet2D(P, 0, 0, s);
	s.val[0] = Syz - Szy;
	cvSet2D(P, 0, 1, s);
	s.val[0] = Szx - Sxz;
	cvSet2D(P, 0, 2, s);
	s.val[0] = Sxy - Syx;
	cvSet2D(P, 0, 3, s);
	s.val[0] = Syz - Szy;
	cvSet2D(P, 1, 0, s);
	s.val[0] = Sxx - Syy - Szz;
	cvSet2D(P, 1, 1, s);
	s.val[0] = Sxy + Syx;
	cvSet2D(P, 1, 2, s);
	s.val[0] = Szx + Sxz;
	cvSet2D(P, 1, 3, s);
	s.val[0] = Szx - Sxz;
	cvSet2D(P, 2, 0, s);
	s.val[0] = Sxy + Syx;
	cvSet2D(P, 2, 1, s);
	s.val[0] = Syy - Sxx - Szz;
	cvSet2D(P, 2, 2, s);
	s.val[0] = Syz + Szy;
	cvSet2D(P, 2, 3, s);
	s.val[0] = Sxy - Syx;
	cvSet2D(P, 3, 0, s);
	s.val[0] = Szx + Sxz;
	cvSet2D(P, 3, 1, s);
	s.val[0] = Syz + Szy;
	cvSet2D(P, 3, 2, s);
	s.val[0] = Szz - Sxx - Syy;
	cvSet2D(P, 3, 3, s);
	//PrintMatrix(P, P->rows, P->cols);

	CvMat *EigenValue_Row=cvCreateMat(4,1,CV_32FC1);//P的特征值
	CvMat *EigenVector=cvCreateMat(4,4,CV_32FC1);//对应的特征向量
	cvEigenVV(P, EigenVector, EigenValue_Row, DBL_EPSILON);
	//PrintMatrix(P, P->rows, P->cols);
	//PrintMatrix(EigenValue_Row, EigenValue_Row->rows, EigenValue_Row->cols);
	//PrintMatrix(EigenVector, EigenVector->rows, EigenVector->cols);

	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	cvSetData(R, Array, R->step);

	float max_eigenvalue = cvGet2D(EigenValue_Row,0,0).val[0];
	int max_eigenvalue_index = 0;
	for (i=0; i<4; i++)     
	{
		if (cvGet2D(EigenValue_Row,i,0).val[0] > max_eigenvalue) 
		{
			max_eigenvalue = cvGet2D(EigenValue_Row,i,0).val[0];
			max_eigenvalue_index = i;
		}
	}
	//printf("max_eigenvalue = %f\n", max_eigenvalue);
	//printf("max_eigenvalue_index = %d\n", max_eigenvalue_index);
	Transformation RT;		//RT.R赋值，P的最大的特征值对应的特征向量
	RT.R.q0 = cvGet2D(EigenVector,max_eigenvalue_index,0).val[0];
	RT.R.q1 = cvGet2D(EigenVector,max_eigenvalue_index,1).val[0];
	RT.R.q2 = cvGet2D(EigenVector,max_eigenvalue_index,2).val[0];
	RT.R.q3 = cvGet2D(EigenVector,max_eigenvalue_index,3).val[0];
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	printf("Optimal Rotation (%f,%f,%f,%f)\n", RT.R.q0, RT.R.q1, RT.R.q2, RT.R.q3);
	#endif
	SetRotationMatrix(R, RT.R);
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	PrintMatrix(R, R->rows, R->cols);
	#endif
	CvMat *T = cvCreateMat(3,1,CV_32FC1);
	CvMat *mbarMatrix = cvCreateMat(3,1,CV_32FC1);
	s.val[0] = mbar.x;
	cvSet2D(mbarMatrix, 0, 0, s);
	s.val[0] = mbar.y;
	cvSet2D(mbarMatrix, 1, 0, s);
	s.val[0] = mbar.z;
	cvSet2D(mbarMatrix, 2, 0, s);
	CvMat *dbarMatrix = cvCreateMat(3,1,CV_32FC1);
	s.val[0] = dbar.x;
	cvSet2D(dbarMatrix, 0, 0, s);
	s.val[0] = dbar.y;
	cvSet2D(dbarMatrix, 1, 0, s);
	s.val[0] = dbar.z;
	cvSet2D(dbarMatrix, 2, 0, s);
	CvMat *Matrix1 = cvCreateMat(3,1,CV_32FC1);
	cvmMul(R, dbarMatrix, Matrix1);//Matrix1=R*dbarMatrix
	CvMat *Matrix2 = cvCreateMat(3,1,CV_32FC1);
	cvmSub(mbarMatrix, Matrix1, Matrix2);//Matrix2=R*dbarMatrix+mbarMatrix
	#ifdef GetOptimalRotation_DEBUG_MESSAGE
	PrintMatrix(Matrix2, Matrix2->rows, Matrix2->cols);
	#endif
	RT.T.t0 = cvGet2D(Matrix2, 0, 0).val[0];//T <-- Matrix2=R*dbarMatrix+mbarMatrix
	RT.T.t1 = cvGet2D(Matrix2, 1, 0).val[0];
	RT.T.t2 = cvGet2D(Matrix2, 2, 0).val[0];

	cvReleaseMat(&Matrix2);
	cvReleaseMat(&Matrix1);
	cvReleaseMat(&mbarMatrix);
	cvReleaseMat(&dbarMatrix);
	cvReleaseMat(&EigenValue_Row);
	cvReleaseMat(&EigenVector);
	cvReleaseMat(&T);
	cvReleaseMat(&P);
	cvReleaseMat(&R);

	return RT;
}

void ICP::TransformPoint3DSet(Transformation RT, Point3DSet *pset, Point3DSet *yset) 
{
	//Point3DSet *yset =(Point3DSet*) malloc(sizeof(Point3DSet));
	//yset->number = pset->number;
	//yset->point =(Point3D*) malloc((pset->number)*sizeof(Point3D));
	//yset->invalid_point_number = 0;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	CvMat *T = cvCreateMat(3,1,CV_32FC1);

	SetRotationMatrix(R, RT.R);//生成旋转矩阵R
	CvScalar s;
	s.val[0] = RT.T.t0;
	cvSet2D(T, 0, 0, s);
	s.val[0] = RT.T.t1;
	cvSet2D(T, 1, 0, s);
	s.val[0] = RT.T.t2;
	cvSet2D(T, 2, 0, s);//生成平移向量T
//	PrintMatrix(R, R->rows, R->cols);
//	PrintMatrix(T, T->rows, T->cols);

	CvMat *point = cvCreateMat(3,1,CV_32FC1);
	CvMat *Matrix1 = cvCreateMat(3,1,CV_32FC1);
	CvMat *Matrix2 = cvCreateMat(3,1,CV_32FC1);
	int i;

	for (i=0; i < pset->number; i++) 
	{
		s.val[0] = pset->point[i].x;
		cvSet2D(point, 0, 0, s);
		s.val[0] = pset->point[i].y;
		cvSet2D(point, 1, 0, s);
		s.val[0] = pset->point[i].z;
		cvSet2D(point, 2, 0, s);
		cvmMul(R, point, Matrix1);//矩阵乘法:Matrix1=R*point
		cvmAdd(Matrix1, T, Matrix2);//矩阵加法:Matrix2=R*point+T,即point经过变换后的点
		yset->point[i].x = cvGet2D(Matrix2, 0, 0).val[0];
		yset->point[i].y = cvGet2D(Matrix2, 1, 0).val[0];
		yset->point[i].z = cvGet2D(Matrix2, 2, 0).val[0];//变换后的点存入pset
		yset->point[i].r = pset->point[i].r;
		yset->point[i].g = pset->point[i].g;
		yset->point[i].b = pset->point[i].b;
	}
	yset->number=pset->number;

			//float r31,r11,r21,r32,r33;
			//r31=CV_MAT_ELEM(*R,float,2,0);
			//r11=CV_MAT_ELEM(*R,float,0,0);
			//r21=CV_MAT_ELEM(*R,float,1,0);	
			//r32=CV_MAT_ELEM(*R,float,2,1);
			//r33=CV_MAT_ELEM(*R,float,2,2);
			//beta=atan2(-r31,sqrt(r11*r11+r21*r21));
			//alpha=atan2(r21/cos(beta),r11/cos(beta));
			//gamma=atan2(r32/cos(beta),r33/cos(beta));
	

	cvReleaseMat(&Matrix2);
	cvReleaseMat(&Matrix1);
	cvReleaseMat(&point);
	cvReleaseMat(&T);
	cvReleaseMat(&R);
	//return yset;
}

Transformation ICP::updatetransform(Transformation t1,Transformation t2)
{
	//transform=t1*t2，再赋给t1
	Transformation transform;
	transform.R.q0=t1.R.q0*t2.R.q0-t1.R.q1*t2.R.q1-t1.R.q2*t2.R.q2-t1.R.q3*t2.R.q3;
	transform.R.q1=t1.R.q0*t2.R.q1+t1.R.q1*t2.R.q0+t1.R.q2*t2.R.q3-t1.R.q3*t2.R.q2;
	transform.R.q2=t1.R.q0*t2.R.q2-t1.R.q1*t2.R.q3+t1.R.q2*t2.R.q0+t1.R.q3*t2.R.q1;
	transform.R.q3=t1.R.q0*t2.R.q3+t1.R.q1*t2.R.q2-t1.R.q2*t2.R.q1+t1.R.q3*t2.R.q0;
	CvMat *R = cvCreateMat(3,3,CV_32FC1);
	SetRotationMatrix(R,t1.R);
	transform.T.t0=t2.T.t0*CV_MAT_ELEM(*R,float,0,0)+t2.T.t1*CV_MAT_ELEM(*R,float,0,1)+t2.T.t2*CV_MAT_ELEM(*R,float,0,2)+t1.T.t0;
	transform.T.t1=t2.T.t0*CV_MAT_ELEM(*R,float,1,0)+t2.T.t1*CV_MAT_ELEM(*R,float,1,1)+t2.T.t2*CV_MAT_ELEM(*R,float,1,2)+t1.T.t1;
	transform.T.t2=t2.T.t0*CV_MAT_ELEM(*R,float,2,0)+t2.T.t1*CV_MAT_ELEM(*R,float,2,1)+t2.T.t2*CV_MAT_ELEM(*R,float,2,2)+t1.T.t2;
	//transform.T.t0=t1.T.t0+t2.T.t0;
	//transform.T.t1=t1.T.t1+t2.T.t1;
	//transform.T.t2=t1.T.t2+t2.T.t2;
	return transform;
}

Transformation ICP::updatetransform_qr(Transformation t1,Transformation t2)
{
	//transform=t1*t2，再赋给t1
	Transformation transform;
	transform.R.q0=t1.R.q0*t2.R.q0-t1.R.q1*t2.R.q1-t1.R.q2*t2.R.q2-t1.R.q3*t2.R.q3;
	transform.R.q1=t1.R.q0*t2.R.q1+t1.R.q1*t2.R.q0+t1.R.q2*t2.R.q3-t1.R.q3*t2.R.q2;
	transform.R.q2=t1.R.q0*t2.R.q2-t1.R.q1*t2.R.q3+t1.R.q2*t2.R.q0+t1.R.q3*t2.R.q1;
	transform.R.q3=t1.R.q0*t2.R.q3+t1.R.q1*t2.R.q2-t1.R.q2*t2.R.q1+t1.R.q3*t2.R.q0;
	transform.T.t0=t1.T.t0+t2.T.t0;
	transform.T.t1=t1.T.t1+t2.T.t1;
	transform.T.t2=t1.T.t2+t2.T.t2;
	return transform;
}

Transformation ICP::Matrix2Quat(CvMat *R,CvMat *T)
{
	Transformation RT;
	float s;
	float tq[4];
	int i,j;
	tq[0]=1+cvGet2D(R, 0, 0).val[0]+cvGet2D(R, 1, 1).val[0]+cvGet2D(R, 2, 2).val[0];
	tq[1]=1+cvGet2D(R, 0, 0).val[0]-cvGet2D(R, 1, 1).val[0]-cvGet2D(R, 2, 2).val[0];
	tq[2]=1-cvGet2D(R, 0, 0).val[0]+cvGet2D(R, 1, 1).val[0]-cvGet2D(R, 2, 2).val[0];
	tq[3]=1-cvGet2D(R, 0, 0).val[0]-cvGet2D(R, 1, 1).val[0]+cvGet2D(R, 2, 2).val[0];
	j=0;
	for(i=1;i<4;i++)
		j=(tq[i]>tq[j])?i:j;
	if(j==0)
	{
		RT.R.q0=tq[0];
		RT.R.q1=-cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
		RT.R.q2=-cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
		RT.R.q3=-cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
	}
	else if(j==1)
	{
		RT.R.q0=-cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
		RT.R.q1=tq[1];
		RT.R.q2=cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
		RT.R.q3=cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
	}
	else if(j==2)
	{
		RT.R.q0=-cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
		RT.R.q1=cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
		RT.R.q2=tq[2];
		RT.R.q3=cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
	}
	else
	{
		RT.R.q0=-cvGet2D(R, 0, 1).val[0]+cvGet2D(R, 1, 0).val[0];
		RT.R.q1=cvGet2D(R, 2, 0).val[0]+cvGet2D(R, 0, 2).val[0];
		RT.R.q2=cvGet2D(R, 1, 2).val[0]+cvGet2D(R, 2, 1).val[0];
		RT.R.q3=tq[3];
	}
	s=sqrt(0.25/tq[j]);
	//cout<<"j: "<<j<<endl;
	RT.R.q0*=s;
	RT.R.q1*=s;
	RT.R.q2*=s;
	RT.R.q3*=s;
	RT.T.t0=cvGet2D(T, 0, 0).val[0];
	RT.T.t1=cvGet2D(T, 1, 0).val[0];
	RT.T.t2=cvGet2D(T, 2, 0).val[0];
	return RT;
}

void ICP::icp(Point3DSet *data_set, Point3DSet *model_set, Mat Rf, Mat Tf, int flag_qr) 
{
	clock_t start,end;	
	Point3DSet *yset;
	Transformation RT,RT0;
	Point3DSet *sel_data_set;
	Point3DSet *sel_model_set;
	Point3DSet *sel_data_set_k;
	Transformation transqr_tmp={{1,0,0,0},{0,0,0}};
	Mat Rf_pre=Mat(3,3,CV_32FC1);
	Mat Tf_pre=Mat(3,1,CV_32FC1);
	sel_data_set_k =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	sel_data_set_k->point =(Point3D*) malloc(19200*sizeof(Point3D));
	sel_data_set_k->invalid_point_number = 0;
	yset =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	yset->point =(Point3D*) malloc(19200*sizeof(Point3D));
	yset->invalid_point_number = 0;
	sel_data_set =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	sel_data_set->point =(Point3D*) malloc(19200*sizeof(Point3D));
	sel_data_set->invalid_point_number = 0;
	sel_model_set =(Point3DSet*) malloc(sizeof(Point3DSet));//Pk
	sel_model_set->point =(Point3D*) malloc(19200*sizeof(Point3D));
	sel_model_set->invalid_point_number = 0;

	CvMat *rot= cvCreateMat(3,3,CV_32FC1);
	CvMat *rott= cvCreateMat(3,3,CV_32FC1);
	int trancount = 0;
	float dms,dms_tmp,dms_avr=0,dms_avr_pre=10;
	int num;
	RT0.R.q0=1;
	RT0.R.q1=0;
	RT0.R.q2=0;
	RT0.R.q3=0;
	RT0.T.t0=0;
	RT0.T.t1=0;
	RT0.T.t2=0;
	CvMat R=cvMat(3,3,CV_32FC1);
	CvMat T=cvMat(3,1,CV_32FC1);
	CvMat RR=cvMat(3,3,CV_32FC1);
	CvMat TT=cvMat(3,1,CV_32FC1);

	//start=clock();
	//if(flag_qr==1)
	//{
	//	transqr_tmp=Matrix2Quat(&Rqr,&Tqr);
	//	transqr=updatetransform_qr(transqr,transqr_tmp);
	//	trans=transqr;
	//	//fp<<"corrected:"<<endl;
	//	//fp<<"trans:R:"<<trans.R.q0<<","<<trans.R.q1<<","<<trans.R.q2<<","<<trans.R.q3<<",T:"<<trans.T.t0<<","<<trans.T.t1<<","<<trans.T.t2<<endl;
	//	TransformPoint3DSet(transqr,data_set,data_set);
	//	SetRotationMatrix(rott, transqr.R);
	//	pose.orgx=transqr.T.t0;
	//	pose.orgy=transqr.T.t1;
	//	pose.orgz=transqr.T.t2;
	//	pose.x1=CV_MAT_ELEM(*rott,float,0,0);
	//	pose.x2=CV_MAT_ELEM(*rott,float,1,0);
	//	pose.x3=CV_MAT_ELEM(*rott,float,2,0);
	//	pose.y1=CV_MAT_ELEM(*rott,float,0,1);
	//	pose.y2=CV_MAT_ELEM(*rott,float,1,1);
	//	pose.y3=CV_MAT_ELEM(*rott,float,2,1);
	//	pose.z1=CV_MAT_ELEM(*rott,float,0,2);
	//	pose.z2=CV_MAT_ELEM(*rott,float,1,2);
	//	pose.z3=CV_MAT_ELEM(*rott,float,2,2);
	//	fp<<"rotaion matrix:"<<endl<<
	//		CV_MAT_ELEM(*rott,float,0,0)<<","<<CV_MAT_ELEM(*rott,float,1,0)<<","<<CV_MAT_ELEM(*rott,float,2,0)<<","<<endl<<
	//		CV_MAT_ELEM(*rott,float,0,1)<<","<<CV_MAT_ELEM(*rott,float,1,1)<<","<<CV_MAT_ELEM(*rott,float,2,1)<<","<<endl<<
	//		CV_MAT_ELEM(*rott,float,0,2)<<","<<CV_MAT_ELEM(*rott,float,1,2)<<","<<CV_MAT_ELEM(*rott,float,2,2)<<endl<<endl<<endl<<endl;
	//	traj.push_back(pose);
	//	pointcloud.push_back(*data_set);
	//}
	//else
	{
		
		//if(inliers>300)
		{
			R=Rf;
			T=Tf;
		}
		//else
		//{
		//	fp<<"*******************************************"<<endl;
		//	R=Rf_pre;
		//	T=Tf_pre;
		//}
		Rf_pre=Rf.clone();
		Tf_pre=Tf.clone();
		RT0=Matrix2Quat(&R,&T);
		TransformPoint3DSet(RT0,data_set,sel_data_set);
		trans=updatetransform(trans,RT0);
		TransformPoint3DSet(RT0,data_set,sel_data_set_k);
		while(fabs(dms_avr-dms_avr_pre)>1&&trancount<10)//(trancount<60)//
		{
			Matching(sel_data_set_k,model_set,yset);
			RT=GetOptimalRotation(sel_data_set,yset);
			TransformPoint3DSet(RT,sel_data_set,sel_data_set_k);
			dms=0;
			num=0;
			for(int i=0;i<yset->number;i++)
			{
				dms_tmp=sqrt((sel_data_set_k->point[index[i]].x-yset->point[i].x)*(sel_data_set_k->point[index[i]].x-yset->point[i].x)
							+(sel_data_set_k->point[index[i]].y-yset->point[i].y)*(sel_data_set_k->point[index[i]].y-yset->point[i].y)
							+(sel_data_set_k->point[index[i]].z-yset->point[i].z)*(sel_data_set_k->point[index[i]].z-yset->point[i].z));
				dms+=dms_tmp;
				num++;
			}	
			dms_avr_pre=dms_avr;
			dms_avr=dms/num;
			//cout<<"dms_avr:"<<dms_avr<<endl;
			trancount++;
		}

		//if(dms_avr<30)
		{
			trans=updatetransform(trans,RT);
			SetRotationMatrix(rot, trans.R);
		}


		TransformPoint3DSet(trans,data_set,data_set);
		pose.orgx=trans.T.t0;
		pose.orgy=trans.T.t1;
		pose.orgz=trans.T.t2;
		pose.x1=CV_MAT_ELEM(*rot,float,0,0);
		pose.x2=CV_MAT_ELEM(*rot,float,1,0);
		pose.x3=CV_MAT_ELEM(*rot,float,2,0);
		pose.y1=CV_MAT_ELEM(*rot,float,0,1);
		pose.y2=CV_MAT_ELEM(*rot,float,1,1);
		pose.y3=CV_MAT_ELEM(*rot,float,2,1);
		pose.z1=CV_MAT_ELEM(*rot,float,0,2);
		pose.z2=CV_MAT_ELEM(*rot,float,1,2);
		pose.z3=CV_MAT_ELEM(*rot,float,2,2);
		{
			traj.push_back(pose);
			pointcloud.push_back(*data_set);
		}
		
		
	}
	//end=clock();
	//cout<<"icp time:"<<end-start<<endl;

	ReleasePoint3DSet(sel_data_set);
	ReleasePoint3DSet(sel_model_set);
	ReleasePoint3DSet(yset);
	ReleasePoint3DSet(sel_data_set_k);
	ReleasePoint3DSet(model_set);
}



