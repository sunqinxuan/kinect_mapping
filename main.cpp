#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "Capture.h"
#include "KeyPointEsti.h"
#include "ICP.h"
#include "GL/freeglut.h"

#define Height 480
#define Width 640

ICP icp;
float xyzdata[480][640][3];
float xyzdata2[480][640][3];
uchar texture[480][640][3];
uchar texture2[480][640][3];
Mat img1,img2;

int glWinWidth = 640, glWinHeight = 480;
double eyex, eyey, eyez, atx, aty, atz;
double sx=1,sy=1,sz=1;
bool leftClickHold = false, rightClickHold = false;
bool wheelup = false, wheeldown = false;
int mx,my;
int ry = 90, rx = 90;
double mindepth=200.0, maxdepth=2000.0; 
double radius = 3000.0;
extern bool flag_endloop=0;
bool flag_send=false;

void mouse(int button, int state, int x, int y)
{
   if(button == GLUT_LEFT_BUTTON)
   {
      if(state == GLUT_DOWN)
      {
         leftClickHold=true;
      }
      else
      {
         leftClickHold=false;
      }
   }

   if (button== GLUT_RIGHT_BUTTON)
   {
      if(state == GLUT_DOWN)
      {
         rightClickHold=true;
      }
      else
      {
         rightClickHold=false;
      }
   }
}

void key(int key, int x, int y)
{
	switch(key)
	{
	case GLUT_KEY_UP:
		sx+=0.01;
		sy+=0.01;
		sz+=0.01;
		glutPostRedisplay();
		break;
	case GLUT_KEY_DOWN:
		sx-=0.01;
		sy-=0.01;
		sz-=0.01;
		glutPostRedisplay();
		break;
	case GLUT_KEY_END:
		flag_endloop=true;
		break;
	default:
		sx=sx;
		sy=sy;
		sz=sz;
	}
}

void motion(int x, int y)
{
   int rstep = 5; 
   if(leftClickHold==true)
   {
      if( abs(x-mx) > abs(y-my) )
      {
         rx += SIGN(x-mx)*rstep;    
      }
      else
      {
         ry -= SIGN(y-my)*rstep;    
      }
      
      mx=x;
      my=y;
      glutPostRedisplay();
   }

   if(rightClickHold==true)
   {
      radius += SIGN(y-my)*100.0;
      radius = std::max( radius, 100.0 );
      mx=x;
      my=y;
      glutPostRedisplay();
   }
}

void reshape (int w, int h) 
{
   glWinWidth = w;
   glWinHeight = h;
   glViewport (0, 0, (GLsizei)w, (GLsizei)h);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective (45, (GLfloat)w / (GLfloat)h, 1.0, 15000.0);   
   glMatrixMode (GL_MODELVIEW);
}

void renderScene(void) 
{
   // clear screen and depth buffer
   glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
   // Reset the coordinate system before modifying 
   glLoadIdentity();   
   // set the camera position
   atx = 0.0f;
   aty = 0.0f;
   atz = ( mindepth - maxdepth ) / 2.0f;
   eyex = atx + radius * sin( CV_PI * ry / 180.0f ) * cos( CV_PI * rx/ 180.0f ); 
   eyey = aty + radius * cos( CV_PI * ry/ 180.0f ); 
   eyez = atz + radius * sin( CV_PI * ry / 180.0f ) * sin( CV_PI * rx/ 180.0f );
   gluLookAt ( eyex, eyey, eyez, atx, aty, atz,0.0, -1.0, 0.0);
   //gluLookAt(0.0f,0.0f,6000.0f,0.0f,0.0f,1000.0f,0.0,1.0,0.0);
   glRotatef(0,0,1,0);
   glRotatef(-180,1,0,0);
   glScalef(sx,sy,sz);

   float x,y,z;
   // 绘制图像点云
   glPointSize(2.0); 
   glBegin(GL_POINTS);

   glColor3f(1,0,0);
   glVertex3f(0,0,0);
   //z:b
   glColor3f(0,0,1);
   glVertex3f(0,0,100);
   glColor3f(0,0,1);
   glVertex3f(0,0,500);
   glColor3f(0,0,1);
   glVertex3f(0,0,1000);
   //x:r
   glColor3f(1,0,0);
   glVertex3f(100,0,0);
   glColor3f(1,0,0);
   glVertex3f(500,0,0);
   glColor3f(1,0,0);
   glVertex3f(1000,0,0);
   //y:g
   glColor3f(0,1,0);
   glVertex3f(0,100,0);
   glColor3f(0,1,0);
   glVertex3f(0,500,0);
   glColor3f(0,1,0);
   glVertex3f(0,1000,0);
   //for(vector<Point3D>::iterator it=display.begin();it!=display.end();++it)
   //{
	  // glColor3f((*it).b/255,(*it).g/255,(*it).r/255);//(1,1,1);//
	  // x= (*it).x;
	  // y= (*it).y;
	  // z= (*it).z;
	  // glVertex3f(-x,y,z); 
   //}
   for(vector<Point3DSet>::iterator it=icp.pointcloud.begin();it!=icp.pointcloud.end();++it)
   {
	   for(int i=0;i<(*it).number;i++)
	   {
		   glColor3f((float)(*it).point[i].b/255,(float)(*it).point[i].g/255,(float)(*it).point[i].r/255);//(1,1,1);//
		   x= (*it).point[i].x;
		   y= (*it).point[i].y;
		   z= (*it).point[i].z;
		   glVertex3f(x,y,z); 
	   }
   }
   glEnd(); 

   /////////////////////////////////////////////////////new added//////////////////////////////////
   glBegin(GL_LINES);
   for(vector<Pose>::iterator it=icp.traj.begin();it!=icp.traj.end();++it)
   {
	   glColor3f(1,0,0);
	   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
	   glVertex3f(((*it).orgx+(*it).x1*200),(*it).orgy+(*it).x2*200,(*it).orgz+(*it).x3*200);
	   glColor3f(0,1,0);
	   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
	   glVertex3f(((*it).orgx+(*it).y1*200),(*it).orgy+(*it).y2*200,(*it).orgz+(*it).y3*200);
	   glColor3f(0,0,1);
	   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
	   glVertex3f(((*it).orgx+(*it).z1*200),(*it).orgy+(*it).z2*200,(*it).orgz+(*it).z3*200);
	   if(it!=icp.traj.begin())
	   {
		   glColor3f(1,1,1);
		   glVertex3f((*it).orgx,(*it).orgy,(*it).orgz);
		   glVertex3f((*(it-1)).orgx,(*(it-1)).orgy,(*(it-1)).orgz);
	   }
   }
   glEnd();
   /////////////////////////////////////////////////////////////////////////////////////////////////

   glFlush();
   glutSwapBuffers();
}

DWORD WINAPI sendpoint3d(LPVOID lpParam)
{
	unsigned char* xyz=(unsigned char*) lpParam;

	int iResult;
    WSADATA wsaData;
    SOCKET SendSocket = INVALID_SOCKET;
    SOCKADDR_IN RecvAddr;
    unsigned short Port = 8000;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR) 
	{
        wprintf(L"WSAStartup failed with error: %d\n", iResult);
        return 1;
    }
    // Create a socket for sending data
    SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (SendSocket == INVALID_SOCKET) 
	{
        wprintf(L"socket failed with error: %ld\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }
    RecvAddr.sin_family = AF_INET;
    RecvAddr.sin_port = htons(Port);
    RecvAddr.sin_addr.s_addr = inet_addr("192.168.0.111");
	cout<<"socket initialized"<<endl;

	int buflen=1000;
    char SendBuf[1000];
	char RecvBuf[10];
	int RecvAddrSize=sizeof(RecvAddr);
	char *buf=SendBuf;
	//char *ptr;
	int len=sizeof(float);
	int numt=1;
	//char *ptr;
	while(true)
	{
		Sleep(1);
		if(flag_send)
		{
			cout<<"flag in the thread:"<<flag_send<<endl;
		}
		
		if(flag_send)
		{
			//cout<<numt<<":ready to send"<<endl;
			//sprintf(SendBuf,"No.%d:send sth,for test",numt);
			//buflen=sizeof(SendBuf);
			//iResult = sendto(SendSocket,SendBuf, buflen, 0, (SOCKADDR *) &RecvAddr, sizeof (RecvAddr));
			//if (iResult == SOCKET_ERROR) 
			//{
			//	wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
			//}
			//iResult = recvfrom(SendSocket,RecvBuf, 10, 0, (SOCKADDR *) & RecvAddr, &RecvAddrSize);
			//if (iResult == SOCKET_ERROR) 
			//{
			//	wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
			//}
			//cout<<numt<<":received:"<<RecvBuf<<endl;
			//ZeroMemory(RecvBuf,10);
			//numt++;
			//Sleep(1000);
			cout<<numt<<":ready to send"<<endl;
			clock_t t_start,t_end;  
			t_start = clock();
			//cout<<"here"<<endl;
			unsigned char *ptr=xyz;
			//cout<<"here before copy"<<endl;
			//for(int i=0;i<pPointSet->number;i++)
			//{
			//	memcpy(buf,ptr,len);
			//	memcpy(buf+len,ptr+len,len);
			//	memcpy(buf+len*2,ptr+len*2,len);
			//	memcpy(buf+len*3,ptr+len*3,1);
			//	memcpy(buf+len*3+1,ptr+len*2+1,1);
			//	memcpy(buf+len*3+2,ptr+len*2+2,1);
			//	buf+=len*3+3;
			//	ptr+=len*3+3;
			//}
			for(int i=0;i<180;i++)
			{
				for(int j=0;j<1000;j++)
				{
					buf[j]=ptr[j];
				}
				//cout<<i<<":before send"<<endl;
				iResult = sendto(SendSocket,SendBuf, buflen, 0, (SOCKADDR *) &RecvAddr, sizeof (RecvAddr));
				if (iResult == SOCKET_ERROR) 
				{
					wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
				}
				ptr+=1000;
				//cout<<i<<":after send"<<endl;
				iResult = recvfrom(SendSocket,RecvBuf, 10, 0, (SOCKADDR *) & RecvAddr, &RecvAddrSize);
				if (iResult == SOCKET_ERROR) 
				{
					wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
				}
				//cout<<i<<":received"<<endl;
			}
			cout<<"a frame of points sent"<<endl;
			flag_send=false;
			t_end = clock() ;
			printf( "a frame of points sent time: %f ms\n",(double)(t_end-t_start) );
		}		
	}

	iResult = closesocket(SendSocket);
    if (iResult == SOCKET_ERROR) 
	{
        wprintf(L"closesocket failed with error: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }
	//---------------------------------------------
    // Clean up and quit.
    wprintf(L"Exiting.\n");
    WSACleanup();
    return 0;
}



int main(int argc, char *argv[])
{
	clock_t start,end;

	Capture capture;
	KeyPointEsti esti;
	Point3DSet *data_set, *model_set;
	int zhen=1;
	int inliers;
	int QR_flag=0;
	unsigned char *buf;
	unsigned char xyz[280000];
    DWORD dwThreadId_point;
    HANDLE hThread_point;

	hThread_point = CreateThread(NULL,0,sendpoint3d,xyz,0,&dwThreadId_point); 
	cout<<"thread ready"<<endl;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowPosition(10,320);
	glutInitWindowSize(640, 480);
	glutCreateWindow("3D image");

	capture.initial();
	while(zhen<20)
	{
		cout<<endl<<"in the loop:"<<zhen<<endl;
		if(zhen>1)
		{
			for(int i=0;i<Height;i++)
			{
				for(int j=0;j<Width;j++)
				{	
					xyzdata2[i][j][0]=xyzdata[i][j][0];
					xyzdata2[i][j][1]=xyzdata[i][j][1];
					xyzdata2[i][j][2]=xyzdata[i][j][2];
					texture2[i][j][0]=texture[i][j][0];
					texture2[i][j][1]=texture[i][j][1];
					texture2[i][j][2]=texture[i][j][2];
				}
			}
			img2=img1.clone();
		}		
		capture.capture();
		img1=capture.cImageBGR.clone();
		if(zhen==1)
		{
			zhen++;
			continue;
		}
		start=clock();
		esti.Extract();
		inliers=esti.RansacEsti();
		end=clock();
		cout<<"estimate:"<<end-start<<endl;
		data_set=icp.loadDataSet();
		model_set=icp.loadModelSet();
		//cout<<"after loading: "<<data_set->number<<","<<model_set->number<<endl;
		start=clock();
		icp.icp(data_set, model_set,esti.Rf,esti.Tf,QR_flag);
		end=clock();
		cout<<"icp:"<<end-start<<endl;

		buf=xyz;
		int len=sizeof(float);
		short temp;
		memcpy(buf,&data_set->number,4);
		cout<<"data_set->number:"<<data_set->number<<endl;
		//int testnum;
		//memcpy(&testnum,buf,len);
		//cout<<"testnum"<<testnum<<endl;
		buf+=len;
		for(int i=0;i<data_set->number;i++)
		{
			temp=(short)data_set->point[i].x;
			memcpy(buf,&temp,2);
			temp=(short)data_set->point[i].y;
			memcpy(buf+2,&temp,2);
			temp=(short)data_set->point[i].z;
			memcpy(buf+4,&temp,2);
			memcpy(buf+6,&data_set->point[i].r,1);
			memcpy(buf+7,&data_set->point[i].g,1);
			memcpy(buf+8,&data_set->point[i].b,1);
			buf+=9;
		}
		flag_send=true;

		glutDisplayFunc(renderScene); 
		zhen++;
			glutReshapeFunc (reshape);         // 窗口变化时重绘图像
			glutMouseFunc(mouse);            // 鼠标按键响应
			glutSpecialFunc(key);
			glutMotionFunc(motion);            // 鼠标移动响应
			glutPostRedisplay();                  // 刷新画面
			glutMainLoopEvent();
	}
	capture.close();

	while(true)
	{
		glutReshapeFunc (reshape);         // 窗口变化时重绘图像
		glutMouseFunc(mouse);            // 鼠标按键响应
		glutSpecialFunc(key);
		glutMotionFunc(motion);            // 鼠标移动响应
		glutPostRedisplay();                  // 刷新画面
		glutMainLoopEvent();
	}
	return 0;
}