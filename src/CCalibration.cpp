/****************************************** 
* Copyright (C) 2015 HolaMirai(HolaMirai@163.com) 
* All rights reserved. 
*  
* 文件名：CCalibration.cpp 
* 摘要：相机标CCalibration类的定实现文件
* 当前版本：V1.0, 2015年11月17日， HolaMirai, 创建该文件
* 历史记录：  
******************************************/  

#include"CCalibration.h"

/* 
* 函数名称：CCalibration
* 函数功能：类构造函数 
* 函数入口：  
* 输入参数：标定板横纵坐标角点数_board_sz, 相邻两次图像获取的时间间隔_board_dt（单位：秒）, 获取图像的总数_n_boards
* 输出参数：无
* 返 回 值：无
* 其它说明：  
*/  
CCalibration::CCalibration(CvSize _board_sz, double _board_dt, int _n_boards)
{
	//标定板的信息
	board_sz = _board_sz;
	board_dt = _board_dt;
	n_boards = _n_boards;

	//为标定参数分配内存
	intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
	distortion_coeffs = cvCreateMat(4,1,CV_32FC1);
}

CCalibration::~CCalibration()
{
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
}

/* 
* 函数名称：calibrateFromCamera
* 函数功能：直接从相机实时获取标定板图像，用于标定
* 函数入口：  
* 输入参数：五
* 输出参数：无
* 返 回 值：是否标定成功，true表示成功，false表示失败
* 其它说明：  
*/  
bool CCalibration::calibrateFromCamera()
{
	cvNamedWindow("Calibration",CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Live",CV_WINDOW_AUTOSIZE);

	CvCapture* capture = cvCreateCameraCapture( 0 );//将要标定的摄像头
	assert( capture );

	int board_n = board_sz.width * board_sz.height;//角点总数
	CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);// cvMat* cvCreateMat ( int rows, int cols, int type )
	CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);//cvCreateMat预定义类型的结构如下：CV_<bit_depth> (S|U|F)C<number_of_channels>
	CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);//cvCreateMat矩阵的元素可以是32位浮点型数据(CV_32FC1)，或者是无符号的8位三元组的整型数据(CV_8UC3)

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];

	IplImage *image = cvQueryFrame( capture );
	//imgSize = cvGetSize(image);
	IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel   创建单通道灰度图像

	int corner_count;
	int successes = 0;//图像系列index
	int step, frame = 0;

	//忽略开始前2s时间的图片
	for (int i = 0; i < 33*2; i++)
	{
		image = cvQueryFrame(capture);
		cvShowImage("Live",image);
		cvWaitKey(30);
	}
	//获取足够多视场图片用于标定
	while (successes < n_boards)
	{
		image = cvQueryFrame(capture);
		cvShowImage("Live", image);
		cvWaitKey(33);//一帧的时间间隔

		//每隔board_dt秒取一张图像
		if ( (frame++ % ((int)(33 * board_dt)) ) == 0 )
		{
			 //Find chessboard corners:
			int found = cvFindChessboardCorners(image, board_sz, corners, &corner_count,
												CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			if(found == 0)  continue;//未正确找到角点，继续下一次

			//Get Subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);                //转换为灰度图像
			cvFindCornerSubPix(gray_image, corners, corner_count,      //cvFindChessboardCorners找到的角点仅仅是近似值，必须调用此函数达到亚像素精度，如果第一次定位...
			cvSize(11,11),cvSize(-1,-1), cvTermCriteria(    //角点时忽略调用此函数，那么会导致标定的实际错误
			CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			 // 如果该视场获得了好的结果，保存它
			 // If we got a good board, add it to our data
			 if (corner_count == board_n)
			 {
				 step = successes*board_n;
				 for( int i=step, j=0; j<board_n; ++i,++j ) 
				 {
					 CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;  // CV_MAT_ELEM 用来访问矩阵每个元素的宏，这个宏只对单通道矩阵有效，多通道会报错...
					 CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;  //CV_MAT_ELEM( matrix, elemtype, row, col )
					 CV_MAT_ELEM(*object_points,float,i,0) = j/board_sz.width;     //matrix：要访问的矩阵,elemtype：矩阵元素的类型,row：所要访问元素的行数,col：所要访问元素的列数
					 CV_MAT_ELEM(*object_points,float,i,1) = j%board_sz.width;
					 CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
				 }
				 CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;    
				 successes++;
			 }

			 //Draw corners
			 cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);//found为cvFindChessboardCorners的返回值
			 char text[10];
			 sprintf(text,"%d/%d", successes,n_boards);
			 CvFont font = cvFont(2,2);
			 cvPutText(image,text,cvPoint(40,40),&font,cvScalar(0,0,255));
			 cvShowImage( "Calibration", image );
		}
	}

	//获取了足够多视场，结束获取
	cvDestroyWindow("Calibration");
	cvDestroyWindow("Live");
	//计算
	doCalibrate(image_points, object_points, point_counts, cvGetSize(image));
	
	//结束
	delete []corners;
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	cvReleaseImage(&gray_image);
	cvReleaseCapture(&capture);

	return true;
}/*  calibrateFromCamera()   */

/* 
* 函数名称：calibrateFromCamera
* 函数功能：根据已获取的图像文件（.bmp格式），标定相机
* 函数入口：  
* 输入参数：无
* 输出参数：无
* 返 回 值：是否标定成功，true表示成功，false表示失败
* 其它说明： 只接受.bmp格式的图片，且图片尺寸要相同，若要标定其他格式图片，请将本函数内的.bmp替换成.jpg
*            文件统一命名格式为 calib_N.bmp,其中N必须从0开始
*/  
bool CCalibration::calibrateFromFile()
{
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("FileImage", CV_WINDOW_AUTOSIZE);
	int board_n = board_sz.width * board_sz.height;//角点总数
	CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);// cvMat* cvCreateMat ( int rows, int cols, int type )
	CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);//cvCreateMat预定义类型的结构如下：CV_<bit_depth> (S|U|F)C<number_of_channels>
	CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);//cvCreateMat矩阵的元素可以是32位浮点型数据(CV_32FC1)，或者是无符号的8位三元组的整型数据(CV_8UC3)

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];

	char imgName[20] = "calib_0.bmp";
	IplImage *image = cvLoadImage(imgName,1);
	IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel   创建单通道灰度图像

	int corner_count;
	int successes = 0, index = 0;//图像系列index
	int step;

	//获取足够多视场图片用于标定
	while (successes < n_boards)
	{
		sprintf(imgName, "calib_%d.bmp",index++);
		image = cvLoadImage(imgName,1);
		if ( !image ) break; //无此图片，则停止
		cvWaitKey(1000*board_dt);//一帧的时间间隔
		
		//Find chessboard corners:
		int found = cvFindChessboardCorners(image, board_sz, corners, &corner_count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if(found == 0)  continue;//未正确找到角点，继续下一次

		//Get Subpixel accuracy on those corners
		cvCvtColor(image, gray_image, CV_BGR2GRAY);                //转换为灰度图像
		cvFindCornerSubPix(gray_image, corners, corner_count,      //cvFindChessboardCorners找到的角点仅仅是近似值，必须调用此函数达到亚像素精度，如果第一次定位...
							cvSize(11,11),cvSize(-1,-1), cvTermCriteria(    //角点时忽略调用此函数，那么会导致标定的实际错误
							CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

		// 如果该视场获得了好的结果，保存它
		// If we got a good board, add it to our data
		if (corner_count == board_n)
		{
			step = successes*board_n;
			for( int i=step, j=0; j<board_n; ++i,++j ) 
			{
				CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;  // CV_MAT_ELEM 用来访问矩阵每个元素的宏，这个宏只对单通道矩阵有效，多通道会报错...
				CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;  //CV_MAT_ELEM( matrix, elemtype, row, col )
				CV_MAT_ELEM(*object_points,float,i,0) = j/board_sz.width;     //matrix：要访问的矩阵,elemtype：矩阵元素的类型,row：所要访问元素的行数,col：所要访问元素的列数
				CV_MAT_ELEM(*object_points,float,i,1) = j%board_sz.width;
				CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;    
			successes++;
		}

		//Draw corners
		cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);//found为cvFindChessboardCorners的返回值
		char text[10];
		sprintf(text,"%d/%d", successes,n_boards);
		CvFont font = cvFont(2,2);
		cvPutText(image,text,cvPoint(40,40),&font,cvScalar(0,0,255));
		cvShowImage( "Calibration", image );
	}

	//获取了足够多视场，结束获取
	cvDestroyWindow("Calibration");
	//cvDestroyWindow("FileImage");
	doCalibrate(image_points, object_points, point_counts, cvGetSize(image));

	delete []corners;
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	cvReleaseImage(&image);
	cvReleaseImage(&gray_image);
	return true;
} /* calibrateFromFile() */

/* 
* 函数名称：doCalibrate 
* 函数功能：计算相机内参数和畸变参数 
* 函数入口：
* 输入参数：存储图像角点坐标（成像仪坐标）信息的矩阵指针image_points，存储有标定板角点坐标（世界坐标）信息的矩阵指针object_points
*			存储有各图像寻找到的角点个数信息的矩阵指针point_counts，图像尺寸size
* 输出参数：无 
* 返 回 值： 是否成功，true成功，false失败
* 其它说明： 标定结果同时存储到当前目录Intrinsics.xml，Distortion.xml文件中
*/  
bool CCalibration::doCalibrate(const CvMat* const image_points, const CvMat* const object_points,const CvMat* const point_counts, CvSize size)
{
	//****************************开始标定*************************//

	// 初始化内参数矩阵的fx和fy为1.0f
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

	//**************计算标定参数*************//
	//CALIBRATE THE CAMERA!
	cvCalibrateCamera2( object_points, image_points, point_counts,  size,
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL,0  //CV_CALIB_FIX_ASPECT_RATIO
		);

	//SAVE THE INTRINSICS AND DISTORTIONS
	cvSave("Intrinsics.xml",intrinsic_matrix);//保存摄像头内参数
	cvSave("Distortion.xml",distortion_coeffs);//保存摄像头外参数

	return true;
}/* doCalibrate() */


/* 
* 函数名称：display 
* 函数功能：根据标定参数，显示修正后的视频图像 
* 函数入口：
* 输入参数：无
* 输出参数：无 
* 返 回 值： 
* 其它说明：  
*/  
void CCalibration::display()
{
	cvNamedWindow("Undistort", CV_WINDOW_AUTOSIZE);//显示修正后图像

	CvCapture *capture = cvCreateCameraCapture(0);
	IplImage *frame = cvQueryFrame(capture);
	IplImage *imgUndistort = cvCreateImage(cvGetSize(frame),frame->depth,frame->nChannels);

	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");//加载摄像头内参数
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");//加载摄像头外参数

	// Build the undistort map which we will use for all subsequent frames.
	IplImage* mapx = cvCreateImage( cvGetSize(frame), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize(frame), IPL_DEPTH_32F, 1 );

	//计算畸变映射 即根据摄像头内、外参数，计算出如果没有这些畸变的话，摄像头获得的理想图像
	cvInitUndistortMap(intrinsic, distortion, mapx, mapy);

	while(cvWaitKey(33) != 27) //ESC
	{
		frame = cvQueryFrame(capture);
		cvRemap( frame, imgUndistort, mapx, mapy);
		cvShowImage("Undistort", imgUndistort);
	}

	cvReleaseCapture(&capture);
	cvReleaseImage(&imgUndistort);
	cvDestroyWindow("Undistort");

}/* display()  */