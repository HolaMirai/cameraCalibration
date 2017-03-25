/****************************************** 
* Copyright (C) 2015 HolaMirai(HolaMirai@163.com) 
* All rights reserved. 
*  
* �ļ�����CCalibration.cpp 
* ժҪ�������CCalibration��Ķ�ʵ���ļ�
* ��ǰ�汾��V1.0, 2015��11��17�գ� HolaMirai, �������ļ�
* ��ʷ��¼��  
******************************************/  

#include"CCalibration.h"

/* 
* �������ƣ�CCalibration
* �������ܣ��๹�캯�� 
* ������ڣ�  
* ����������궨���������ǵ���_board_sz, ��������ͼ���ȡ��ʱ����_board_dt����λ���룩, ��ȡͼ�������_n_boards
* �����������
* �� �� ֵ����
* ����˵����  
*/  
CCalibration::CCalibration(CvSize _board_sz, double _board_dt, int _n_boards)
{
	//�궨�����Ϣ
	board_sz = _board_sz;
	board_dt = _board_dt;
	n_boards = _n_boards;

	//Ϊ�궨���������ڴ�
	intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
	distortion_coeffs = cvCreateMat(4,1,CV_32FC1);
}

CCalibration::~CCalibration()
{
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
}

/* 
* �������ƣ�calibrateFromCamera
* �������ܣ�ֱ�Ӵ����ʵʱ��ȡ�궨��ͼ�����ڱ궨
* ������ڣ�  
* �����������
* �����������
* �� �� ֵ���Ƿ�궨�ɹ���true��ʾ�ɹ���false��ʾʧ��
* ����˵����  
*/  
bool CCalibration::calibrateFromCamera()
{
	cvNamedWindow("Calibration",CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Live",CV_WINDOW_AUTOSIZE);

	CvCapture* capture = cvCreateCameraCapture( 0 );//��Ҫ�궨������ͷ
	assert( capture );

	int board_n = board_sz.width * board_sz.height;//�ǵ�����
	CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);// cvMat* cvCreateMat ( int rows, int cols, int type )
	CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);//cvCreateMatԤ�������͵Ľṹ���£�CV_<bit_depth> (S|U|F)C<number_of_channels>
	CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);//cvCreateMat�����Ԫ�ؿ�����32λ����������(CV_32FC1)���������޷��ŵ�8λ��Ԫ�����������(CV_8UC3)

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];

	IplImage *image = cvQueryFrame( capture );
	//imgSize = cvGetSize(image);
	IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel   ������ͨ���Ҷ�ͼ��

	int corner_count;
	int successes = 0;//ͼ��ϵ��index
	int step, frame = 0;

	//���Կ�ʼǰ2sʱ���ͼƬ
	for (int i = 0; i < 33*2; i++)
	{
		image = cvQueryFrame(capture);
		cvShowImage("Live",image);
		cvWaitKey(30);
	}
	//��ȡ�㹻���ӳ�ͼƬ���ڱ궨
	while (successes < n_boards)
	{
		image = cvQueryFrame(capture);
		cvShowImage("Live", image);
		cvWaitKey(33);//һ֡��ʱ����

		//ÿ��board_dt��ȡһ��ͼ��
		if ( (frame++ % ((int)(33 * board_dt)) ) == 0 )
		{
			 //Find chessboard corners:
			int found = cvFindChessboardCorners(image, board_sz, corners, &corner_count,
												CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			if(found == 0)  continue;//δ��ȷ�ҵ��ǵ㣬������һ��

			//Get Subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);                //ת��Ϊ�Ҷ�ͼ��
			cvFindCornerSubPix(gray_image, corners, corner_count,      //cvFindChessboardCorners�ҵ��Ľǵ�����ǽ���ֵ��������ô˺����ﵽ�����ؾ��ȣ������һ�ζ�λ...
			cvSize(11,11),cvSize(-1,-1), cvTermCriteria(    //�ǵ�ʱ���Ե��ô˺�������ô�ᵼ�±궨��ʵ�ʴ���
			CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			 // ������ӳ�����˺õĽ����������
			 // If we got a good board, add it to our data
			 if (corner_count == board_n)
			 {
				 step = successes*board_n;
				 for( int i=step, j=0; j<board_n; ++i,++j ) 
				 {
					 CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;  // CV_MAT_ELEM �������ʾ���ÿ��Ԫ�صĺ꣬�����ֻ�Ե�ͨ��������Ч����ͨ���ᱨ��...
					 CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;  //CV_MAT_ELEM( matrix, elemtype, row, col )
					 CV_MAT_ELEM(*object_points,float,i,0) = j/board_sz.width;     //matrix��Ҫ���ʵľ���,elemtype������Ԫ�ص�����,row����Ҫ����Ԫ�ص�����,col����Ҫ����Ԫ�ص�����
					 CV_MAT_ELEM(*object_points,float,i,1) = j%board_sz.width;
					 CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
				 }
				 CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;    
				 successes++;
			 }

			 //Draw corners
			 cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);//foundΪcvFindChessboardCorners�ķ���ֵ
			 char text[10];
			 sprintf(text,"%d/%d", successes,n_boards);
			 CvFont font = cvFont(2,2);
			 cvPutText(image,text,cvPoint(40,40),&font,cvScalar(0,0,255));
			 cvShowImage( "Calibration", image );
		}
	}

	//��ȡ���㹻���ӳ���������ȡ
	cvDestroyWindow("Calibration");
	cvDestroyWindow("Live");
	//����
	doCalibrate(image_points, object_points, point_counts, cvGetSize(image));
	
	//����
	delete []corners;
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	cvReleaseImage(&gray_image);
	cvReleaseCapture(&capture);

	return true;
}/*  calibrateFromCamera()   */

/* 
* �������ƣ�calibrateFromCamera
* �������ܣ������ѻ�ȡ��ͼ���ļ���.bmp��ʽ�����궨���
* ������ڣ�  
* �����������
* �����������
* �� �� ֵ���Ƿ�궨�ɹ���true��ʾ�ɹ���false��ʾʧ��
* ����˵���� ֻ����.bmp��ʽ��ͼƬ����ͼƬ�ߴ�Ҫ��ͬ����Ҫ�궨������ʽͼƬ���뽫�������ڵ�.bmp�滻��.jpg
*            �ļ�ͳһ������ʽΪ calib_N.bmp,����N�����0��ʼ
*/  
bool CCalibration::calibrateFromFile()
{
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("FileImage", CV_WINDOW_AUTOSIZE);
	int board_n = board_sz.width * board_sz.height;//�ǵ�����
	CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);// cvMat* cvCreateMat ( int rows, int cols, int type )
	CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);//cvCreateMatԤ�������͵Ľṹ���£�CV_<bit_depth> (S|U|F)C<number_of_channels>
	CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);//cvCreateMat�����Ԫ�ؿ�����32λ����������(CV_32FC1)���������޷��ŵ�8λ��Ԫ�����������(CV_8UC3)

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];

	char imgName[20] = "calib_0.bmp";
	IplImage *image = cvLoadImage(imgName,1);
	IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel   ������ͨ���Ҷ�ͼ��

	int corner_count;
	int successes = 0, index = 0;//ͼ��ϵ��index
	int step;

	//��ȡ�㹻���ӳ�ͼƬ���ڱ궨
	while (successes < n_boards)
	{
		sprintf(imgName, "calib_%d.bmp",index++);
		image = cvLoadImage(imgName,1);
		if ( !image ) break; //�޴�ͼƬ����ֹͣ
		cvWaitKey(1000*board_dt);//һ֡��ʱ����
		
		//Find chessboard corners:
		int found = cvFindChessboardCorners(image, board_sz, corners, &corner_count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if(found == 0)  continue;//δ��ȷ�ҵ��ǵ㣬������һ��

		//Get Subpixel accuracy on those corners
		cvCvtColor(image, gray_image, CV_BGR2GRAY);                //ת��Ϊ�Ҷ�ͼ��
		cvFindCornerSubPix(gray_image, corners, corner_count,      //cvFindChessboardCorners�ҵ��Ľǵ�����ǽ���ֵ��������ô˺����ﵽ�����ؾ��ȣ������һ�ζ�λ...
							cvSize(11,11),cvSize(-1,-1), cvTermCriteria(    //�ǵ�ʱ���Ե��ô˺�������ô�ᵼ�±궨��ʵ�ʴ���
							CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

		// ������ӳ�����˺õĽ����������
		// If we got a good board, add it to our data
		if (corner_count == board_n)
		{
			step = successes*board_n;
			for( int i=step, j=0; j<board_n; ++i,++j ) 
			{
				CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;  // CV_MAT_ELEM �������ʾ���ÿ��Ԫ�صĺ꣬�����ֻ�Ե�ͨ��������Ч����ͨ���ᱨ��...
				CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;  //CV_MAT_ELEM( matrix, elemtype, row, col )
				CV_MAT_ELEM(*object_points,float,i,0) = j/board_sz.width;     //matrix��Ҫ���ʵľ���,elemtype������Ԫ�ص�����,row����Ҫ����Ԫ�ص�����,col����Ҫ����Ԫ�ص�����
				CV_MAT_ELEM(*object_points,float,i,1) = j%board_sz.width;
				CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;    
			successes++;
		}

		//Draw corners
		cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);//foundΪcvFindChessboardCorners�ķ���ֵ
		char text[10];
		sprintf(text,"%d/%d", successes,n_boards);
		CvFont font = cvFont(2,2);
		cvPutText(image,text,cvPoint(40,40),&font,cvScalar(0,0,255));
		cvShowImage( "Calibration", image );
	}

	//��ȡ���㹻���ӳ���������ȡ
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
* �������ƣ�doCalibrate 
* �������ܣ���������ڲ����ͻ������ 
* ������ڣ�
* ����������洢ͼ��ǵ����꣨���������꣩��Ϣ�ľ���ָ��image_points���洢�б궨��ǵ����꣨�������꣩��Ϣ�ľ���ָ��object_points
*			�洢�и�ͼ��Ѱ�ҵ��Ľǵ������Ϣ�ľ���ָ��point_counts��ͼ��ߴ�size
* ����������� 
* �� �� ֵ�� �Ƿ�ɹ���true�ɹ���falseʧ��
* ����˵���� �궨���ͬʱ�洢����ǰĿ¼Intrinsics.xml��Distortion.xml�ļ���
*/  
bool CCalibration::doCalibrate(const CvMat* const image_points, const CvMat* const object_points,const CvMat* const point_counts, CvSize size)
{
	//****************************��ʼ�궨*************************//

	// ��ʼ���ڲ��������fx��fyΪ1.0f
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

	//**************����궨����*************//
	//CALIBRATE THE CAMERA!
	cvCalibrateCamera2( object_points, image_points, point_counts,  size,
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL,0  //CV_CALIB_FIX_ASPECT_RATIO
		);

	//SAVE THE INTRINSICS AND DISTORTIONS
	cvSave("Intrinsics.xml",intrinsic_matrix);//��������ͷ�ڲ���
	cvSave("Distortion.xml",distortion_coeffs);//��������ͷ�����

	return true;
}/* doCalibrate() */


/* 
* �������ƣ�display 
* �������ܣ����ݱ궨��������ʾ���������Ƶͼ�� 
* ������ڣ�
* �����������
* ����������� 
* �� �� ֵ�� 
* ����˵����  
*/  
void CCalibration::display()
{
	cvNamedWindow("Undistort", CV_WINDOW_AUTOSIZE);//��ʾ������ͼ��

	CvCapture *capture = cvCreateCameraCapture(0);
	IplImage *frame = cvQueryFrame(capture);
	IplImage *imgUndistort = cvCreateImage(cvGetSize(frame),frame->depth,frame->nChannels);

	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");//��������ͷ�ڲ���
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");//��������ͷ�����

	// Build the undistort map which we will use for all subsequent frames.
	IplImage* mapx = cvCreateImage( cvGetSize(frame), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize(frame), IPL_DEPTH_32F, 1 );

	//�������ӳ�� ����������ͷ�ڡ����������������û����Щ����Ļ�������ͷ��õ�����ͼ��
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