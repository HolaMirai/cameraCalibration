/****************************************** 
* Copyright (C) 2015 HolaMirai(HolaMirai@163.com) 
* All rights reserved. 
*  
* �ļ�����CCalibration.h 
* ժҪ��Ѱ��Hausdorff Distance��������ƥ��λ�ú���ƥ��λ�õľ��� 
* ��ǰ�汾��V1.0, 2015��11��17�գ�HolaMirai,�������ļ�
* ��ʷ��¼��... 
******************************************/  

/* 
* �ඨ��˵�� 
*/  
/******************************************** 
* CCalibration�� 
* CCalibration���ձ궨���������ǵ���_board_sz, ��������ͼ���ȡ��ʱ����_board_dt����λ���룩, ��ȡͼ�������_n_boards
* ʹ��calibrateFromCamera()ֱ�Ӵ�����л�ȡ�궨��ͼ�񣬲��궨
* ʹ��calibrateFromFile()���ѻ�ȡ��ͼ���б궨���
*  
* 
********************************************/  

#ifndef CCALIBRATION_H 
#define CCALIBRATION_H  

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>

class CCalibration
{
public:
	CCalibration(CvSize _board_sz, double _board_dt, int _n_boards = 15);
	~CCalibration();

public:
	bool doCalibrate(const CvMat* const image_points, const CvMat* const object_points,const CvMat* const point_counts, CvSize size);
	bool calibrateFromCamera();
	bool calibrateFromFile();
	void display();

protected:

private:
	CvSize board_sz; //�궨����Ϣ
	int n_boards;    //�ӳ�����
	double board_dt; //�����ӳ���Ļ�ȡʱ����

private:
	 
	  CvMat* intrinsic_matrix;//�ڲ�������
	  CvMat* distortion_coeffs;//�������

};

#endif