/****************************************** 
* Copyright (C) 2015 HolaMirai(HolaMirai@163.com) 
* All rights reserved. 
*  
* 文件名：CCalibration.h 
* 摘要：寻找Hausdorff Distance度量的最匹配位置和最匹配位置的距离 
* 当前版本：V1.0, 2015年11月17日，HolaMirai,创建该文件
* 历史记录：... 
******************************************/  

/* 
* 类定义说明 
*/  
/******************************************** 
* CCalibration类 
* CCalibration接收标定板横纵坐标角点数_board_sz, 相邻两次图像获取的时间间隔_board_dt（单位：秒）, 获取图像的总数_n_boards
* 使用calibrateFromCamera()直接从相机中获取标定板图像，并标定
* 使用calibrateFromFile()从已获取的图像集中标定相机
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
	CvSize board_sz; //标定板信息
	int n_boards;    //视场总数
	double board_dt; //相邻视场间的获取时间间隔

private:
	 
	  CvMat* intrinsic_matrix;//内参数矩阵
	  CvMat* distortion_coeffs;//畸变矩阵

};

#endif