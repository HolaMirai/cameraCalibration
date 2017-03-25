#include <cv.h>
#include <highgui.h>
#include "iostream"
#include "CCalibration.h"
using namespace std;


void main()
{
	
	CCalibration calib(cvSize(7,8),1,10);

	//从相机中获取图像标定
	//calib.calibrateFromCamera();

	//从已有图像中标定
	calib.calibrateFromFile();
	
	//运用标定结果显示修正后图像
	calib.display();
	//system("pause");

}