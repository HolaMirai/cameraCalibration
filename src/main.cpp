#include <cv.h>
#include <highgui.h>
#include "iostream"
#include "CCalibration.h"
using namespace std;


void main()
{
	
	CCalibration calib(cvSize(7,8),1,10);

	//������л�ȡͼ��궨
	//calib.calibrateFromCamera();

	//������ͼ���б궨
	calib.calibrateFromFile();
	
	//���ñ궨�����ʾ������ͼ��
	calib.display();
	//system("pause");

}