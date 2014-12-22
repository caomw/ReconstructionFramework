#include <iostream>
#include "TemperatureReader.h"
#include "BundlerCalibrationReader.h"
#include "Utils.h"
#include "GlobalDefines.h"

using namespace std;
using namespace RC;


int main()
{
    BundlerCalibrationReader bcrBundleReader (sThermalPath + "110602_160725/bundle.out");
    bcrBundleReader.dumpCameraInfo (sThermalPath + "110602_160725/cameras.ply");

    TemperatureReader trTempReader (sThermalPath + "110602_160725/temps.bin");
    cv::Mat mImg = trTempReader.matAt (0), mSilhouette = cv::Mat (mImg.cols, mImg.rows, CV_32F);
//    computeImage (mImg, PALETTE_RAIN, sOutputPath + "test_01.bmp", false);
    computeSilhouette (mImg, mSilhouette);
    generateSurface (mImg, bcrBundleReader.m_vCamData[0], sOutputPath + "surf.ply");

    cout << "Hello World!" << endl;
    return 0;
}

