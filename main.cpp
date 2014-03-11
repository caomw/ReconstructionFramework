#include <iostream>
#include "TemperatureReader.h"
#include "BundlerCalibrationReader.h"
#include "Utils.h"
#include "GlobalDefines.h"

using namespace std;
using namespace RC;


int main()
{
    BundlerCalibrationReader bcrBundleReader ("/home/tomas/Documents/data/110602_152056/bundle.out");
    bcrBundleReader.dumpCameraInfo ("/home/tomas/Documents/data/110602_152056/cameras.ply");

    TemperatureReader trTempReader ("/home/tomas/Documents/data/110602_152056/temps.bin");
    cv::Mat mImg = trTempReader.matAt (0), mSilhouette = cv::Mat (mImg.cols, mImg.rows, CV_32F);
    computeImage (mImg, PALETTE_RAIN, "/home/tomas/Documents/output/test_01.bmp", false);
    computeSilhouette (mImg, mSilhouette);
    generateSurface (mImg, bcrBundleReader.m_vCamData[0], "/home/tomas/Documents/output/surf.ply");

    cout << "Hello World!" << endl;
    return 0;
}

