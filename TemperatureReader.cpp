#include "TemperatureReader.h"
#include "GlobalDefines.h"
#include "Utils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <ios>
#include <iostream>

namespace RC
{
    TemperatureReader::TemperatureReader (std::string sFileName)
    {
        uint uiTotalElems = uiTEMPMATCOLS * uiTEMPMATROWS * uiNImages;
        float* aTempData = (float*)malloc (sizeof (float) * uiTotalElems);
        std::ifstream fin (sFileName.c_str(), std::ios_base::binary);
        fin.read ((char*)aTempData, sizeof (float) * uiTotalElems);
        m_vTempData.resize (uiTotalElems);
        std::copy (aTempData, aTempData + uiTotalElems, m_vTempData.begin());
        fin.close ();

        free (aTempData);
    }

    bool TemperatureReader::at (const uint uiIdx, std::vector<float>& vTempData)
    {
        if (uiIdx >= uiNImages)
            return false;
        uint uiFrameElems = uiTEMPMATCOLS * uiTEMPMATROWS;
        vTempData.resize (uiFrameElems);
        std::copy (&m_vTempData[uiIdx * uiFrameElems], &m_vTempData[(uiIdx + 1) * uiFrameElems], vTempData.begin());
        return true;
    }

    bool TemperatureReader::at (const std::vector<uint>& vIdxs, std::vector<float>& vTempData)
    {
        uint uiFrameElems = uiTEMPMATCOLS * uiTEMPMATROWS;
        vTempData.resize (uiFrameElems * vIdxs.size());
        for (uint ui = 0; ui < vIdxs.size(); ui++)
        {
            uint uiIdx = vIdxs[ui];
            if (uiIdx >= uiNImages)
            {
                vTempData.clear ();
                return false;
            }
            std::copy (&m_vTempData[uiIdx * uiFrameElems], &m_vTempData[(uiIdx + 1) * uiFrameElems], &vTempData[ui * uiFrameElems]);
        }
        return true;
    }

    cv::Mat TemperatureReader::matAt (const uint uiIdx)
    {
        cv::Mat mResult (uiTEMPMATROWS, uiTEMPMATCOLS, CV_32F, 0.f);
        if (uiIdx < uiNImages)
        {
            for (uint row = 0; row < mResult.rows; row ++)
            {
                for (uint col = 0; col < mResult.cols; col ++)
                {
                    float fVal =  m_vTempData [uiIdx * uiTEMPMATCOLS * uiTEMPMATROWS +  row * uiTEMPMATCOLS + col];
                    mResult.at<float> (row, col) = fVal;
                }
            }
        }
/*
        cv::Mat mFiltered (uiTEMPMATROWS, uiTEMPMATCOLS, CV_32F, 0.f);
        cv::Laplacian (mResult, mFiltered, CV_32F, 9);
        return mFiltered;
*/
        return mResult;
    }

    bool TemperatureReader::dumpTempData (const uint uiIdx, const COLOR_PALETTE &cpMap, const std::string &sFileName)
    {
        if (uiIdx >= uiNImages)
            return false;
        return computeImage (&m_vTempData[uiIdx * uiTEMPMATCOLS * uiTEMPMATROWS], cpMap, sFileName, false);
    }

    bool TemperatureReader::dumpTempData (const std::vector<uint> &vIdxs, const COLOR_PALETTE& cpMap, const std::string& sPath)
    {
        for (uint ui = 0; ui < vIdxs.size(); ui++)
        {
            std::stringstream ss;
            ss << vIdxs[ui];
            if (dumpTempData(vIdxs[ui], cpMap, sPath + ss.str() + ".bmp") == false)
                return false;
        }
        return true;
    }

}
