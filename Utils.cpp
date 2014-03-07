#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <set>
#include <limits>
#include <math.h>

#include "Utils.h"
#include "GlobalDefines.h"

namespace RC
{
    void loadPalette (const COLOR_PALETTE& cpMap, std::vector<cv::Point3f>& vColors)
    {
        std::string sImgName = "";
        switch (cpMap)
        {
            case PALETTE_GLOW:
                sImgName = "GLOW.bmp";
                break;
            case PALETTE_GRERED:
                sImgName = "GRERED.bmp";
                break;
            case PALETTE_GREY:
                sImgName = "GREY.bmp";
                break;
            case PALETTE_IGREY:
                sImgName = "IGREY.bmp";
                break;
            case PALETTE_IRON:
                sImgName = "IRON.bmp";
                break;
            case PALETTE_MED:
                sImgName = "MED.bmp";
                break;
            case PALETTE_RAIN:
                sImgName = "RAIN.bmp";
                break;
            case PALETTE_YELL:
                sImgName = "YELL.bmp";
                break;
            default:
                sImgName = "GREY.bmp";
        }

        std::string sPath = sPalettePath + sImgName;
        cv::Mat mImg = cv::imread (sPath, CV_LOAD_IMAGE_COLOR);

        uint iNrows = mImg.rows, iNcols = mImg.cols;
        vColors.resize (iNrows);
        for ( uint row = 0; row < iNrows; row ++ )
        {
            cv::Point3f color;
            color.x = color.y = color.z = 0.f;
            for ( uint col = 0; col < iNcols; col ++ )
            {
                color.x += (float)mImg.data [row * mImg.step + col * mImg.channels() + 2];
                color.y += (float)mImg.data [row * mImg.step + col * mImg.channels() + 1];
                color.z += (float)mImg.data [row * mImg.step + col * mImg.channels() + 0];
            }
            color.x /= (float)iNcols;
            color.y /= (float)iNcols;
            color.z /= (float)iNcols;

            vColors[row] = color;
        }
    }

    void compute3DRay (const cv::Point2f& pPoint, const CameraInfo& ciCamInfo, Ray3D& rRay)
    {
        rRay.m_pOrigin = ciCamInfo.m_pPosition;
        ///direction (local coordinate system)
        cv::Mat mDir (3, 1, CV_32F);
        mDir.at <float> (0, 0) = pPoint.x - pPrincipalPoint.x;
        mDir.at <float> (1, 0) = pPrincipalPoint.y - pPoint.y;
        mDir.at <float> (2, 0) = -ciCamInfo.m_fFocalLength;
        cv::normalize (mDir, mDir);
        ///taking it to the global coordinate system
        mDir = ciCamInfo.m_mRotMatrix.t () * mDir;
        cv::normalize (mDir, mDir);

        rRay.m_pDirection.x = mDir.at <float> (0, 0);
        rRay.m_pDirection.y = mDir.at <float> (1, 0);
        rRay.m_pDirection.z = mDir.at <float> (2, 0);
    }

    bool computeImage (const cv::Mat& mData, const COLOR_PALETTE& cpMap, const std::string& sFileName, bool bLog)
    {
        float* pData = (float*)malloc (sizeof (float) * mData.rows * mData.cols);
        for (uint row = 0; row < mData.rows; row ++)
            for (uint col = 0; col < mData.cols; col ++)
                pData [row * mData.cols + col] = mData.at<float>(row, col);
        bool bResult = computeImage (pData, cpMap, sFileName, bLog);
        free (pData);
        return bResult;
    }

    bool computeImage (const float* pData, const COLOR_PALETTE& cpMap, const std::string& sFileName, bool bLog)
    {
        std::vector<cv::Point3f> vColors;
        float fMin, fMax, fRatio, fVal;
        uint uiIdx;
        loadPalette (cpMap, vColors);

        fMin = *std::min_element (pData, pData + (uiTEMPMATROWS * uiTEMPMATCOLS));
        fMax = *std::max_element (pData, pData + (uiTEMPMATROWS * uiTEMPMATCOLS));

        cv::Mat mImg (uiTEMPMATROWS, uiTEMPMATCOLS, CV_8UC3);
        if (bLog)
        {
            fMin = log (fMin);
            fMax = log (fMax);
        }

        for (uint row = 0; row < uiTEMPMATROWS; row ++)
        {
            for (uint col = 0; col < uiTEMPMATCOLS; col ++)
            {
                fVal =  pData [row * uiTEMPMATCOLS + col];
                if (bLog)
                    fVal = log (fVal);
                fRatio = (fVal - fMin) / (fMax - fMin);
                uiIdx = uint (fRatio * (float)(vColors.size() - 1));
                mImg.data [row * mImg.step + col * mImg.channels() + 0] = uint(vColors[uiIdx].z);
                mImg.data [row * mImg.step + col * mImg.channels() + 1] = uint(vColors[uiIdx].y);
                mImg.data [row * mImg.step + col * mImg.channels() + 2] = uint(vColors[uiIdx].x);
            }
        }

        cv::imwrite (sFileName, mImg);
        return true;
    }

    void plotData (const std::vector<cv::Point3f>& vData, const std::string& sFileName)
    {
        std::ofstream fout (sFileName.c_str ());

        fout << "ply" << std::endl;
        fout << "format ascii 1.0" << std::endl;
        fout << "element vertex " << vData.size() << std::endl;
        fout << "property float x" << std::endl;
        fout << "property float y" << std::endl;
        fout << "property float z" << std::endl;
        fout << "end_header" << std::endl;

        for (uint ui = 0; ui < vData.size(); ui ++)
            fout << vData[ui].x << " " << vData[ui].y << " " << vData[ui].z  << std::endl;
        fout.close ();
    }

    float computeEikonal (const cv::Mat& mForceField, const cv::Mat& mTimes, const cv::Point2i& pos)
    {
        float Tij, T1, T2, fij = mForceField.at<float> (pos.y, pos.x);

        if (pos.y == 0)
            T1 = mTimes.at<float> (pos.y + 1, pos.x);
        else if (pos.y == mTimes.rows - 1)
            T1 = mTimes.at<float> (pos.y - 1, pos.x);
        else
            T1 = std::min (mTimes.at<float>(pos.y - 1, pos.x), mTimes.at<float>(pos.y + 1, pos.x));

        if (pos.x == 0)
            T2 = mTimes.at<float> (pos.y, pos.x + 1);
        else if (pos.x == mTimes.cols - 1)
            T2 = mTimes.at<float> (pos.y, pos.x - 1);
        else
            T2 = std::min (mTimes.at<float>(pos.y, pos.x - 1), mTimes.at<float>(pos.y, pos.x + 1));

        if (T1 == std::numeric_limits<float>::max() && T2 == std::numeric_limits<float>::max())
            throw ReconstructionFrameworkException ("unexpected situation");

        if (T1 == std::numeric_limits<float>::max () ||
            T2 == std::numeric_limits<float>::max ())
            Tij = fij + std::min (T1, T2);
        else
        {
            float s1, s2;
            float D = 4.f * ((T1 + T2) * (T1 + T2)) - 8.f * (T1 * T1 + T2 * T2);
            s1 = (2.f * (T1 + T2) + sqrt (D)) / 4.f;
            s2 = (2.f * (T1 + T2) - sqrt (D)) / 4.f;
            Tij = std::min (s1, s2);
        }

        return Tij;
    }

    void computeForceField(const cv::Mat& mData, cv::Mat& forceField)
    {
        float std_dev = .1f;
        float c1 = 1.f / (std_dev * std::sqrt(2.f * M_PI));
        float c2 = 1.f / (2.f * std_dev * std_dev);
        cv::Laplacian (mData, forceField, CV_32F, 9);
        for (int r = 0; r < forceField.rows; r ++)
        {
            for (int c = 0; c  < forceField.cols; c++)
            {
                float gr = forceField.at<float> (r, c);
                forceField.at<float> (r, c) = c1 * exp (-c2 * (gr * gr));
            }
        }
    }

    void computeSilhouette (const cv::Mat& mData, cv::Mat& silhouette)
    {
        /// posible directions
        cv::Point2i disp [4];
        disp[0] = cv::Point2i (-1, 0);
        disp[1] = cv::Point2i (0, -1);
        disp[2] = cv::Point2i (1,  0);
        disp[3] = cv::Point2i (0,  1);
        std::vector<bool> accepted (uiTEMPMATCOLS * uiTEMPMATROWS, false);
        std::multiset<BoundaryNode, BoundaryNodeComparer> boundary;
        /// computing the image gradient = force field
        cv::Mat mForce (uiTEMPMATROWS, uiTEMPMATCOLS, CV_32F, 0.f);
        computeForceField (mData, mForce);
        silhouette = std::numeric_limits<float>::max();
        /// initializing the algorithm in the top left corner (0, 0)



        boundary.insert(BoundaryNode());
        while (!boundary.empty())
        {
            std::multiset<BoundaryNode, BoundaryNodeComparer>::iterator it = boundary.begin();
            BoundaryNode head = *it;
            silhouette.at<float>(head.pos.y, head.pos.x) = head.t;
            accepted[head.pos.y * uiTEMPMATCOLS + head.pos.x] = true;
            /// checking the neighboring elements
            for (unsigned int i = 0; i < 4; i++)
            {
                cv::Point2i pos = head.pos + disp[i];
                if (pos.x >= 0 && pos.x < uiTEMPMATROWS &&
                    pos.y >= 0 && pos.y < uiTEMPMATROWS &&
                    !accepted[pos.y * uiTEMPMATCOLS + pos.x])
                {

                }
            }
            boundary.erase(it);
        }

    }

    /// Debug methods

    void generateSurface (const cv::Mat &mData, const CameraInfo &ciCamInfo, const std::string &sFileName)
    {
        float fMinDist = 1.f, fMaxDist = 2.f;
        float fMinTemp = 20.f, fMaxTemp = 40.f;
        std::vector<cv::Point3f> vSurface;
        for (uint row = 0; row < mData.rows; row ++)
            for (uint col = 0; col < mData.cols; col ++)
            {
                float fTemp = mData.at<float> (row, col);
                if (fTemp > fMinTemp && fTemp < fMaxTemp)
                {
                    float ratio = (fTemp - fMinTemp) / (fMaxTemp - fMinTemp);
                    float dist = fMinDist + (fMaxDist - fMinDist) * ratio;
                    Ray3D ray;
                    compute3DRay(cv::Point2f (col, row), ciCamInfo, ray);
                    cv::Point3f surfPoint = ray.m_pOrigin + ray.m_pDirection * dist;
                    vSurface.push_back (surfPoint);
                }
            }
        plotData (vSurface, sFileName);
    }
}

