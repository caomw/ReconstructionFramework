#include "BundlerCalibrationReader.h"
#include "GlobalDefines.h"
#include "Utils.h"

#include <string>
#include <vector>
#include <fstream>
#include <opencv2/core/core.hpp>

namespace RC
{
    BundlerCalibrationReader::BundlerCalibrationReader (std::string sFileName)
    {
        std::ifstream fin (sFileName.c_str());
        ///the first line is a comment one
        char dummy [STR_SIZE];
        fin.getline (dummy, STR_SIZE);
        ///reading number of cameras and points
        uint uiNCams, uiNPoints;
        fin >> uiNCams >> uiNPoints;

        cv::Mat mRot (3, 3, CV_32F);
        cv::Mat mT (3, 1, CV_32F);
        cv::Mat mPos (3, 1, CV_32F);
        cv::Mat mDir (3, 1, CV_32F);
        cv::Point3f tv, cdir, cpos;
        cv::Point2f cdist;

        ///CAMERAS
        for (uint uiCam = 0; uiCam < uiNCams; uiCam ++)
        {
            CameraInfo ciCamInfo;
            ///focal length + distortion parameters
            fin >> ciCamInfo.m_fFocalLength >> ciCamInfo.m_pDistortions.x >> ciCamInfo.m_pDistortions.y;
            ///rotation matrix
            fin >> mRot.at<float> (0, 0) >> mRot.at<float> (0, 1) >> mRot.at<float> (0, 2);
            fin >> mRot.at<float> (1, 0) >> mRot.at<float> (1, 1) >> mRot.at<float> (1, 2);
            fin >> mRot.at<float> (2, 0) >> mRot.at<float> (2, 1) >> mRot.at<float> (2, 2);
            ciCamInfo.m_mRotMatrix = mRot.clone ();
            ///translation vector
            fin >> ciCamInfo.m_pTransfVectors.x >> ciCamInfo.m_pTransfVectors.y >> ciCamInfo.m_pTransfVectors.z;
            mT.at<float> (0, 0) = ciCamInfo.m_pTransfVectors.x;
            mT.at<float> (1, 0) = ciCamInfo.m_pTransfVectors.y;
            mT.at<float> (2, 0) = ciCamInfo.m_pTransfVectors.z;
            ///camera position
            mPos = -mRot.t() * mT;
            ciCamInfo.m_pPosition.x = mPos.at <float> (0, 0);
            ciCamInfo.m_pPosition.y = mPos.at <float> (1, 0);
            ciCamInfo.m_pPosition.z = mPos.at <float> (2, 0);

            ciCamInfo.m_uiCamID = uiCam;
            m_vCamData.push_back (ciCamInfo);
        }
    }

    void BundlerCalibrationReader::dumpCameraInfo (std::string sOutFileName)
    {
        uint uiNCams = m_vCamData.size();
        std::ofstream fout (sOutFileName.c_str ());

        fout << "ply" << std::endl;
        fout << "format ascii 1.0" << std::endl;
        fout << "element vertex " << (uiNCams * 3) << std::endl;
        fout << "property float x" << std::endl;
        fout << "property float y" << std::endl;
        fout << "property float z" << std::endl;
        fout << "property uchar diffuse_red" << std::endl;
        fout << "property uchar diffuse_green" << std::endl;
        fout << "property uchar diffuse_blue" << std::endl;
        fout << "element face " << uiNCams << std::endl;
        fout << "property list uint8 int32 vertex_indices" << std::endl;
        fout << "end_header" << std::endl;

        Ray3D ray;
        float factor = 1.f;

        for (uint uiCam = 0; uiCam < uiNCams; uiCam ++)
        {
            compute3DRay (pPrincipalPoint, m_vCamData[uiCam], ray);

            fout << ray.m_pOrigin.x << " " << ray.m_pOrigin.y << " " << ray.m_pOrigin.z << " 128 255 255" << std::endl;
            fout << ray.m_pOrigin.x + ray.m_pDirection.x * factor << " " << ray.m_pOrigin.y + ray.m_pDirection.y * factor << " " << ray.m_pOrigin.z + ray.m_pDirection.z * factor << " 255 128 255" << std::endl;
            fout << ray.m_pOrigin.x << " " << ray.m_pOrigin.y << " " << ray.m_pOrigin.z << " 128 255 255" << std::endl;
        }
        for (uint uiCam = 0; uiCam < uiNCams; uiCam ++)
            fout << "3 " << (3 * uiCam) << " " << (3 * uiCam + 1) << " " << (3 * uiCam + 2) << std::endl;

        fout.close ();
    }
}
