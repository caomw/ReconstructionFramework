#ifndef CAMERAINFO_H
#define CAMERAINFO_H

#include <stdlib.h>
#include <opencv2/core/core.hpp>

namespace RC
{
    struct CameraInfo
    {
        uint m_uiCamID;
        float m_fFocalLength;
        float m_fAccuracyCoef;
        cv::Mat m_mRotMatrix;
        cv::Point3f m_pPosition;
        cv::Point3f m_pTransfVectors;
        cv::Point3f m_pDistortions;
    };
}

#endif // CAMERAINFO_H
