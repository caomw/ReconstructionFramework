#ifndef UTILS_H
#define UTILS_H

#include "CameraInfo.h"

namespace RC
{
    struct Ray3D
    {
        cv::Point3f m_pOrigin;
        cv::Point3f m_pDirection;
    };

    enum COLOR_PALETTE
    {
        PALETTE_GLOW,
        PALETTE_GRERED,
        PALETTE_GREY,
        PALETTE_IGREY,
        PALETTE_IRON,
        PALETTE_MED,
        PALETTE_RAIN,
        PALETTE_YELL
    };

    struct BoundaryNode
    {
        cv::Point2i pos;
        float t;

        BoundaryNode ()
        {
            pos = cv::Point2i (0, 0);
            t = 0.f;
        }

        BoundaryNode (cv::Point2i p, float t)
        {
            this->pos = p;
            this->t = t;
        }
    };

    struct BoundaryNodeComparer
    {
        bool operator () (const BoundaryNode& lhs, const BoundaryNode& rhs) const
        {
            return (lhs.t < rhs.t);
        }
    };

    void loadPalette (const COLOR_PALETTE& cpMap, std::vector<cv::Point3f>& vColors);

    void compute3DRay (const cv::Point2f& pPoint, const CameraInfo& ciCamInfo, Ray3D& rRay);

    bool computeImage (const float* pData, const COLOR_PALETTE& cpMap, const std::string& sFileName, bool bLog);

    bool computeImage (const cv::Mat& mData, const COLOR_PALETTE& cpMap, const std::string& sFileName, bool bLog);

    void computeForceField (const cv::Mat& mData, cv::Mat& forceField);

    float computeEikonal (const cv::Mat& mForceField, const cv::Mat& mTimes, const cv::Point2i& pos);

    void computeSilhouette (const cv::Mat& mData, cv::Mat& silhouette);

    void plotData (const std::vector<cv::Point3f>& vData, const std::string& sFileName);

    /// Debug methods
    void generateSurface (const cv::Mat& mData, const CameraInfo& ciCamInfo, const std::string& sFileName);
}
#endif // UTILS_H
