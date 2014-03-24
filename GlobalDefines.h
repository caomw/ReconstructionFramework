#ifndef GLOBALDEFINES_H
#define GLOBALDEFINES_H

#include <opencv2/core/core.hpp>

#define STR_SIZE 65535

namespace RC
{
    typedef unsigned int uint;

    static const uint uiTEMPMATCOLS = 384;
    static const uint uiTEMPMATROWS = 288;
    static const uint uiNImages = 435;
    static const std::string sPalettePath ("/home/cloud/Documents/paletten/");

    static const cv::Point2f pPrincipalPoint (192.f, 144.f);
}

#endif // GLOBALDEFINES_H
