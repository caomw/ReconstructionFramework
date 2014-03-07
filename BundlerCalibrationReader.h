#ifndef BUNDLERCALIBRATIONREADER_H
#define BUNDLERCALIBRATIONREADER_H

#include <string>
#include <vector>
#include "CameraInfo.h"

namespace RC
{
    class BundlerCalibrationReader
    {
        public:
            std::vector<CameraInfo> m_vCamData;

            BundlerCalibrationReader (std::string sFileName);
            void dumpCameraInfo (std::string sOutFileName);
    };
}

#endif // BUNDLERCALIBRATIONREADER_H
