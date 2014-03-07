#ifndef TEMPERATUREREADER_H
#define TEMPERATUREREADER_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

#include "Utils.h"

namespace RC
{
    class TemperatureReader
    {
        private:
            std::vector<float> m_vTempData;
        public:
            TemperatureReader (std::string sFileName);
            bool at (const uint uiIdx, std::vector<float>& vTempData);
            bool at (const std::vector<uint>& vIdxs, std::vector<float>& vTempData);
            cv::Mat matAt (const uint uiIdx);
            bool dumpTempData (const uint uiIdx, const COLOR_PALETTE& cpMap, const std::string& sFileName);
            bool dumpTempData (const std::vector<uint>& vIdxs, const COLOR_PALETTE& cpMap, const std::string& sPath);
    };
}

#endif // TEMPERATUREREADER_H
