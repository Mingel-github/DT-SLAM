#ifndef RGBDINPUTRECTIFIER_H
#define RGBDINPUTRECTIFIER_H

#include <string>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class RGBDInputRectifier
{
public:
    RGBDInputRectifier();

    void Configure(const cv::FileStorage &settings);

    bool IsEnabled() const;
    const std::string &DomainName() const;
    std::string DomainSignature() const;

    void RectifyRGB(const cv::Mat &rawRGB, cv::Mat &rectifiedRGB);
    void RectifyRGBD(const cv::Mat &rawRGB,
                     const cv::Mat &rawDepth,
                     cv::Mat &rectifiedRGB,
                     cv::Mat &rectifiedDepth);

private:
    void EnsureMaps(const cv::Size &imageSize);

    bool mEnabled;
    cv::Mat mInputK;
    cv::Mat mInputDistortion;
    cv::Mat mOutputK;
    cv::Mat mMapX;
    cv::Mat mMapY;
    cv::Size mMapSize;
    std::string mDomainName;
};

} // namespace ORB_SLAM2

#endif // RGBDINPUTRECTIFIER_H
