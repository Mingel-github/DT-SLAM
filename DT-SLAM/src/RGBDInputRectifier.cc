#include "RGBDInputRectifier.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace
{

double ReadRequiredFinite(
    const cv::FileStorage &settings,
    const char *name)
{
    const cv::FileNode node = settings[name];
    if(node.empty())
        throw std::invalid_argument(std::string("Missing required setting: ")+name);

    const double value = static_cast<double>(node);
    if(!std::isfinite(value))
        throw std::invalid_argument(std::string("Non-finite setting: ")+name);
    return value;
}

double ReadOptionalFinite(
    const cv::FileStorage &settings,
    const char *name,
    const double defaultValue)
{
    const cv::FileNode node = settings[name];
    if(node.empty())
        return defaultValue;

    const double value = static_cast<double>(node);
    if(!std::isfinite(value))
        throw std::invalid_argument(std::string("Non-finite setting: ")+name);
    return value;
}

void RequirePositiveFocalLengths(const cv::Mat &K, const char *modelName)
{
    if(K.at<double>(0,0)<=0.0 || K.at<double>(1,1)<=0.0)
    {
        throw std::invalid_argument(
            std::string(modelName)+" focal lengths must be positive");
    }
}

} // namespace

namespace ORB_SLAM2
{

RGBDInputRectifier::RGBDInputRectifier()
    : mEnabled(false),
      mMapSize(),
      mDomainName("input_native")
{
}

void RGBDInputRectifier::Configure(const cv::FileStorage &settings)
{
    mEnabled = false;
    mInputK.release();
    mInputDistortion.release();
    mOutputK.release();
    mMapX.release();
    mMapY.release();
    mMapSize = cv::Size();
    mDomainName = "input_native";

    const cv::FileNode enableNode =
        settings["RGBD.InputRectification.Enable"];
    if(enableNode.empty() || static_cast<int>(enableNode)==0)
        return;

    const double inputFx = ReadRequiredFinite(
        settings,"RGBD.InputRectification.fx");
    const double inputFy = ReadRequiredFinite(
        settings,"RGBD.InputRectification.fy");
    const double inputCx = ReadRequiredFinite(
        settings,"RGBD.InputRectification.cx");
    const double inputCy = ReadRequiredFinite(
        settings,"RGBD.InputRectification.cy");
    const double inputK1 = ReadRequiredFinite(
        settings,"RGBD.InputRectification.k1");
    const double inputK2 = ReadRequiredFinite(
        settings,"RGBD.InputRectification.k2");
    const double inputP1 = ReadRequiredFinite(
        settings,"RGBD.InputRectification.p1");
    const double inputP2 = ReadRequiredFinite(
        settings,"RGBD.InputRectification.p2");
    const double inputK3 = ReadRequiredFinite(
        settings,"RGBD.InputRectification.k3");

    mInputK = cv::Mat::eye(3,3,CV_64F);
    mInputK.at<double>(0,0) = inputFx;
    mInputK.at<double>(1,1) = inputFy;
    mInputK.at<double>(0,2) = inputCx;
    mInputK.at<double>(1,2) = inputCy;
    mInputDistortion = (cv::Mat_<double>(5,1) <<
        inputK1,inputK2,inputP1,inputP2,inputK3);

    mOutputK = cv::Mat::eye(3,3,CV_64F);
    mOutputK.at<double>(0,0) =
        ReadRequiredFinite(settings,"Camera.fx");
    mOutputK.at<double>(1,1) =
        ReadRequiredFinite(settings,"Camera.fy");
    mOutputK.at<double>(0,2) =
        ReadRequiredFinite(settings,"Camera.cx");
    mOutputK.at<double>(1,2) =
        ReadRequiredFinite(settings,"Camera.cy");

    RequirePositiveFocalLengths(mInputK,"input camera");
    RequirePositiveFocalLengths(mOutputK,"output camera");

    const double cameraDistortion[] = {
        ReadOptionalFinite(settings,"Camera.k1",0.0),
        ReadOptionalFinite(settings,"Camera.k2",0.0),
        ReadOptionalFinite(settings,"Camera.p1",0.0),
        ReadOptionalFinite(settings,"Camera.p2",0.0),
        ReadOptionalFinite(settings,"Camera.k3",0.0)
    };
    for(double value : cameraDistortion)
    {
        if(std::abs(value)>1e-12)
        {
            throw std::invalid_argument(
                "RGBD input rectification requires zero Camera distortion "
                "after rectification");
        }
    }

    if(cv::norm(mInputK-mOutputK,cv::NORM_INF)>1e-6)
    {
        throw std::invalid_argument(
            "G2-4B requires P=K: RGBD input and Camera intrinsics must match");
    }

    mEnabled = true;
    mDomainName = "undistorted_pinhole";
}

bool RGBDInputRectifier::IsEnabled() const
{
    return mEnabled;
}

const std::string &RGBDInputRectifier::DomainName() const
{
    return mDomainName;
}

std::string RGBDInputRectifier::DomainSignature() const
{
    if(!mEnabled)
        return "domain=input_native rectification=disabled";

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "domain=" << mDomainName
           << " size=" << mMapSize.width << "x" << mMapSize.height
           << " input_K="
           << mInputK.at<double>(0,0) << ","
           << mInputK.at<double>(1,1) << ","
           << mInputK.at<double>(0,2) << ","
           << mInputK.at<double>(1,2)
           << " input_D="
           << mInputDistortion.at<double>(0) << ","
           << mInputDistortion.at<double>(1) << ","
           << mInputDistortion.at<double>(2) << ","
           << mInputDistortion.at<double>(3) << ","
           << mInputDistortion.at<double>(4)
           << " output_P=K rgb_interp=linear depth_interp=nearest";
    return stream.str();
}

void RGBDInputRectifier::EnsureMaps(const cv::Size &imageSize)
{
    if(!mEnabled)
        return;
    if(imageSize.width<=0 || imageSize.height<=0)
        throw std::invalid_argument("RGBD rectification image size is invalid");
    if(!mMapX.empty() && imageSize==mMapSize)
        return;

    cv::initUndistortRectifyMap(
        mInputK,mInputDistortion,cv::Mat(),mOutputK,imageSize,
        CV_32FC1,mMapX,mMapY);
    mMapSize = imageSize;
}

void RGBDInputRectifier::RectifyRGB(
    const cv::Mat &rawRGB,
    cv::Mat &rectifiedRGB)
{
    if(rawRGB.empty())
        throw std::invalid_argument("Cannot rectify an empty RGB image");
    if(!mEnabled)
    {
        rectifiedRGB = rawRGB;
        return;
    }

    EnsureMaps(rawRGB.size());
    cv::remap(
        rawRGB,rectifiedRGB,mMapX,mMapY,cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,cv::Scalar());
}

void RGBDInputRectifier::RectifyRGBD(
    const cv::Mat &rawRGB,
    const cv::Mat &rawDepth,
    cv::Mat &rectifiedRGB,
    cv::Mat &rectifiedDepth)
{
    if(rawRGB.empty() || rawDepth.empty())
        throw std::invalid_argument("Cannot rectify empty RGB-D input");
    if(rawRGB.size()!=rawDepth.size())
        throw std::invalid_argument(
            "RGB and registered depth must have the same size before rectification");
    if(!mEnabled)
    {
        rectifiedRGB = rawRGB;
        rectifiedDepth = rawDepth;
        return;
    }

    EnsureMaps(rawRGB.size());
    cv::remap(
        rawRGB,rectifiedRGB,mMapX,mMapY,cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,cv::Scalar());
    cv::remap(
        rawDepth,rectifiedDepth,mMapX,mMapY,cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,cv::Scalar(0));
}

} // namespace ORB_SLAM2
