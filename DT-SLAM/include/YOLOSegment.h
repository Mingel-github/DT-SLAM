/**
 * DT-SLAM: YOLOv8-seg ONNX Runtime 异步语义线程
 * 推理引擎: ONNX Runtime C++ API
 * 预处理/后处理: OpenCV
 */

#ifndef YOLOSEGMENT_H
#define YOLOSEGMENT_H

#include <opencv2/core/core.hpp>
#include <onnxruntime_cxx_api.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>

namespace ORB_SLAM2
{

class YOLOSegment
{
public:
    YOLOSegment(const std::string &modelPath,
                float confThreshold = 0.5,
                float nmsThreshold = 0.45);
    ~YOLOSegment();

    void Start();
    void Stop();

    // Tracking线程调用：提交新帧（非阻塞深拷贝），取最新mask（异步一帧滞后）
    void PushFrame(const cv::Mat &imRGB);
    cv::Mat GetLatestMask();

private:
    void Run();

    // 预处理：letterbox→blob
    cv::Mat Preprocess(const cv::Mat &imRGB, float &scale, int &padX, int &padY);
    // NMS筛选person检测框 + mask系数解码
    cv::Mat Postprocess(const cv::Mat &imRGB, std::vector<Ort::Value> &outputs,
                        float scale, int padX, int padY);

    // ONNX Runtime
    Ort::Env mEnv;
    Ort::SessionOptions mSessionOpts;
    std::unique_ptr<Ort::Session> mSession;
    Ort::AllocatorWithDefaultOptions mAlloc;
    std::vector<std::string> mInputNames;
    std::vector<std::string> mOutputNames;
    std::vector<int64_t> mInputShape;

    float mConfThreshold;
    float mNmsThreshold;

    // 线程安全buffer
    std::mutex mMutexFrame;
    cv::Mat mPendingFrame;
    bool mNewFrame;

    std::mutex mMutexMask;
    cv::Mat mLatestMask;

    std::thread mThread;
    std::atomic<bool> mRunning;
};

} // namespace ORB_SLAM2

#endif
