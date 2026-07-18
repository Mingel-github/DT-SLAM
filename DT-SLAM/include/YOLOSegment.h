/**
 * DT-SLAM: YOLOv8-seg ONNX 异步语义线程
 * 独立线程运行实例分割，Tracking通过GetLatestMask()取上一帧结果（异步一帧滞后）
 * 推理引擎: ONNX Runtime C++ API（替换OpenCV DNN，兼容YOLOv8 ONNX算子）
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
    // modelPath: ONNX模型路径
    // confThreshold: 检测置信度阈值
    // nmsThreshold: NMS IoU阈值
    YOLOSegment(const std::string &modelPath,
                float confThreshold = 0.5,
                float nmsThreshold = 0.45);
    ~YOLOSegment();

    // 启动推理线程
    void Start();
    // 停止推理线程（析构时自动调用）
    void Stop();

    // Tracking线程调用：提交新帧（非阻塞，深拷贝）
    void PushFrame(const cv::Mat &imRGB);
    // Tracking线程调用：取最新完成的动态mask。空矩阵=暂无结果
    cv::Mat GetLatestMask();

private:
    // 推理线程主循环
    void Run();
    // 同步推理：预处理→前向→后处理→返回person类合并mask
    cv::Mat Segment(const cv::Mat &imRGB);

    // ONNX Runtime
    Ort::Env mEnv;
    Ort::SessionOptions mSessionOptions;
    std::unique_ptr<Ort::Session> mSession;
    std::vector<const char*> mInputNames;
    std::vector<const char*> mOutputNames;
    std::vector<int64_t> mInputShape;
    int mInputW, mInputH;

    // 阈值参数
    float mConfThreshold;
    float mNmsThreshold;

    // 输入帧buffer（mutex保护）
    std::mutex mMutexFrame;
    cv::Mat mPendingFrame;
    bool mNewFrame;

    // 输出mask buffer（mutex保护）
    std::mutex mMutexMask;
    cv::Mat mLatestMask;

    // 线程控制
    std::thread mThread;
    std::atomic<bool> mRunning;
};

} // namespace ORB_SLAM2

#endif // YOLOSEGMENT_H
