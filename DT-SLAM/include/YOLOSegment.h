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
#include <condition_variable>
#include <atomic>
#include <vector>
#include <string>

namespace ORB_SLAM2
{

// 单个检测结果，用于Pangolin调试可视化
struct Detection
{
    cv::Rect box;
    float confidence;
};

struct FrameSubmitTiming
{
    double mutexWaitMs;
    double copyMs;
};

class YOLOSegment
{
public:
    YOLOSegment(const std::string &modelPath,
                float confThreshold = 0.5,
                float nmsThreshold = 0.45);
    ~YOLOSegment();

    void Start();
    void Stop();

    // Tracking线程调用：提交新帧（深拷贝，带帧序号）并读取最新结果。
    // 同步或异步策略由上层调用者决定。
    FrameSubmitTiming PushFrame(const cv::Mat &imRGB, int seq);
    bool WaitForMask(int seq, cv::Mat &mask, int timeoutMs = 30000);
    cv::Mat GetLatestMask();
    int GetMaskSeq();              // 返回mask对应帧序号（-1=暂无）
    std::vector<Detection> GetDetections();

    // 推理耗时统计，Stop()后有效
    float GetInferenceAvgMs() const;
    float GetInferenceMedianMs() const;
    float GetInferenceMinMs() const;
    float GetInferenceMaxMs() const;
    int   GetInferenceCount() const;
    int   GetProcessedFrames() const { return mProcessedFrames.load(); }

private:
    void Run();

    // 预处理：letterbox→blob
    cv::Mat Preprocess(const cv::Mat &imRGB, float &scale, int &padX, int &padY);
    // NMS筛选person检测框 + mask解码，返回mask和检测列表
    cv::Mat Postprocess(const cv::Mat &imRGB, std::vector<Ort::Value> &outputs,
                        float scale, int padX, int padY,
                        std::vector<Detection> &detections);

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
    std::condition_variable mConditionFrame;
    cv::Mat mPendingFrame;
    int mPendingSeq;
    bool mNewFrame;

    std::mutex mMutexResult;
    std::condition_variable mConditionResult;
    cv::Mat mLatestMask;
    int mLatestMaskSeq;
    std::vector<Detection> mLatestDetections;
    std::vector<float> mPreprocessTimes; // 每帧预处理耗时(ms)
    std::vector<float> mExecutionTimes;  // 每帧ONNX执行耗时(ms)
    std::vector<float> mPostprocessTimes;// 每帧后处理耗时(ms)
    std::vector<float> mInferenceTimes;  // 每帧端到端语义耗时(ms)
    std::atomic<int> mProcessedFrames;  // 实际完成的推理帧数（原子，无data race）

    std::thread mThread;
    std::atomic<bool> mRunning;
};

} // namespace ORB_SLAM2

#endif
