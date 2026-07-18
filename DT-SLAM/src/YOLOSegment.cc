/**
 * DT-SLAM: YOLOv8-seg ONNX 异步语义线程实现
 * 推理引擎: ONNX Runtime C++ API
 * 预处理/后处理: OpenCV
 */

#include "YOLOSegment.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <cmath>

namespace ORB_SLAM2
{

YOLOSegment::YOLOSegment(const std::string &modelPath,
                         float confThreshold, float nmsThreshold)
    : mEnv(ORT_LOGGING_LEVEL_WARNING, "YOLOSegment")
    , mConfThreshold(confThreshold)
    , mNmsThreshold(nmsThreshold)
    , mNewFrame(false)
    , mRunning(false)
    , mInputW(640)
    , mInputH(640)
{
    // 配置CPU推理
    mSessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    mSessionOptions.SetIntraOpNumThreads(2);

    // 加载ONNX模型
    mSession.reset(new Ort::Session(mEnv, modelPath.c_str(), mSessionOptions));

    // 分配输入输出名称内存（ORT需要C字符串指针，类成员持有std::string保证生命周期）
    Ort::AllocatorWithDefaultOptions allocator;
    size_t numInputs = mSession->GetInputCount();
    for (size_t i = 0; i < numInputs; i++)
    {
        char* name = mSession->GetInputNameAllocated(i, allocator).release();
        mInputNames.push_back(name);
        auto typeInfo = mSession->GetInputTypeInfo(i);
        auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
        mInputShape = tensorInfo.GetShape();
    }
    size_t numOutputs = mSession->GetOutputCount();
    for (size_t i = 0; i < numOutputs; i++)
    {
        char* name = mSession->GetOutputNameAllocated(i, allocator).release();
        mOutputNames.push_back(name);
    }

    // 允许动态batch: 输入shape[0]可以是任意值
    if (!mInputShape.empty())
        mInputShape[0] = 1;

    std::cout << "[YOLO] ONNX Runtime模型加载: " << modelPath
              << " (inputs:" << numInputs << " outputs:" << numOutputs << ")" << std::endl;
}

YOLOSegment::~YOLOSegment()
{
    Stop();
    // 释放输入输出名称
    for (auto& name : mInputNames)
        free(const_cast<char*>(name));
    for (auto& name : mOutputNames)
        free(const_cast<char*>(name));
}

void YOLOSegment::Start()
{
    mRunning = true;
    mThread = std::thread(&YOLOSegment::Run, this);
    std::cout << "[YOLO] 推理线程已启动" << std::endl;
}

void YOLOSegment::Stop()
{
    mRunning = false;
    if (mThread.joinable())
        mThread.join();
    std::cout << "[YOLO] 推理线程已停止" << std::endl;
}

void YOLOSegment::PushFrame(const cv::Mat &imRGB)
{
    std::lock_guard<std::mutex> lock(mMutexFrame);
    imRGB.copyTo(mPendingFrame);
    mNewFrame = true;
}

cv::Mat YOLOSegment::GetLatestMask()
{
    std::lock_guard<std::mutex> lock(mMutexMask);
    if (mLatestMask.empty())
        return cv::Mat();
    cv::Mat result;
    mLatestMask.copyTo(result);
    return result;
}

void YOLOSegment::Run()
{
    while (mRunning)
    {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mMutexFrame);
            if (!mNewFrame || mPendingFrame.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            mNewFrame = false;
            mPendingFrame.copyTo(frame);
        }

        try
        {
            cv::Mat mask = Segment(frame);
            std::lock_guard<std::mutex> lock(mMutexMask);
            mask.copyTo(mLatestMask);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[YOLO] 推理异常: " << e.what() << std::endl;
        }
    }
}

cv::Mat YOLOSegment::Segment(const cv::Mat &imRGB)
{
    int imgW = imRGB.cols;
    int imgH = imRGB.rows;
    cv::Mat mask = cv::Mat::zeros(imgH, imgW, CV_8U);

    // ---- 预处理：letterbox + blob ----
    float scale = std::min((float)mInputW / imgW, (float)mInputH / imgH);
    int newW = (int)(imgW * scale);
    int newH = (int)(imgH * scale);
    int padX = (mInputW - newW) / 2;
    int padY = (mInputH - newH) / 2;

    cv::Mat resized, padded;
    cv::resize(imRGB, resized, cv::Size(newW, newH));
    cv::copyMakeBorder(resized, padded, padY, mInputH - newH - padY,
                       padX, mInputW - newW - padX, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    cv::Mat blob;
    cv::dnn::blobFromImage(padded, blob, 1.0 / 255.0, cv::Size(mInputW, mInputH),
                           cv::Scalar(), true, false); // swapRB: BGR→RGB, 无crop

    // ---- 前向推理 ----
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<int64_t> inputShape = {1, 3, mInputH, mInputW};
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memInfo, (float*)blob.ptr<float>(), blob.total(), inputShape.data(), inputShape.size());

    auto outputs = mSession->Run(Ort::RunOptions{nullptr},
                                  mInputNames.data(), &inputTensor, 1,
                                  mOutputNames.data(), mOutputNames.size());

    if (outputs.size() < 2)
        return mask;

    // output0: [1, 116, 8400] — 检测头
    //   col[0:4]=bbox(cx,cy,w,h), col[4:84]=80类score, col[84:116]=32 mask coefficients
    // output1: [1, 32, 160, 160] — 原型mask
    float* detData = outputs[0].GetTensorMutableData<float>();
    float* protoData = outputs[1].GetTensorMutableData<float>();

    auto detInfo = outputs[0].GetTensorTypeAndShapeInfo();
    auto protoInfo = outputs[1].GetTensorTypeAndShapeInfo();
    auto detShape = detInfo.GetShape();      // [1, 116, 8400]
    auto protoShape = protoInfo.GetShape();  // [1, 32, 160, 160]

    const int numAnchors = (int)detShape[2];
    const int detChannels = (int)detShape[1];  // 4 + 80 + 32 = 116
    const int numClasses = 80;
    const int maskCoeffDim = 32;
    const int protoH = (int)protoShape[2];
    const int protoW = (int)protoShape[3];

    // ---- 后处理 ----
    // 转置 [116, 8400] → [8400, 116]
    cv::Mat detMat(numAnchors, detChannels, CV_32F);
    for (int j = 0; j < detChannels; j++)
        for (int i = 0; i < numAnchors; i++)
            detMat.at<float>(i, j) = detData[j * numAnchors + i];

    cv::Mat bboxes    = detMat.colRange(0, 4);
    cv::Mat scores    = detMat.colRange(4, 4 + numClasses);
    cv::Mat maskCoeffs = detMat.colRange(4 + numClasses, detChannels);

    // 筛选person类(class=0)
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> keptIndices;

    for (int i = 0; i < numAnchors; i++)
    {
        float personScore = scores.at<float>(i, 0);
        if (personScore < mConfThreshold)
            continue;

        float cx = bboxes.at<float>(i, 0);
        float cy = bboxes.at<float>(i, 1);
        float w  = bboxes.at<float>(i, 2);
        float h  = bboxes.at<float>(i, 3);

        float x1 = (cx - w / 2 - padX) / scale;
        float y1 = (cy - h / 2 - padY) / scale;
        float x2 = (cx + w / 2 - padX) / scale;
        float y2 = (cy + h / 2 - padY) / scale;

        x1 = std::max(0.0f, std::min(x1, (float)imgW));
        y1 = std::max(0.0f, std::min(y1, (float)imgH));
        x2 = std::max(0.0f, std::min(x2, (float)imgW));
        y2 = std::max(0.0f, std::min(y2, (float)imgH));

        int bw = (int)(x2 - x1);
        int bh = (int)(y2 - y1);
        if (bw <= 0 || bh <= 0)
            continue;

        boxes.push_back(cv::Rect((int)x1, (int)y1, bw, bh));
        confidences.push_back(personScore);
        keptIndices.push_back(i);
    }

    // NMS
    std::vector<int> nmsIndices;
    cv::dnn::NMSBoxes(boxes, confidences, mConfThreshold, mNmsThreshold, nmsIndices);

    if (nmsIndices.empty())
        return mask;

    // ---- Mask解码 ----
    // 原型mask: [1, 32, 160, 160] → [32, 25600]
    cv::Mat protosFlat(protoH * protoW, maskCoeffDim, CV_32F);
    for (int c = 0; c < maskCoeffDim; c++)
    {
        for (int y = 0; y < protoH; y++)
            for (int x = 0; x < protoW; x++)
                protosFlat.at<float>(y * protoW + x, c) = protoData[c * protoH * protoW + y * protoW + x];
    }

    for (int idx : nmsIndices)
    {
        int origIdx = keptIndices[idx];
        cv::Rect box = boxes[idx];

        cv::Mat coeffs = maskCoeffs.row(origIdx);      // 1×32
        cv::Mat maskFlat = coeffs * protosFlat.t();     // 1×25600

        cv::Mat sigmoidMask(1, protoH * protoW, CV_32F);
        for (int i = 0; i < protoH * protoW; i++)
            sigmoidMask.at<float>(0, i) = 1.0f / (1.0f + std::exp(-maskFlat.at<float>(0, i)));

        cv::Mat maskProto = sigmoidMask.reshape(1, protoH);
        cv::Mat maskBin;
        cv::threshold(maskProto, maskBin, 0.5, 255, cv::THRESH_BINARY);
        maskBin.convertTo(maskBin, CV_8U);

        cv::Mat maskResized;
        cv::resize(maskBin, maskResized, cv::Size(box.width, box.height));
        cv::Mat roi = mask(box);
        cv::bitwise_or(maskResized, roi, roi);
    }

    return mask;
}

} // namespace ORB_SLAM2
