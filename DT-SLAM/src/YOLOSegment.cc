/**
 * DT-SLAM: YOLOv8-seg ONNX Runtime 异步语义线程实现
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
{
    mSessionOpts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    mSessionOpts.SetIntraOpNumThreads(2);

    mSession.reset(new Ort::Session(mEnv, modelPath.c_str(), mSessionOpts));

    // 从模型读取输入尺寸，不再硬编码
    for (size_t i = 0; i < mSession->GetInputCount(); i++)
    {
        auto name = mSession->GetInputNameAllocated(i, mAlloc);
        mInputNames.push_back(name.get());
        auto shape = mSession->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        mInputShape = shape; // [1, 3, H, W]
    }
    if (mInputShape.size() >= 4)
        std::cout << "[YOLO] 输入尺寸: " << mInputShape[2] << "x" << mInputShape[3] << std::endl;

    for (size_t i = 0; i < mSession->GetOutputCount(); i++)
    {
        auto name = mSession->GetOutputNameAllocated(i, mAlloc);
        mOutputNames.push_back(name.get());
    }

    std::cout << "[YOLO] ONNX模型加载: " << modelPath << std::endl;
}

YOLOSegment::~YOLOSegment()
{
    Stop();
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
}

void YOLOSegment::PushFrame(const cv::Mat &imRGB)
{
    std::lock_guard<std::mutex> lock(mMutexFrame);
    imRGB.copyTo(mPendingFrame);
    mNewFrame = true;
}

cv::Mat YOLOSegment::GetLatestMask()
{
    std::lock_guard<std::mutex> lock(mMutexResult);
    if (mLatestMask.empty())
        return cv::Mat();
    return mLatestMask.clone();
}

std::vector<Detection> YOLOSegment::GetDetections()
{
    std::lock_guard<std::mutex> lock(mMutexResult);
    return mLatestDetections;
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
            frame = mPendingFrame.clone();
        }

        try
        {
            float scale; int padX, padY;
            cv::Mat blob = Preprocess(frame, scale, padX, padY);

            // 前向推理
            Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
                memInfo, (float*)blob.ptr<float>(), blob.total(),
                mInputShape.data(), mInputShape.size());

            std::vector<const char*> inNames, outNames;
            for (auto& n : mInputNames)  inNames.push_back(n.c_str());
            for (auto& n : mOutputNames) outNames.push_back(n.c_str());

            auto outputs = mSession->Run(Ort::RunOptions{nullptr},
                                          inNames.data(), &inputTensor, 1,
                                          outNames.data(), outNames.size());

            std::vector<Detection> detections;
            cv::Mat mask = Postprocess(frame, outputs, scale, padX, padY, detections);

            std::lock_guard<std::mutex> lock(mMutexResult);
            mask.copyTo(mLatestMask);
            mLatestDetections = detections;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[YOLO] 推理异常: " << e.what() << std::endl;
        }
    }
}

// ---- Preprocess: letterbox缩放+padding+blob ----
cv::Mat YOLOSegment::Preprocess(const cv::Mat &imRGB, float &scale, int &padX, int &padY)
{
    int imgW = imRGB.cols, imgH = imRGB.rows;
    int netW = (int)mInputShape[3], netH = (int)mInputShape[2];

    scale = std::min((float)netW / imgW, (float)netH / imgH);
    int newW = (int)(imgW * scale), newH = (int)(imgH * scale);
    padX = (netW - newW) / 2;
    padY = (netH - newH) / 2;

    cv::Mat resized, padded;
    cv::resize(imRGB, resized, cv::Size(newW, newH));
    cv::copyMakeBorder(resized, padded, padY, netH - newH - padY,
                       padX, netW - newW - padX, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    cv::Mat blob;
    cv::dnn::blobFromImage(padded, blob, 1.0/255.0, cv::Size(netW, netH),
                           cv::Scalar(), true, false);
    return blob;
}

// ---- Postprocess: NMS筛选 + mask解码 ----
cv::Mat YOLOSegment::Postprocess(const cv::Mat &imRGB, std::vector<Ort::Value> &outputs,
                                  float scale, int padX, int padY,
                                  std::vector<Detection> &detections)
{
    int imgW = imRGB.cols, imgH = imRGB.rows;
    cv::Mat mask = cv::Mat::zeros(imgH, imgW, CV_8U);

    if (outputs.size() < 2)
        return mask;

    float* detData  = outputs[0].GetTensorMutableData<float>();
    float* protoData = outputs[1].GetTensorMutableData<float>();
    auto detShape   = outputs[0].GetTensorTypeAndShapeInfo().GetShape();   // [1,116,8400]
    auto protoShape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();   // [1,32,160,160]

    const int nAnchors = (int)detShape[2];
    const int detCh   = (int)detShape[1];  // 4+80+32=116
    const int nCls    = 80;
    const int maskDim = 32;
    const int protoH  = (int)protoShape[2];
    const int protoW  = (int)protoShape[3];

    // 转置 [nCh, nAnchors] → [nAnchors, nCh]
    cv::Mat detMat(nAnchors, detCh, CV_32F);
    for (int j = 0; j < detCh; j++)
        for (int i = 0; i < nAnchors; i++)
            detMat.at<float>(i, j) = detData[j * nAnchors + i];

    // 筛选person类，转换坐标。同时保存输入空间bbox用于proto裁剪
    std::vector<cv::Rect> boxes;       // 原图坐标
    std::vector<cv::Rect> inputBoxes;  // 输入空间坐标(640×640, 用于proto裁剪)
    std::vector<float> confs;
    std::vector<int> keptIdx;

    for (int i = 0; i < nAnchors; i++)
    {
        float score = detMat.at<float>(i, 4); // class 0 = person
        if (score < mConfThreshold) continue;

        float cx = detMat.at<float>(i, 0), cy = detMat.at<float>(i, 1);
        float w  = detMat.at<float>(i, 2), h  = detMat.at<float>(i, 3);

        // 输入空间坐标(含letterbox padding)
        float ix1 = cx - w/2, iy1 = cy - h/2;
        float ix2 = cx + w/2, iy2 = cy + h/2;

        // 原图坐标(去掉padding+缩放)
        float x1 = (ix1 - padX) / scale, y1 = (iy1 - padY) / scale;
        float x2 = (ix2 - padX) / scale, y2 = (iy2 - padY) / scale;
        x1 = std::max(0.f, std::min(x1, (float)imgW));
        y1 = std::max(0.f, std::min(y1, (float)imgH));
        x2 = std::max(0.f, std::min(x2, (float)imgW));
        y2 = std::max(0.f, std::min(y2, (float)imgH));

        int bw = (int)(x2-x1), bh = (int)(y2-y1);
        if (bw <= 0 || bh <= 0) continue;

        boxes.push_back(cv::Rect((int)x1, (int)y1, bw, bh));
        inputBoxes.push_back(cv::Rect((int)ix1, (int)iy1, (int)(ix2-ix1), (int)(iy2-iy1)));
        confs.push_back(score);
        keptIdx.push_back(i);
    }

    std::vector<int> nmsIdx;
    cv::dnn::NMSBoxes(boxes, confs, mConfThreshold, mNmsThreshold, nmsIdx);

    // 保存NMS后的检测结果，用于Pangolin可视化
    detections.clear();
    for (int idx : nmsIdx)
        detections.push_back({boxes[idx], confs[idx]});

    if (nmsIdx.empty()) return mask;

    // ---- Mask解码：mask_coeffs @ protos → sigmoid → 二值化 ----
    cv::Mat protosFlat(protoH*protoW, maskDim, CV_32F);
    for (int c = 0; c < maskDim; c++)
        for (int y = 0; y < protoH; y++)
            for (int x = 0; x < protoW; x++)
                protosFlat.at<float>(y*protoW+x, c) = protoData[c*protoH*protoW + y*protoW + x];

    for (int idx : nmsIdx)
    {
        int orig = keptIdx[idx];
        cv::Rect box = boxes[idx];
        cv::Rect inBox = inputBoxes[idx]; // 输入空间bbox，用于proto裁剪
        cv::Mat coeffs = detMat.row(orig).colRange(4+nCls, detCh); // 1×32

        cv::Mat logits = coeffs * protosFlat.t(); // 1×25600

        cv::Mat tmp;
        cv::exp(-logits, tmp);
        cv::Mat prob = 1.0f / (1.0f + tmp);

        cv::Mat maskProto = prob.reshape(1, protoH); // 160×160, 覆盖全图(640×640)

        // 将输入空间bbox映射到proto坐标系(160×160)，裁剪后再resize
        float ps = (float)protoW / (float)mInputShape[3]; // proto缩放比 = 160/640 = 0.25
        int ppx = std::max(0, (int)(inBox.x * ps));
        int ppy = std::max(0, (int)(inBox.y * ps));
        int ppw = std::min(protoW - ppx, std::max(1, (int)(inBox.width * ps)));
        int pph = std::min(protoH - ppy, std::max(1, (int)(inBox.height * ps)));
        cv::Mat cropped = maskProto(cv::Range(ppy, ppy+pph), cv::Range(ppx, ppx+ppw));

        cv::Mat maskBin;
        cv::threshold(cropped, maskBin, 0.5, 255, cv::THRESH_BINARY);
        maskBin.convertTo(maskBin, CV_8U);

        cv::Mat maskResized;
        cv::resize(maskBin, maskResized, cv::Size(box.width, box.height));
        cv::bitwise_or(maskResized, mask(box), mask(box));
    }

    return mask;
}

} // namespace ORB_SLAM2
