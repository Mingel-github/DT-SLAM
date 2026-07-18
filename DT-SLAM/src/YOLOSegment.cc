/**
 * DT-SLAM: YOLOv8-seg ONNX 异步语义线程实现
 * 基于ultralytics YOLOv8-seg ONNX导出格式
 * 参考: NGD-SLAM异步架构, ultralytics公开输出协议
 */

#include "YOLOSegment.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace ORB_SLAM2
{

YOLOSegment::YOLOSegment(const std::string &modelPath,
                         float confThreshold, float nmsThreshold)
    : mConfThreshold(confThreshold)
    , mNmsThreshold(nmsThreshold)
    , mNewFrame(false)
    , mRunning(false)
    , mInputW(640)
    , mInputH(640)
{
    // 加载ONNX模型
    mNet = cv::dnn::readNetFromONNX(modelPath);
    // 尝试CUDA后端，不可用则回退CPU
    mNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    mNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
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
    std::cout << "[YOLO] 推理线程已停止" << std::endl;
}

void YOLOSegment::PushFrame(const cv::Mat &imRGB)
{
    // 非阻塞：只保留最新一帧
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
        // 等待新帧
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mMutexFrame);
            if (!mNewFrame || mPendingFrame.empty())
            {
                // 没有新帧，短暂休眠后重试
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            mNewFrame = false;
            mPendingFrame.copyTo(frame);
        }

        // 同步推理
        cv::Mat mask = Segment(frame);

        // 更新最新mask
        {
            std::lock_guard<std::mutex> lock(mMutexMask);
            mask.copyTo(mLatestMask);
        }
    }
}

cv::Mat YOLOSegment::Segment(const cv::Mat &imRGB)
{
    int imgW = imRGB.cols;
    int imgH = imRGB.rows;
    cv::Mat mask = cv::Mat::zeros(imgH, imgW, CV_8U);

    // ---- 预处理 ----
    cv::Mat blob;
    // letterbox: 等比缩放+padding，保持网络输入640×640
    float scale = std::min((float)mInputW / imgW, (float)mInputH / imgH);
    int newW = (int)(imgW * scale);
    int newH = (int)(imgH * scale);
    int padX = (mInputW - newW) / 2;
    int padY = (mInputH - newH) / 2;

    cv::Mat resized, padded;
    cv::resize(imRGB, resized, cv::Size(newW, newH));
    cv::copyMakeBorder(resized, padded, padY, mInputH - newH - padY,
                       padX, mInputW - newW - padX, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::dnn::blobFromImage(padded, blob, 1.0 / 255.0, cv::Size(mInputW, mInputH),
                           cv::Scalar(), true, false); // swapRB=true → BGR→RGB, crop=false

    // ---- 前向推理 ----
    mNet.setInput(blob);
    std::vector<cv::Mat> outputs;
    mNet.forward(outputs, mNet.getUnconnectedOutLayersNames());

    if (outputs.size() < 2)
        return mask;

    // output0: [1, 116, 8400] — 检测头
    //   [0:4]=bbox(cx,cy,w,h), [4:84]=80类score, [84:116]=32 mask coefficients
    // output1: [1, 32, 160, 160] — 原型mask
    cv::Mat detOut = outputs[0];  // [1, 116, 8400]
    cv::Mat protoOut = outputs[1]; // [1, 32, 160, 160]

    // ---- 后处理 ----
    const int numAnchors = detOut.size[2];
    const int numClasses = 80;
    const int maskCoeffDim = 32;
    const int protoH = protoOut.size[2];
    const int protoW = protoOut.size[3];

    // 转置为 [8400, 116] — detOut是4D [1, 116, 8400]，用ptr访问
    cv::Mat detMat(numAnchors, 4 + numClasses + maskCoeffDim, CV_32F);
    int detChannels = detOut.size[1];
    for (int j = 0; j < detChannels; j++)
    {
        float* detData = detOut.ptr<float>(0, j);
        for (int i = 0; i < numAnchors; i++)
            detMat.at<float>(i, j) = detData[i];
    }

    // 提取各分量
    cv::Mat bboxes = detMat.colRange(0, 4);          // [N, 4]  cx,cy,w,h
    cv::Mat scores = detMat.colRange(4, 4 + numClasses); // [N, 80]
    cv::Mat maskCoeffs = detMat.colRange(4 + numClasses, detMat.cols); // [N, 32]

    // 筛选person类(class=0)，转换bbox到原图坐标
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> keptIndices;

    for (int i = 0; i < numAnchors; i++)
    {
        float personScore = scores.at<float>(i, 0); // class 0 = person
        if (personScore < mConfThreshold)
            continue;

        // cx,cy,w,h → 网络输入坐标 (640×640)
        float cx = bboxes.at<float>(i, 0);
        float cy = bboxes.at<float>(i, 1);
        float w  = bboxes.at<float>(i, 2);
        float h  = bboxes.at<float>(i, 3);

        // 还原letterbox → 原图坐标
        float x1 = (cx - w / 2 - padX) / scale;
        float y1 = (cy - h / 2 - padY) / scale;
        float x2 = (cx + w / 2 - padX) / scale;
        float y2 = (cy + h / 2 - padY) / scale;

        // 裁剪到图像范围
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
        float* protoData = protoOut.ptr<float>(0, c); // 第0样本, 第c通道
        for (int y = 0; y < protoH; y++)
            for (int x = 0; x < protoW; x++)
                protosFlat.at<float>(y * protoW + x, c) = protoData[y * protoW + x];
    }

    for (int idx : nmsIndices)
    {
        int origIdx = keptIndices[idx];
        cv::Rect box = boxes[idx];

        // mask_coefficients (1×32) @ protos (32×25600) → (1×25600)
        cv::Mat coeffs = maskCoeffs.row(origIdx);  // 1×32
        cv::Mat maskFlat = coeffs * protosFlat.t(); // 1×25600

        // sigmoid
        cv::Mat sigmoidMask(1, protoH * protoW, CV_32F);
        for (int i = 0; i < protoH * protoW; i++)
            sigmoidMask.at<float>(0, i) = 1.0f / (1.0f + std::exp(-maskFlat.at<float>(0, i)));

        // reshape → [160, 160] → 二值化
        cv::Mat maskProto = sigmoidMask.reshape(1, protoH); // 160×160
        cv::Mat maskBin;
        cv::threshold(maskProto, maskBin, 0.5, 255, cv::THRESH_BINARY);
        maskBin.convertTo(maskBin, CV_8U);

        // resize到检测框大小，填充到原图mask上
        cv::Mat maskResized;
        cv::resize(maskBin, maskResized, cv::Size(box.width, box.height));

        // ROI范围内OR合并（多个人各自独立）
        cv::Mat roi = mask(box);
        cv::bitwise_or(maskResized, roi, roi);
    }

    return mask;
}

} // namespace ORB_SLAM2
