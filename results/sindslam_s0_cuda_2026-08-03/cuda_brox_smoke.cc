#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/imgproc.hpp>

int main()
{
    std::cout << "OpenCV=" << CV_VERSION << '\n';
    const int device_count = cv::cuda::getCudaEnabledDeviceCount();
    std::cout << "cuda_device_count=" << device_count << '\n';
    if (device_count <= 0)
        return 2;

    cv::cuda::setDevice(0);
    cv::cuda::printShortCudaDeviceInfo(0);

    cv::Mat previous = cv::Mat::zeros(96, 128, CV_32FC1);
    cv::Mat current = cv::Mat::zeros(previous.size(), previous.type());
    cv::rectangle(previous, cv::Rect(32, 32, 24, 24), cv::Scalar(1.0f), -1);
    cv::rectangle(current, cv::Rect(35, 34, 24, 24), cv::Scalar(1.0f), -1);

    cv::cuda::GpuMat previous_gpu(previous);
    cv::cuda::GpuMat current_gpu(current);
    cv::cuda::GpuMat flow_gpu;
    const cv::Ptr<cv::cuda::BroxOpticalFlow> brox =
        cv::cuda::BroxOpticalFlow::create(0.197f, 50.0f, 0.8f, 10, 77, 10);
    brox->calc(previous_gpu, current_gpu, flow_gpu);

    cv::Mat flow;
    flow_gpu.download(flow);
    if (flow.type() != CV_32FC2 || flow.size() != previous.size())
        return 3;

    const cv::Scalar mean_flow = cv::mean(flow(cv::Rect(36, 35, 16, 16)));
    std::cout << "flow_type=" << flow.type()
              << " flow_size=" << flow.cols << 'x' << flow.rows
              << " mean_dx=" << mean_flow[0]
              << " mean_dy=" << mean_flow[1] << '\n';

    if (!std::isfinite(mean_flow[0]) || !std::isfinite(mean_flow[1]))
        return 4;

    return 0;
}
