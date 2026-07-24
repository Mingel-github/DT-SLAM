/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<cmath>
#include<cstdlib>
#include<fstream>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<YOLOSegment.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void PrintTimingSummary(const string &name, const vector<double> &samples)
{
    if(samples.empty())
        return;

    vector<double> sorted = samples;
    sort(sorted.begin(),sorted.end());

    double sum = 0.0;
    for(double value : samples)
        sum += value;

    const size_t p95Index = static_cast<size_t>(
        ceil(0.95*static_cast<double>(sorted.size()))) - 1;

    cout << "[RGBD Timing] " << name << "(ms): mean=" << sum/samples.size()
         << " median=" << sorted[sorted.size()/2]
         << " p95=" << sorted[p95Index]
         << " min=" << sorted.front()
         << " max=" << sorted.back()
         << " n=" << sorted.size() << endl;
}

int main(int argc, char **argv)
{
    // DT-SLAM: 支持可选第5参数(ONNX模型路径)启用语义动态过滤
    if(argc != 5 && argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association [path_to_onnx_model]" << endl;
        cerr << "  ONNX model: YOLOv8n-seg ONNX,启用动态过滤; 不提供=纯几何baseline" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // DT-SLAM: 初始化语义线程
    ORB_SLAM2::YOLOSegment* pYOLO = nullptr;
    int nMaskReady = 0;
    vector<int> vMaskAges; // mask年龄统计（帧延时）
    if(argc == 6 && string(argv[5]).find(".onnx") != string::npos)
    {
        try
        {
            pYOLO = new ORB_SLAM2::YOLOSegment(argv[5], 0.5f, 0.45f);
        }
        catch(const std::exception& e)
        {
            cerr << "[DT-SLAM] Failed to initialize semantic inference: "
                 << e.what() << endl;
            return 1;
        }
        pYOLO->Start();
        cout << "[DT-SLAM] 语义线程已启动，模型: " << argv[5] << endl;

        // Precompute the first exact-frame mask while the SLAM system has not
        // started yet. The main loop reuses this result for sequence 0.
        cv::Mat imFirst = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[0], cv::IMREAD_UNCHANGED);
        if (!imFirst.empty())
        {
            pYOLO->PushFrame(imFirst, 0);
            cv::Mat firstMask;
            if(!pYOLO->WaitForMask(0,firstMask))
            {
                cerr << "[DT-SLAM] Failed to obtain the semantic mask for frame 0" << endl;
                pYOLO->Stop();
                delete pYOLO;
                return 1;
            }
            cout << "[DT-SLAM] 首帧mask就绪，开始跟踪" << endl;
        }
    }

    // Create SLAM system. Headless runs can disable Pangolin without changing
    // the tracking pipeline; the viewer remains enabled by default.
    const char* disableViewer = std::getenv("DT_SLAM_DISABLE_VIEWER");
    const bool useViewer = !(disableViewer && string(disableViewer)=="1");
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,useViewer);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    vector<double> vTrackingTimesMs;

    // End-to-end timing is intentionally collected without changing the
    // existing processing order or dataset pacing policy.
    vector<double> vImageIOTimes;
    vector<double> vGetMaskSeqTimes;
    vector<double> vFrameMutexWaitTimes;
    vector<double> vFrameCopyTimes;
    vector<double> vPushCopyTimes;
    vector<double> vMaskWaitTimes;
    vector<double> vSemanticBlockTimes;
    vector<double> vActiveTotalTimes;
    vector<double> vSleepTimes;
    vector<double> vFrameWallTimes;
    vector<double> vPacingErrorTimes;
    vector<double> vDeadlineOverruns;
    int nDeadlineMisses = 0;

    std::chrono::steady_clock::time_point firstLoopStart;
    std::chrono::steady_clock::time_point previousLoopStart;
    std::chrono::steady_clock::time_point lastLoopStart;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        const std::chrono::steady_clock::time_point tLoopStart =
            std::chrono::steady_clock::now();
        if(ni==0)
        {
            firstLoopStart = tLoopStart;
        }
        else
        {
            const double frameWallMs =
                std::chrono::duration<double,std::milli>(
                    tLoopStart-previousLoopStart).count();
            const double targetIntervalMs =
                (vTimestamps[ni]-vTimestamps[ni-1])*1000.0;
            vFrameWallTimes.push_back(frameWallMs);
            vPacingErrorTimes.push_back(frameWallMs-targetIntervalMs);
        }
        previousLoopStart = tLoopStart;
        lastLoopStart = tLoopStart;

        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        const std::chrono::steady_clock::time_point tImagesReady =
            std::chrono::steady_clock::now();
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        vImageIOTimes.push_back(
            std::chrono::duration<double,std::milli>(
                tImagesReady-tLoopStart).count());

        // Phase 0 semantic baseline: every RGB frame is paired with the mask
        // carrying the same sequence number. The worker remains separate, but
        // tracking never consumes a stale mask.
        cv::Mat mask;
        if(pYOLO)
        {
            const std::chrono::steady_clock::time_point tSemanticStart =
                std::chrono::steady_clock::now();
            bool pushedFrame = false;
            const int currentMaskSeq = pYOLO->GetMaskSeq();
            const std::chrono::steady_clock::time_point tGetMaskSeqDone =
                std::chrono::steady_clock::now();
            vGetMaskSeqTimes.push_back(
                std::chrono::duration<double,std::milli>(
                    tGetMaskSeqDone-tSemanticStart).count());
            if(currentMaskSeq!=ni)
            {
                const ORB_SLAM2::FrameSubmitTiming submitTiming =
                    pYOLO->PushFrame(imRGB, ni);
                vFrameMutexWaitTimes.push_back(submitTiming.mutexWaitMs);
                vFrameCopyTimes.push_back(submitTiming.copyMs);
                pushedFrame = true;
            }
            const std::chrono::steady_clock::time_point tPushDone =
                std::chrono::steady_clock::now();

            if(!pYOLO->WaitForMask(ni,mask))
            {
                cerr << "[DT-SLAM] Failed to obtain the semantic mask for frame " << ni << endl;
                pYOLO->Stop();
                delete pYOLO;
                SLAM.Shutdown();
                return 1;
            }

            const std::chrono::steady_clock::time_point tMaskReady =
                std::chrono::steady_clock::now();
            if(pushedFrame)
            {
                vPushCopyTimes.push_back(
                    std::chrono::duration<double,std::milli>(
                        tPushDone-tSemanticStart).count());
            }
            vMaskWaitTimes.push_back(
                std::chrono::duration<double,std::milli>(
                    tMaskReady-tPushDone).count());
            vSemanticBlockTimes.push_back(
                std::chrono::duration<double,std::milli>(
                    tMaskReady-tSemanticStart).count());

            SLAM.UpdateDetections(pYOLO->GetDetections());
            nMaskReady++;
            vMaskAges.push_back(ni-pYOLO->GetMaskSeq());
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,mask,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        vTrackingTimesMs.push_back(ttrack*1000.0);
        const double activeTotalMs =
            std::chrono::duration<double,std::milli>(
                t2-tLoopStart).count();
        vActiveTotalTimes.push_back(activeTotalMs);

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        // Pace the dataset using the complete active frame time, including
        // image I/O, exact-frame semantic inference and SLAM tracking.
        // vTimesTrack intentionally remains the SLAM-only statistic.
        const std::chrono::steady_clock::time_point tSleepStart =
            std::chrono::steady_clock::now();
        const double activeTotalSeconds = activeTotalMs/1000.0;
        if(activeTotalSeconds<T)
            usleep((T-activeTotalSeconds)*1e6);
        const std::chrono::steady_clock::time_point tSleepEnd =
            std::chrono::steady_clock::now();
        vSleepTimes.push_back(
            std::chrono::duration<double,std::milli>(
                tSleepEnd-tSleepStart).count());

        const double targetIntervalMs = T*1000.0;
        const double deadlineOverrunMs =
            std::max(0.0,activeTotalMs-targetIntervalMs);
        vDeadlineOverruns.push_back(deadlineOverrunMs);
        if(deadlineOverrunMs>0.0)
            nDeadlineMisses++;
    }
    const std::chrono::steady_clock::time_point sequenceEnd =
        std::chrono::steady_clock::now();

    // DT-SLAM: 停止语义线程 + 输出性能统计
    if(pYOLO)
    {
        pYOLO->Stop();
        cout << "[DT-SLAM] mask就绪: " << nMaskReady << "/" << nImages << endl;
        if (!vMaskAges.empty())
        {
            std::sort(vMaskAges.begin(), vMaskAges.end());
            int am = vMaskAges[vMaskAges.size()/2];
            int aMax = *std::max_element(vMaskAges.begin(), vMaskAges.end());
            cout << "[DT-SLAM] mask年龄(帧): median=" << am
                      << " max=" << aMax << " n=" << vMaskAges.size() << endl;
        }
        delete pYOLO;
    }

    cout << "[RGBD Timing] End-to-end frame timing (measurement only)" << endl;
    PrintTimingSummary("image_io",vImageIOTimes);
    PrintTimingSummary("get_mask_seq",vGetMaskSeqTimes);
    PrintTimingSummary("frame_mutex_wait",vFrameMutexWaitTimes);
    PrintTimingSummary("frame_copy",vFrameCopyTimes);
    PrintTimingSummary("push_copy",vPushCopyTimes);
    PrintTimingSummary("mask_wait",vMaskWaitTimes);
    PrintTimingSummary("semantic_block_all",vSemanticBlockTimes);
    if(vSemanticBlockTimes.size()>1)
    {
        const vector<double> steadySemanticBlock(
            vSemanticBlockTimes.begin()+1,vSemanticBlockTimes.end());
        PrintTimingSummary("semantic_block_steady",steadySemanticBlock);
    }
    PrintTimingSummary("tracking",vTrackingTimesMs);
    PrintTimingSummary("active_total",vActiveTotalTimes);
    PrintTimingSummary("sleep_actual",vSleepTimes);
    PrintTimingSummary("frame_wall",vFrameWallTimes);
    PrintTimingSummary("pacing_error",vPacingErrorTimes);
    PrintTimingSummary("deadline_overrun",vDeadlineOverruns);

    const double sequenceWallSeconds =
        std::chrono::duration<double>(sequenceEnd-firstLoopStart).count();
    cout << "[RGBD Timing] deadline_missed=" << nDeadlineMisses
         << "/" << nImages << endl;
    cout << "[RGBD Timing] sequence_wall(s)=" << sequenceWallSeconds << endl;
    if(nImages>1)
    {
        const double loopStartSpanSeconds =
            std::chrono::duration<double>(lastLoopStart-firstLoopStart).count();
        const double datasetSpanSeconds =
            vTimestamps.back()-vTimestamps.front();
        cout << "[RGBD Timing] actual_fps=" << (nImages-1)/loopStartSpanSeconds
             << " dataset_fps=" << (nImages-1)/datasetSpanSeconds << endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
