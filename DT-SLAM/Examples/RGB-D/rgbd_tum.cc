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
#include<fstream>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<YOLOSegment.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

bool WaitForMask(ORB_SLAM2::YOLOSegment* pYOLO, const int seq, cv::Mat &mask,
                 const int timeoutMs = 30000)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while(std::chrono::steady_clock::now()<deadline)
    {
        const int maskSeq = pYOLO->GetMaskSeq();
        if(maskSeq==seq)
        {
            mask = pYOLO->GetLatestMask();
            return !mask.empty();
        }
        if(maskSeq>seq)
            return false;
        usleep(2000);
    }
    return false;
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
        pYOLO = new ORB_SLAM2::YOLOSegment(argv[5], 0.5f, 0.45f);
        pYOLO->Start();
        cout << "[DT-SLAM] 语义线程已启动，模型: " << argv[5] << endl;

        // Precompute the first exact-frame mask while the SLAM system has not
        // started yet. The main loop reuses this result for sequence 0.
        cv::Mat imFirst = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[0], cv::IMREAD_UNCHANGED);
        if (!imFirst.empty())
        {
            pYOLO->PushFrame(imFirst, 0);
            cv::Mat firstMask;
            if(!WaitForMask(pYOLO,0,firstMask))
            {
                cerr << "[DT-SLAM] Failed to obtain the semantic mask for frame 0" << endl;
                pYOLO->Stop();
                delete pYOLO;
                return 1;
            }
            cout << "[DT-SLAM] 首帧mask就绪，开始跟踪" << endl;
        }
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true); // 开启可视化

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        // Phase 0 semantic baseline: every RGB frame is paired with the mask
        // carrying the same sequence number. The worker remains separate, but
        // tracking never consumes a stale mask.
        cv::Mat mask;
        if(pYOLO)
        {
            if(pYOLO->GetMaskSeq()!=ni)
                pYOLO->PushFrame(imRGB, ni);

            if(!WaitForMask(pYOLO,ni,mask))
            {
                cerr << "[DT-SLAM] Failed to obtain the semantic mask for frame " << ni << endl;
                pYOLO->Stop();
                delete pYOLO;
                SLAM.Shutdown();
                return 1;
            }

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

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

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
