#include "JiGeometryBaseline.h"

#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

void Require(const bool condition, const char *message)
{
    if(!condition)
        throw std::runtime_error(message);
}

}

int main()
{
    try
    {
        cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_32F);
        cameraMatrix.at<float>(0,0) = 100.0f;
        cameraMatrix.at<float>(1,1) = 100.0f;
        cameraMatrix.at<float>(0,2) = 2.5f;
        cameraMatrix.at<float>(1,2) = 2.5f;

        cv::Mat depth(6,6,CV_32F,cv::Scalar(1.0f));
        depth.colRange(3,6).setTo(3.0f);
        depth.at<float>(0,0) = 0.0f;

        ORB_SLAM2::JiGeometryBaseline baseline;
        baseline.SetCameraMatrix(cameraMatrix);
        baseline.SetClusterCount(2);
        baseline.SetKMeansCriteria(20,1e-3);
        baseline.SetKMeansAttempts(1);
        baseline.SetRandomSeed(2021);

        ORB_SLAM2::JiDepthClusteringResult first;
        ORB_SLAM2::JiDepthClusteringResult second;
        Require(
            baseline.ComputeDepthClusters(depth,first),
            "first clustering failed");
        Require(
            baseline.ComputeDepthClusters(depth,second),
            "second clustering failed");
        Require(first.labelImage.type()==CV_16SC1,"label type mismatch");
        Require(first.labelImage.at<short>(0,0)==-1,"invalid depth was labeled");
        Require(
            cv::countNonZero(first.labelImage!=second.labelImage)==0,
            "fixed seed did not produce deterministic labels");
        Require(first.clusters.size()==2,"cluster count mismatch");
        Require(
            first.stats.validDepthPixels==35,
            "valid depth count mismatch");

        const int nearCluster = first.labelImage.at<short>(2,1);
        const int farCluster = first.labelImage.at<short>(2,4);
        Require(nearCluster!=farCluster,"near and far surfaces were merged");
        for(int v=0; v<depth.rows; ++v)
        {
            for(int u=0; u<depth.cols; ++u)
            {
                if(v==0 && u==0)
                    continue;
                const int expected = u<3 ? nearCluster : farCluster;
                Require(
                    first.labelImage.at<short>(v,u)==expected,
                    "synthetic plane was split incorrectly");
            }
        }
        Require(
            std::fabs(
                first.clusters[nearCluster].centroid[2]-1.0f)<1e-4f,
            "near centroid depth mismatch");
        Require(
            std::fabs(
                first.clusters[farCluster].centroid[2]-3.0f)<1e-4f,
            "far centroid depth mismatch");

        std::vector<cv::Point2f> rawFeaturePixels;
        rawFeaturePixels.push_back(cv::Point2f(1.0f,2.0f));
        rawFeaturePixels.push_back(cv::Point2f(4.0f,2.0f));
        rawFeaturePixels.push_back(cv::Point2f(0.0f,0.0f));

        ORB_SLAM2::JiReprojectionObservation observation;
        observation.rawPixel = cv::Point2f(1.0f,2.0f);
        observation.observedPinholePixel = cv::Point2f(1.0f,2.0f);
        observation.worldPoint = cv::Point3f(-0.005f,-0.005f,1.0f);
        observation.optimizerOutlier = true;
        std::vector<ORB_SLAM2::JiReprojectionObservation> observations;
        observations.push_back(observation);

        cv::Mat identityPose = cv::Mat::eye(4,4,CV_32F);
        std::vector<ORB_SLAM2::JiClusterReprojectionStats>
            reprojectionStats;
        ORB_SLAM2::JiReprojectionFrameStats reprojectionFrameStats;
        Require(
            baseline.ComputeClusterReprojectionStats(
                first,rawFeaturePixels,observations,identityPose,
                cameraMatrix,reprojectionStats,reprojectionFrameStats),
            "reprojection statistics failed");
        Require(
            reprojectionFrameStats.initialPoseAvailable,
            "initial pose availability was not recorded");
        Require(
            reprojectionFrameStats.featureCount==3 &&
            reprojectionFrameStats.featuresAssignedToClusters==2,
            "raw feature-to-cluster assignment mismatch");
        Require(
            reprojectionFrameStats.validReprojections==1,
            "valid reprojection count mismatch");
        Require(
            reprojectionStats[nearCluster].featureCount==1 &&
            reprojectionStats[nearCluster].matchedMapSupport==1 &&
            reprojectionStats[nearCluster].optimizerOutlierSupport==1 &&
            reprojectionStats[nearCluster].validReprojectionSupport==1,
            "near cluster support mismatch");
        Require(
            std::fabs(
                reprojectionStats[nearCluster].
                    meanSquaredErrorPixels2-1.0)<1e-6 &&
            std::fabs(
                reprojectionStats[nearCluster].
                    meanErrorPixels-1.0)<1e-6,
            "known one-pixel reprojection error mismatch");
        Require(
            reprojectionStats[nearCluster].hasGeometryEvidence,
            "supported cluster was marked unknown");
        Require(
            reprojectionStats[farCluster].featureCount==1 &&
            reprojectionStats[farCluster].matchedMapSupport==0 &&
            !reprojectionStats[farCluster].hasGeometryEvidence,
            "unsupported cluster was not kept unknown");

        Require(
            reprojectionFrameStats.optimizerOutlierObservations==1,
            "optimizer outlier support count mismatch");

        std::cout << "[Ji GJ-1/GJ-2 Test] PASS" << std::endl;
        return 0;
    }
    catch(const std::exception &exception)
    {
        std::cerr << "[Ji GJ-1/GJ-2 Test] FAIL: "
                  << exception.what() << std::endl;
        return 1;
    }
}
