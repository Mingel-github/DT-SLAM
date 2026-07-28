#ifndef JI_GEOMETRY_BASELINE_H
#define JI_GEOMETRY_BASELINE_H

#include <opencv2/core/core.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ORB_SLAM2
{

struct JiDepthCluster
{
    int id = -1;
    std::size_t pixels = 0;
    cv::Vec3f centroid = cv::Vec3f(0.0f,0.0f,0.0f);
};

struct JiDepthClusteringStats
{
    std::size_t imagePixels = 0;
    std::size_t validDepthPixels = 0;
    int requestedClusters = 0;
    int producedClusters = 0;
    std::size_t smallestClusterPixels = 0;
    std::size_t largestClusterPixels = 0;
    double compactness = 0.0;
    double prepareMs = 0.0;
    double kmeansMs = 0.0;
    double labelMs = 0.0;
    double totalMs = 0.0;
};

struct JiDepthClusteringResult
{
    // Raw registered depth pixel domain. -1 denotes invalid/no depth.
    cv::Mat labelImage;
    std::vector<JiDepthCluster> clusters;
    JiDepthClusteringStats stats;
};

struct JiReprojectionObservation
{
    // rawPixel selects the raw registered depth cluster.
    cv::Point2f rawPixel;
    // observedPinholePixel is in the undistorted optimizer domain.
    cv::Point2f observedPinholePixel;
    cv::Point3f worldPoint;
    bool optimizerOutlier = false;
};

struct JiClusterReprojectionStats
{
    int clusterId = -1;
    std::size_t featureCount = 0;
    std::size_t matchedMapSupport = 0;
    std::size_t optimizerOutlierSupport = 0;
    std::size_t validReprojectionSupport = 0;
    std::size_t invalidProjectionCount = 0;
    double meanSquaredErrorPixels2 = 0.0;
    double meanErrorPixels = 0.0;
    double medianErrorPixels = 0.0;
    double p90ErrorPixels = 0.0;
    double maximumErrorPixels = 0.0;
    bool hasGeometryEvidence = false;
};

struct JiReprojectionFrameStats
{
    bool initialPoseAvailable = false;
    std::size_t featureCount = 0;
    std::size_t featuresAssignedToClusters = 0;
    std::size_t matchedObservations = 0;
    std::size_t optimizerOutlierObservations = 0;
    std::size_t matchesAssignedToClusters = 0;
    std::size_t validReprojections = 0;
    std::size_t clustersWithoutEvidence = 0;
    double featureAssignmentMs = 0.0;
    double reprojectionMs = 0.0;
    double aggregateMs = 0.0;
    double totalMs = 0.0;
};

class JiGeometryBaseline
{
public:
    JiGeometryBaseline();

    void SetCameraMatrix(const cv::Mat &cameraMatrix);
    void SetClusterCount(int clusterCount);
    void SetKMeansCriteria(int maxIterations, double epsilon);
    void SetKMeansAttempts(int attempts);
    void SetRandomSeed(std::uint64_t randomSeed);

    int ClusterCount() const;
    int MaxIterations() const;
    double Epsilon() const;
    int Attempts() const;
    std::uint64_t RandomSeed() const;

    bool ComputeDepthClusters(
        const cv::Mat &depthMeters,
        JiDepthClusteringResult &result) const;

    bool ComputeClusterReprojectionStats(
        const JiDepthClusteringResult &clustering,
        const std::vector<cv::Point2f> &rawFeaturePixels,
        const std::vector<JiReprojectionObservation> &observations,
        const cv::Mat &initialTcw,
        const cv::Mat &projectionCameraMatrix,
        std::vector<JiClusterReprojectionStats> &clusterStats,
        JiReprojectionFrameStats &frameStats) const;

private:
    cv::Mat mCameraMatrix;
    int mClusterCount;
    int mMaxIterations;
    double mEpsilon;
    int mAttempts;
    std::uint64_t mRandomSeed;
};

} // namespace ORB_SLAM2

#endif // JI_GEOMETRY_BASELINE_H
