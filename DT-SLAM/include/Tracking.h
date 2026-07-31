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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "GeometricDynamicDetector.h"
#include "JiGeometryBaseline.h"

#include <mutex>
#include <deque>
#include <set>
#include <string>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

struct GeometrySemanticProxyStats
{
    std::size_t semanticPixels = 0;
    std::size_t validComparisonPixels = 0;
    std::size_t semanticValidPixels = 0;
    std::size_t positiveSeedPixels = 0;
    std::size_t positiveInsideSemanticPixels = 0;
    std::size_t positiveOutsideSemanticPixels = 0;
    double semanticValidCoverage = 0.0;
    double proxyPrecision = 0.0;
    double conditionalRecall = 0.0;
    double staticBackgroundFpr = 0.0;
};

struct GeometryFeatureShadowStats
{
    int radiusPixels = 0;
    std::size_t featureCount = 0;
    std::size_t semanticFeatureCount = 0;
    std::size_t eligibleFeatureCount = 0;
    std::size_t semanticEligibleFeatureCount = 0;
    std::size_t candidateFeatureCount = 0;
    std::size_t candidateInsideSemanticFeatureCount = 0;
    std::size_t candidateOutsideSemanticFeatureCount = 0;
    double eligibleCoverage = 0.0;
    double semanticEligibleCoverage = 0.0;
    double proxyPrecision = 0.0;
    double conditionalRecall = 0.0;
    double proxyBackgroundRate = 0.0;
};

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const cv::Mat &mask, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
    void SetGroundTruthPoseForGeometry(const cv::Mat &TcwGroundTruth);
    void SaveGeometryPoseDiagnostics();

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    // Semantic dynamic observations are kept separate from optimizer outliers.
    void UpdateDynamicFeaturesFromMask(Frame &frame, const cv::Mat &mask);
    int RemoveDynamicAssociations(Frame &frame);
    void RunGeometryShadow();
    void RunMultiReferenceGeometryShadow();
    void RunSparseEgoFlowShadow();
    int ApplySparseFlowTrackingFilter();
    void RecordSparseFlowAssociationSnapshot(
        const std::string &stage,
        const int trackingInliers = -1,
        const std::vector<unsigned char> &countedTrackingInliers =
            std::vector<unsigned char>());
    void UpdateSparseEgoFlowReference();
    void UpdateMultiReferenceGeometryHistory(
        const cv::Mat &referenceDepth);
    void SaveGeometryDebugImages(const GeometricWarpResult &result);
    void RunJiGeometryShadow();
    void SaveJiGeometryDebugImages(
        const JiDepthClusteringResult &result);
    void CaptureJiInitialTrackingSnapshot();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    // G0 geometry shadow state. It is read-only with respect to SLAM.
    cv::Mat mCurrentDepthMeters;
    cv::Mat mCurrentGeometryDebugImage;
    cv::Mat mCurrentGroundTruthTcw;
    cv::Mat mGeometryK;
    GeometricDynamicDetector mGeometricDetector;
    GeometricDynamicDetector mGeometricGroundTruthDetector;
    bool mbGeometryShadowEnabled;
    bool mbGeometrySingleReferenceShadowEnabled;
    bool mbGeometryDebugSaveEnabled;
    bool mbGeometryUsesDedicatedCameraModel;
    int mnGeometryLogEveryN;
    int mnGeometryDebugEveryN;
    std::string mGeometryDebugOutputDir;
    std::string mGeometryPoseDiagnosticCsvPath;
    std::string mGeometrySemanticProxyCsvPath;
    std::string mGeometryFeatureShadowCsvPath;
    long unsigned int mnGeometryComputedFrames;

    // G2-1/G2-2R multi-reference evidence and reference selection.
    // Shadow-only.
    bool mbGeometryMultiReferenceShadowEnabled;
    int mnGeometryMultiReferenceMaxReferences;
    int mnGeometryMultiReferenceHistorySize;
    std::string mGeometryMultiReferenceSelectionPolicy;
    std::string mGeometryMultiReferenceSamplingPolicy;
    int mnGeometryMultiReferenceGridStride;
    int mnGeometryMultiReferencePyramidScale;
    bool mbGeometryMultiReferenceDenseAuditEnabled;
    std::string mGeometryMultiReferenceCsvPath;
    std::string mGeometryMultiReferenceFeatureCsvPath;
    std::set<long unsigned int> mGeometryMultiReferenceFeatureFrameFilter;
    std::string mGeometryReferenceSelectionCsvPath;
    std::string mGeometryMultiReferenceDebugOutputDir;
    long unsigned int mnGeometryMultiReferenceComputedFrames;
    std::deque<GeometricReferenceFrame> mqGeometryKeyframeReferences;

    struct GeometryMultiReferenceHistogramRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string samplingPolicy;
        int referenceCount;
        int comparisonCount;
        int positiveCount;
        std::size_t pixelCount;
        std::size_t semanticPixelCount;
        GeometricMultiReferenceStats frameStats;
    };
    std::vector<GeometryMultiReferenceHistogramRecord>
        mvGeometryMultiReferenceHistogram;

    struct GeometryMultiReferenceFeatureRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string samplingPolicy;
        std::size_t featureIndex;
        int imageU;
        int imageV;
        int octave;
        bool hasMapPoint;
        bool currentFrameOutlierFlag;
        bool semanticNonzero;
        int nativeScale;
        int nativeU;
        int nativeV;
        unsigned char comparisonCount;
        unsigned char positiveCount;
        unsigned char negativeCount;
        unsigned char consistentCount;
    };
    std::vector<GeometryMultiReferenceFeatureRecord>
        mvGeometryMultiReferenceFeatureDiagnostics;

    // G2-4F1 adjacent-successful-frame sparse ego-flow residual. The raw
    // measurement remains diagnostic; default-off G1-F1 may consume it
    // through the separate association-filter state below.
    bool mbGeometrySparseEgoFlowShadowEnabled;
    std::string mGeometrySparseFlowCsvPath;
    std::set<long unsigned int> mGeometrySparseFlowFrameFilter;
    long unsigned int mnGeometrySparseFlowComputedFrames;

    // G1-F0B raw association snapshots. Counterfactual-only: the online
    // path records existing state and never scores or removes observations.
    bool mbGeometrySparseFlowCounterfactualShadowEnabled;
    std::string mGeometryAssociationSnapshotCsvPath;

    // G1-F1 default-off removal of high-residual MapPoint associations before
    // the existing TrackLocalMap pose optimization. It does not write
    // Frame::mvbDynamic and does not veto mapping.
    bool mbGeometrySparseFlowTrackingFilterEnabled;
    float mfGeometrySparseFlowTrackingFilterQ;
    float mfGeometrySparseFlowTrackingFilterMaximumAssociationFraction;
    int mnGeometrySparseFlowTrackingFilterMinimumAssociations;
    std::size_t mnGeometrySparseFlowTrackingFilterMinimumScaleSupport;
    std::string mGeometrySparseFlowTrackingFilterCsvPath;
    GeometricSparseFlowFilterResult mCurrentSparseFlowFilterResult;
    std::vector<unsigned char>
        mvbCurrentSparseFlowRemovedAssociations;
    bool mbCurrentSparseFlowTrackingSafeguardsPassed;
    std::string mCurrentSparseFlowTrackingSafeguardState;

    struct GeometrySparseFlowTrackingFilterRecord
    {
        long unsigned int frameId;
        double timestamp;
        float qThreshold;
        bool scaleValid;
        float frameScalePixels;
        std::size_t scaleSupport;
        std::size_t qualityEligibleFeatures;
        std::size_t candidateFeatures;
        int baselineAssociations;
        int candidateAssociations;
        int removedAssociations;
        int remainingAssociations;
        double candidateAssociationFraction;
        bool withinRelocalizationWindow;
        bool applied;
        std::string state;
    };
    std::vector<GeometrySparseFlowTrackingFilterRecord>
        mvGeometrySparseFlowTrackingFilterDiagnostics;

    // G1-M0 default-off, read-only audit of q10 candidate admission into
    // RGB-D initialization and Tracking-side KeyFrame MapPoint creation.
    bool mbGeometrySparseFlowMappingCounterfactualEnabled;
    std::string mGeometrySparseFlowMappingCounterfactualCsvPath;
    struct GeometrySparseFlowMappingAdmissionRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string stage;
        float qThreshold;
        bool scaleValid;
        bool candidateVectorValid;
        std::string candidateState;
        std::size_t featureCount;
        std::size_t candidateFeatures;
        std::size_t candidateAssociationsBeforeMapping;
        std::size_t candidateTrackingRemovals;
        std::size_t validDepthFeatures;
        std::size_t candidateValidDepthFeatures;
        std::size_t depthAdmissionFeatures;
        std::size_t candidateDepthAdmissionFeatures;
        std::size_t createdMapPoints;
        std::size_t candidateCreatedMapPoints;
        std::size_t recreatedAfterTrackingRemoval;
    };
    std::vector<GeometrySparseFlowMappingAdmissionRecord>
        mvGeometrySparseFlowMappingAdmissionDiagnostics;

    // G1-M1 default-off MapPoint admission protection. Once the existing
    // G1-F1 safeguards and the mapping-specific fail-open limits pass, q10
    // candidates are fused into Frame::mvbDynamic before KeyFrame creation.
    bool mbGeometrySparseFlowMappingFilterEnabled;
    float mfGeometrySparseFlowMappingFilterMaximumFeatureFraction;
    float mfGeometrySparseFlowMappingFilterMaximumDepthFraction;
    std::size_t
        mnGeometrySparseFlowMappingFilterMinimumRemainingDepthFeatures;
    std::string mGeometrySparseFlowMappingFilterCsvPath;
    struct GeometrySparseFlowMappingFilterRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string stage;
        float qThreshold;
        bool scaleValid;
        bool candidateVectorValid;
        bool trackingSafeguardsPassed;
        std::string trackingSafeguardState;
        std::size_t featureCount;
        std::size_t availableFeatures;
        std::size_t candidateFeatures;
        double candidateFeatureFraction;
        std::size_t validDepthFeatures;
        std::size_t candidateValidDepthFeatures;
        double candidateDepthFraction;
        float maximumFeatureFraction;
        float maximumDepthFraction;
        std::size_t minimumRemainingDepthFeatures;
        std::size_t remainingValidDepthFeatures;
        std::size_t candidateAssociationsBeforeVeto;
        std::size_t candidateTrackingRemovals;
        std::size_t newDynamicFlags;
        std::size_t removedAssociations;
        std::size_t vetoedDepthFeatures;
        std::size_t createdMapPoints;
        std::size_t candidateCreatedMapPoints;
        bool applied;
        std::string state;
    };
    std::vector<GeometrySparseFlowMappingFilterRecord>
        mvGeometrySparseFlowMappingFilterDiagnostics;

    // Read-only final-map quality audit for q10 candidate MapPoints created
    // by either the G1-M0 baseline or the G1-M1 fail-open path.
    bool mbGeometrySparseFlowMapQualityAuditEnabled;
    std::string mGeometrySparseFlowMapQualityPrefix;
    struct GeometrySparseFlowCandidateMapPointRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::size_t featureIndex;
        float pixelX;
        float pixelY;
        float depthMeters;
        std::string mode;
        std::string mappingState;
        long unsigned int originalMapPointId;
        MapPoint *originalMapPoint;
    };
    std::vector<GeometrySparseFlowCandidateMapPointRecord>
        mvGeometrySparseFlowCandidateMapPoints;

    struct GeometryAssociationSnapshotRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string stage;
        std::size_t featureIndex;
        bool hasMapPoint;
        bool mapPointBad;
        int mapPointObservations;
        bool currentFrameOutlier;
        bool semanticNonzero;
        bool onlyTracking;
        bool withinRelocalizationWindow;
        bool countedTrackingInlier;
        int trackingInliers;
    };
    std::vector<GeometryAssociationSnapshotRecord>
        mvGeometryAssociationSnapshotDiagnostics;

    // G2-4F3 adjacent-frame local 3-D edge-length consistency.
    // Shadow-only: no rigidity threshold, motion class, or SLAM mutation.
    bool mbGeometryLocalRigidityShadowEnabled;
    std::string mGeometryLocalRigidityCsvPath;
    std::set<long unsigned int> mGeometryLocalRigidityFrameFilter;
    long unsigned int mnGeometryLocalRigidityComputedFrames;

    struct SparseFlowReference
    {
        cv::Mat gray;
        cv::Mat depthMeters;
        cv::Mat TcwFinal;
        cv::Mat TcwGroundTruth;
        long unsigned int frameId = 0;
        double timestamp = 0.0;
        bool valid = false;
    };
    SparseFlowReference mSparseFlowReference;

    struct GeometrySparseFlowFeatureRecord
    {
        long unsigned int frameId;
        double timestamp;
        long unsigned int referenceFrameId;
        double referenceTimestamp;
        int octave;
        bool hasMapPoint;
        bool semanticNonzero;
        GeometricSparseFlowSample sample;
    };
    std::vector<GeometrySparseFlowFeatureRecord>
        mvGeometrySparseFlowFeatureDiagnostics;

    struct GeometrySparseFlowFrameRecord
    {
        long unsigned int frameId;
        double timestamp;
        long unsigned int referenceFrameId;
        double referenceTimestamp;
        bool referenceAvailable;
        bool domainValid;
        double recordMs;
        double activeTotalMs;
        GeometricSparseFlowStats stats;
    };
    std::vector<GeometrySparseFlowFrameRecord>
        mvGeometrySparseFlowFrameDiagnostics;

    struct GeometryLocalRigidityNodeRecord
    {
        long unsigned int frameId;
        double timestamp;
        long unsigned int referenceFrameId;
        double referenceTimestamp;
        int octave;
        bool hasMapPoint;
        bool semanticNonzero;
        GeometricRigidityNodeSample sample;
    };
    std::vector<GeometryLocalRigidityNodeRecord>
        mvGeometryLocalRigidityNodeDiagnostics;

    struct GeometryLocalRigidityEdgeRecord
    {
        long unsigned int frameId;
        double timestamp;
        long unsigned int referenceFrameId;
        double referenceTimestamp;
        bool hasMapPointA;
        bool hasMapPointB;
        bool semanticNonzeroA;
        bool semanticNonzeroB;
        GeometricRigidityEdgeSample sample;
    };
    std::vector<GeometryLocalRigidityEdgeRecord>
        mvGeometryLocalRigidityEdgeDiagnostics;

    struct GeometryLocalRigidityFrameRecord
    {
        long unsigned int frameId;
        double timestamp;
        long unsigned int referenceFrameId;
        double referenceTimestamp;
        bool referenceAvailable;
        bool domainValid;
        GeometricRigidityStats stats;
    };
    std::vector<GeometryLocalRigidityFrameRecord>
        mvGeometryLocalRigidityFrameDiagnostics;

    // G2-3R1 fixed-region evidence distributions. Shadow-only.
    bool mbGeometryRegionEvidenceShadowEnabled;
    bool mbGeometryRegionRiskDiagnosticsEnabled;
    bool mbGeometryLowResolutionRegionShadowEnabled;
    float mGeometryRegionRelativeThreshold;
    float mGeometryRegionAbsoluteThresholdMeters;
    std::string mGeometryRegionEvidenceCsvPath;
    long unsigned int mnGeometryRegionEvidenceComputedFrames;

    struct GeometryRegionEvidenceRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string samplingPolicy;
        GeometricRegionPartitionStats partitionStats;
        GeometricRegionEvidenceAggregationStats aggregationStats;
        GeometricRegionEvidenceStats region;
    };
    std::vector<GeometryRegionEvidenceRecord>
        mvGeometryRegionEvidenceDiagnostics;

    struct GeometryReferenceSelectionRecord
    {
        long unsigned int frameId;
        double timestamp;
        std::string policy;
        std::string samplingPolicy;
        int requestedReferenceCount;
        GeometricReferenceSelectionStats stats;
        bool evidenceComputed;
        std::vector<long unsigned int> selectedFrameIds;
        std::vector<int> selectedCovisibilityWeights;
        std::vector<long int> selectedFrameAges;
        std::vector<GeometricPerReferenceStats> perReference;
        bool denseAuditComputed;
        std::size_t sampledComparisonPixels;
        std::size_t denseComparisonOnSampledPixels;
        std::size_t sampledPositivePresencePixels;
        std::size_t densePositiveOnSampledPixels;
        std::size_t bothPositivePixels;
        std::size_t positivePresenceAgreementPixels;
        std::size_t exactVoteAgreementPixels;
    };
    std::vector<GeometryReferenceSelectionRecord>
        mvGeometryReferenceSelectionDiagnostics;

    struct GeometryPoseDiagnosticRecord
    {
        long unsigned int frameId;
        long unsigned int referenceFrameId;
        double timestamp;
        double referenceTimestamp;
        GeometricWarpStats slam;
        GeometricWarpStats groundTruth;
    };
    std::vector<GeometryPoseDiagnosticRecord> mvGeometryPoseDiagnostics;

    struct GeometrySemanticProxyRecord
    {
        long unsigned int frameId;
        long unsigned int referenceFrameId;
        double timestamp;
        bool hasGroundTruth;
        GeometrySemanticProxyStats slam;
        GeometrySemanticProxyStats groundTruth;
    };
    std::vector<GeometrySemanticProxyRecord> mvGeometrySemanticProxyDiagnostics;

    struct GeometryFeatureShadowRecord
    {
        long unsigned int frameId;
        long unsigned int referenceFrameId;
        double timestamp;
        bool groundTruthPose;
        GeometryFeatureShadowStats stats;
    };
    std::vector<GeometryFeatureShadowRecord> mvGeometryFeatureShadowDiagnostics;

    // GJ-1 Ji 2021 depth-clustering baseline. Shadow-only: labels and
    // diagnostics must not affect tracking, mapping, or dynamic flags.
    JiGeometryBaseline mJiGeometryBaseline;
    bool mbJiGeometryShadowEnabled;
    bool mbJiGeometryReprojectionStatsEnabled;
    bool mbJiGeometryDebugSaveEnabled;
    bool mbJiGeometryDebugRawLabelsOnly;
    int mnJiGeometryLogEveryN;
    int mnJiGeometryDebugEveryN;
    std::string mJiGeometryDebugOutputDir;
    std::string mJiGeometryClusterCsvPath;
    std::string mJiGeometryReprojectionCsvPath;
    long unsigned int mnJiGeometryComputedFrames;
    cv::Mat mJiInitialTcw;
    std::vector<JiReprojectionObservation> mvJiInitialObservations;

    struct JiGeometryClusterRecord
    {
        long unsigned int frameId;
        double timestamp;
        JiDepthClusteringStats frameStats;
        JiDepthCluster cluster;
    };
    std::vector<JiGeometryClusterRecord> mvJiGeometryClusterDiagnostics;

    struct JiGeometryReprojectionRecord
    {
        long unsigned int frameId;
        double timestamp;
        JiDepthCluster depthCluster;
        JiReprojectionFrameStats frameStats;
        JiClusterReprojectionStats clusterStats;
    };
    std::vector<JiGeometryReprojectionRecord>
        mvJiGeometryReprojectionDiagnostics;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
