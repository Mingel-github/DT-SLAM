#include "SInStyleRAGRegionMerger.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <vector>

namespace ORB_SLAM2
{
namespace
{

double ElapsedMilliseconds(
    const std::chrono::steady_clock::time_point &start,
    const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration_cast<
        std::chrono::duration<double,std::milli> >(end-start).count();
}

bool IsDepthValid(const float depth, const float maximumDepth)
{
    return std::isfinite(depth) && depth>0.0f && depth<maximumDepth;
}

double Median(std::vector<int> values)
{
    if(values.empty())
        return 0.0;
    const std::size_t middle = values.size()/2;
    std::nth_element(values.begin(),values.begin()+middle,values.end());
    if(values.size()%2!=0)
        return values[middle];
    const int lower = *std::max_element(values.begin(),values.begin()+middle);
    return 0.5*static_cast<double>(lower+values[middle]);
}

struct RegionAttributes
{
    int sourceLabel = 0;
    int parentInitialLabel = 0;
    int fixedRank = -1;
    std::size_t area = 0;
    double depthSum = 0.0;
    double centerDepth = 0.0;
    double score = 0.0;
    std::vector<double> histogramCounts;
    cv::Mat coreMask;
};

struct Group
{
    bool active = true;
    int fixedRank = -1;
    std::vector<int> members;
    std::set<int> parentInitialLabels;
    std::size_t area = 0;
    double depthSum = 0.0;
    std::vector<double> histogramCounts;
    cv::Mat coreMask;
    cv::Mat dilatedMask;
};

struct PairEvidence
{
    bool candidate = false;
    bool spatialAdjacent = false;
    bool sharedFakeEdge = false;
    bool depthRejected = false;
    int overlapPixels = 0;
    int fakeBoundaryPixels = 0;
    double histogramSimilarity = 0.0;
    double totalScore = 0.0;
};

bool Disjoint(const std::set<int> &first, const std::set<int> &second)
{
    std::set<int>::const_iterator a = first.begin();
    std::set<int>::const_iterator b = second.begin();
    while(a!=first.end() && b!=second.end())
    {
        if(*a==*b)
            return false;
        if(*a<*b)
            ++a;
        else
            ++b;
    }
    return true;
}

double HistogramSimilarity(
    const std::vector<double> &firstCounts,
    const std::vector<double> &secondCounts)
{
    if(firstCounts.empty() || firstCounts.size()!=secondCounts.size())
        return 0.0;
    const double firstTotal = std::accumulate(
        firstCounts.begin(),firstCounts.end(),0.0);
    const double secondTotal = std::accumulate(
        secondCounts.begin(),secondCounts.end(),0.0);
    if(firstTotal<=0.0 || secondTotal<=0.0)
        return 0.0;

    const std::size_t bins = firstCounts.size();
    const double mean = 1.0/static_cast<double>(bins);
    double numerator = 0.0;
    double firstVariance = 0.0;
    double secondVariance = 0.0;
    double bhattacharyyaCoefficient = 0.0;
    double intersectionNumerator = 0.0;
    double intersectionDenominator = 0.0;
    double absoluteDifference = 0.0;
    for(std::size_t bin=0; bin<bins; ++bin)
    {
        const double first = firstCounts[bin]/firstTotal;
        const double second = secondCounts[bin]/secondTotal;
        const double firstCentered = first-mean;
        const double secondCentered = second-mean;
        numerator += firstCentered*secondCentered;
        firstVariance += firstCentered*firstCentered;
        secondVariance += secondCentered*secondCentered;
        bhattacharyyaCoefficient += std::sqrt(std::max(0.0,first*second));
        intersectionNumerator += std::min(first,second);
        intersectionDenominator += std::max(first,second);
        absoluteDifference += std::abs(first-second);
    }

    double correlation = 0.0;
    const double denominator = std::sqrt(firstVariance*secondVariance);
    if(denominator>std::numeric_limits<double>::epsilon())
        correlation = numerator/denominator;
    else if(absoluteDifference<=std::numeric_limits<double>::epsilon())
        correlation = 1.0;

    bhattacharyyaCoefficient = std::max(
        0.0,std::min(1.0,bhattacharyyaCoefficient));
    const double bhattacharyyaDistance = std::sqrt(
        std::max(0.0,1.0-bhattacharyyaCoefficient));
    const double normalizedIntersection =
        intersectionDenominator>0.0 ?
        intersectionNumerator/intersectionDenominator : 0.0;
    return correlation+(1.0-bhattacharyyaDistance)+
           normalizedIntersection;
}

void RecomputeDilation(Group &group, const cv::Mat &element, const int radius)
{
    if(radius>0)
        cv::dilate(group.coreMask,group.dilatedMask,element);
    else
        group.dilatedMask = group.coreMask.clone();
}

int SharedFakeBoundary(
    const Group &first,
    const Group &second,
    const std::vector<int> &componentFakeBoundary,
    const int componentCount)
{
    int count = 0;
    for(std::size_t i=0; i<first.members.size(); ++i)
    {
        const int firstMember = first.members[i];
        for(std::size_t j=0; j<second.members.size(); ++j)
        {
            count += componentFakeBoundary[
                static_cast<std::size_t>(firstMember)*componentCount+
                second.members[j]];
        }
    }
    return count;
}

double RankWeight(
    const Group &first,
    const Group &second,
    const int regionCount,
    const SInStyleRAGMergeConfig &config)
{
    // Paper profile: pair weight is determined by the better (minimum)
    // fixed score rank. The public source's smaller-area-label heuristic is
    // deliberately not mixed into this profile.
    const int pairRank = std::min(first.fixedRank,second.fixedRank);

    const int largeBoundary = static_cast<int>(
        std::floor(config.largeFraction*regionCount));
    const int smallBoundary = static_cast<int>(
        std::floor(config.smallFraction*regionCount));
    if(pairRank<largeBoundary)
        return config.largeRegionWeight;
    if(pairRank>smallBoundary)
        return config.smallRegionWeight;
    return config.middleRegionWeight;
}

PairEvidence EvaluatePair(
    const Group &first,
    const Group &second,
    const std::vector<int> &componentFakeBoundary,
    const int componentCount,
    const SInStyleRAGMergeConfig &config)
{
    PairEvidence evidence;
    if(!first.active || !second.active ||
       !Disjoint(first.parentInitialLabels,second.parentInitialLabels))
    {
        return evidence;
    }

    evidence.fakeBoundaryPixels = SharedFakeBoundary(
        first,second,componentFakeBoundary,componentCount);
    evidence.sharedFakeEdge = evidence.fakeBoundaryPixels>0;
    if(!evidence.sharedFakeEdge)
        return evidence;

    cv::Mat overlap;
    cv::bitwise_and(first.dilatedMask,second.dilatedMask,overlap);
    evidence.overlapPixels = cv::countNonZero(overlap);
    const double adjacencyThreshold = 0.4*std::min(
        static_cast<double>(config.adjacencyThresholdPixels),
        static_cast<double>(std::min(first.area,second.area)));
    evidence.spatialAdjacent = evidence.overlapPixels>adjacencyThreshold;
    if(!evidence.spatialAdjacent)
        return evidence;

    evidence.histogramSimilarity = HistogramSimilarity(
        first.histogramCounts,second.histogramCounts);
    evidence.depthRejected =
        evidence.histogramSimilarity<config.depthRejectThreshold;
    if(evidence.depthRejected)
        return evidence;

    evidence.totalScore =
        (config.fakeEdgeWeight*evidence.fakeBoundaryPixels+
         evidence.histogramSimilarity)*
        RankWeight(first,second,componentCount,config);
    evidence.candidate = true;
    return evidence;
}

void MergeGroup(
    Group &target,
    Group &donor,
    const cv::Mat &element,
    const int dilationRadius)
{
    cv::bitwise_or(target.coreMask,donor.coreMask,target.coreMask);
    target.area += donor.area;
    target.depthSum += donor.depthSum;
    for(std::size_t bin=0; bin<target.histogramCounts.size(); ++bin)
        target.histogramCounts[bin] += donor.histogramCounts[bin];
    target.members.insert(
        target.members.end(),donor.members.begin(),donor.members.end());
    target.parentInitialLabels.insert(
        donor.parentInitialLabels.begin(),donor.parentInitialLabels.end());
    target.fixedRank = std::min(target.fixedRank,donor.fixedRank);
    RecomputeDilation(target,element,dilationRadius);
    donor.active = false;
}

} // namespace

SInStyleRAGRegionMerger::SInStyleRAGRegionMerger()
{
}

void SInStyleRAGRegionMerger::Configure(
    const SInStyleRAGMergeConfig &config)
{
    if(config.enabled)
    {
        const bool finite =
            std::isfinite(config.maximumDepthMeters) &&
            std::isfinite(config.adjacencyThresholdPixels) &&
            std::isfinite(config.areaDepthScoreWeight) &&
            std::isfinite(config.fakeEdgeWeight) &&
            std::isfinite(config.largeRegionWeight) &&
            std::isfinite(config.middleRegionWeight) &&
            std::isfinite(config.smallRegionWeight) &&
            std::isfinite(config.mergeThreshold) &&
            std::isfinite(config.depthRejectThreshold) &&
            std::isfinite(config.highMiddleFraction) &&
            std::isfinite(config.largeFraction) &&
            std::isfinite(config.smallFraction);
        if(!finite || config.maximumDepthMeters<=0.0f ||
           config.adjacencyDilationRadius<0 || config.histogramBins<=1 ||
           config.adjacencyThresholdPixels<0.0f ||
           config.areaDepthScoreWeight<0.0f || config.fakeEdgeWeight<0.0f ||
           config.largeRegionWeight<0.0f ||
           config.middleRegionWeight<0.0f ||
           config.smallRegionWeight<0.0f || config.mergeThreshold<0.0f ||
           config.depthRejectThreshold<0.0f ||
           config.highMiddleFraction<=0.0f ||
           config.highMiddleFraction>1.0f ||
           config.largeFraction<0.0f ||
           config.largeFraction>config.smallFraction ||
           config.smallFraction>1.0f)
        {
            throw std::invalid_argument(
                "SIn gradient-only RAG configuration is invalid");
        }
    }
    mConfig = config;
}

SInStyleRAGMergeResult SInStyleRAGRegionMerger::Compute(
    const cv::Mat &depthMeters,
    const cv::Mat &initialLabels,
    const cv::Mat &splitCoreLabels,
    const cv::Mat &realEdgeMask) const
{
    SInStyleRAGMergeResult result;
    result.stats.enabled = mConfig.enabled;
    if(!mConfig.enabled)
        return result;
    if(depthMeters.empty() || depthMeters.type()!=CV_32FC1 ||
       initialLabels.empty() || initialLabels.type()!=CV_32SC1 ||
       splitCoreLabels.empty() || splitCoreLabels.type()!=CV_32SC1 ||
       realEdgeMask.empty() || realEdgeMask.type()!=CV_8UC1 ||
       depthMeters.size()!=initialLabels.size() ||
       depthMeters.size()!=splitCoreLabels.size() ||
       depthMeters.size()!=realEdgeMask.size())
    {
        throw std::invalid_argument(
            "SIn RAG requires aligned metric depth, labels, and real edge");
    }

    const std::chrono::steady_clock::time_point totalStart =
        std::chrono::steady_clock::now();
    result.stats.imagePixels = depthMeters.total();
    result.mergedLabels = cv::Mat(
        splitCoreLabels.size(),CV_32SC1,cv::Scalar(-1));
    for(int row=0; row<splitCoreLabels.rows; ++row)
    {
        const int *split = splitCoreLabels.ptr<int>(row);
        int *output = result.mergedLabels.ptr<int>(row);
        for(int col=0; col<splitCoreLabels.cols; ++col)
        {
            if(split[col]==0)
                output[col] = 0;
        }
    }

    const std::chrono::steady_clock::time_point attributeStart =
        std::chrono::steady_clock::now();
    int maximumSourceLabel = 0;
    for(int row=0; row<splitCoreLabels.rows; ++row)
    {
        const int *labels = splitCoreLabels.ptr<int>(row);
        for(int col=0; col<splitCoreLabels.cols; ++col)
            maximumSourceLabel = std::max(maximumSourceLabel,labels[col]);
    }
    std::vector<int> labelToRegion(
        static_cast<std::size_t>(maximumSourceLabel)+1,-1);
    std::vector<RegionAttributes> regions;
    for(int row=0; row<splitCoreLabels.rows; ++row)
    {
        const int *split = splitCoreLabels.ptr<int>(row);
        const int *initial = initialLabels.ptr<int>(row);
        const float *depth = depthMeters.ptr<float>(row);
        for(int col=0; col<splitCoreLabels.cols; ++col)
        {
            const int sourceLabel = split[col];
            if(sourceLabel<=0)
                continue;
            if(initial[col]<=0 ||
               !IsDepthValid(depth[col],mConfig.maximumDepthMeters))
            {
                throw std::invalid_argument(
                    "SIn RAG split core escapes valid initial-depth domain");
            }
            int regionIndex = labelToRegion[sourceLabel];
            if(regionIndex<0)
            {
                regionIndex = static_cast<int>(regions.size());
                labelToRegion[sourceLabel] = regionIndex;
                RegionAttributes region;
                region.sourceLabel = sourceLabel;
                region.parentInitialLabel = initial[col];
                region.histogramCounts.assign(mConfig.histogramBins,0.0);
                region.coreMask = cv::Mat(
                    splitCoreLabels.size(),CV_8UC1,cv::Scalar(0));
                regions.push_back(region);
            }
            RegionAttributes &region = regions[regionIndex];
            if(region.parentInitialLabel!=initial[col])
                throw std::invalid_argument(
                    "SIn RAG split component crosses initial regions");
            ++region.area;
            region.depthSum += depth[col];
            int bin = static_cast<int>(
                depth[col]/mConfig.maximumDepthMeters*mConfig.histogramBins);
            bin = std::max(0,std::min(mConfig.histogramBins-1,bin));
            region.histogramCounts[bin] += 1.0;
            region.coreMask.at<unsigned char>(row,col) = 255;
        }
    }

    result.stats.inputComponentCount = static_cast<int>(regions.size());
    result.stats.inputCorePixels = static_cast<std::size_t>(
        cv::countNonZero(splitCoreLabels>0));
    if(regions.empty())
    {
        result.mergedValidMask = cv::Mat(
            splitCoreLabels.size(),CV_8UC1,cv::Scalar(0));
        result.stats.available = true;
        result.stats.totalMs = ElapsedMilliseconds(
            totalStart,std::chrono::steady_clock::now());
        return result;
    }

    for(std::size_t index=0; index<regions.size(); ++index)
    {
        regions[index].centerDepth =
            regions[index].depthSum/regions[index].area;
        regions[index].score =
            mConfig.areaDepthScoreWeight*regions[index].area-
            regions[index].centerDepth;
    }
    std::vector<int> order(regions.size());
    std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),
        [&regions](const int first, const int second)
        {
            if(regions[first].score!=regions[second].score)
                return regions[first].score>regions[second].score;
            return regions[first].sourceLabel<regions[second].sourceLabel;
        });
    for(std::size_t rank=0; rank<order.size(); ++rank)
        regions[order[rank]].fixedRank = static_cast<int>(rank);
    const int regionCount = static_cast<int>(regions.size());
    result.stats.totalPairCount =
        static_cast<std::size_t>(regionCount)*(regionCount-1)/2;
    std::vector<int> componentFakeBoundary(
        static_cast<std::size_t>(regionCount)*regionCount,0);
    const int rowOffsets[2] = {0,1};
    const int colOffsets[2] = {1,0};
    for(int row=0; row<splitCoreLabels.rows; ++row)
    {
        for(int col=0; col<splitCoreLabels.cols; ++col)
        {
            const int firstLabel = splitCoreLabels.at<int>(row,col);
            if(firstLabel<=0 ||
               realEdgeMask.at<unsigned char>(row,col)!=0)
            {
                continue;
            }
            const int firstRegion = labelToRegion[firstLabel];
            for(int neighbor=0; neighbor<2; ++neighbor)
            {
                const int nextRow = row+rowOffsets[neighbor];
                const int nextCol = col+colOffsets[neighbor];
                if(nextRow>=splitCoreLabels.rows ||
                   nextCol>=splitCoreLabels.cols)
                {
                    continue;
                }
                const int secondLabel =
                    splitCoreLabels.at<int>(nextRow,nextCol);
                if(secondLabel<=0 || secondLabel==firstLabel ||
                   realEdgeMask.at<unsigned char>(nextRow,nextCol)!=0 ||
                   initialLabels.at<int>(row,col)==
                       initialLabels.at<int>(nextRow,nextCol))
                {
                    continue;
                }
                const int secondRegion = labelToRegion[secondLabel];
                ++componentFakeBoundary[
                    static_cast<std::size_t>(firstRegion)*regionCount+
                    secondRegion];
                ++componentFakeBoundary[
                    static_cast<std::size_t>(secondRegion)*regionCount+
                    firstRegion];
            }
        }
    }

    const int diameter = 2*mConfig.adjacencyDilationRadius+1;
    const cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,cv::Size(diameter,diameter));
    std::vector<Group> groups(regionCount);
    for(int rank=0; rank<regionCount; ++rank)
    {
        const int regionIndex = order[rank];
        Group &group = groups[rank];
        group.fixedRank = rank;
        group.members.push_back(regionIndex);
        group.parentInitialLabels.insert(
            regions[regionIndex].parentInitialLabel);
        group.area = regions[regionIndex].area;
        group.depthSum = regions[regionIndex].depthSum;
        group.histogramCounts = regions[regionIndex].histogramCounts;
        group.coreMask = regions[regionIndex].coreMask.clone();
        RecomputeDilation(
            group,element,mConfig.adjacencyDilationRadius);
    }

    // Attribute time includes the component fake-boundary graph and the
    // initial group masks/dilations; those are prerequisites for RAG scoring.
    const std::chrono::steady_clock::time_point attributeEnd =
        std::chrono::steady_clock::now();
    const std::chrono::steady_clock::time_point ragStart =
        std::chrono::steady_clock::now();
    double histogramSimilaritySum = 0.0;
    double eligibleScoreSum = 0.0;
    for(int first=0; first<regionCount; ++first)
    {
        for(int second=first+1; second<regionCount; ++second)
        {
            const PairEvidence evidence = EvaluatePair(
                groups[first],groups[second],componentFakeBoundary,
                regionCount,mConfig);
            if(evidence.sharedFakeEdge)
                ++result.stats.sharedFakeEdgePairCount;
            if(evidence.spatialAdjacent)
            {
                ++result.stats.spatialAdjacentPairCount;
                histogramSimilaritySum += evidence.histogramSimilarity;
                result.stats.maximumHistogramSimilarityOnAdjacentPairs =
                    std::max(
                        result.stats.maximumHistogramSimilarityOnAdjacentPairs,
                        evidence.histogramSimilarity);
            }
            if(evidence.depthRejected)
                ++result.stats.depthRejectedPairCount;
            if(evidence.candidate)
            {
                ++result.stats.eligiblePairCount;
                eligibleScoreSum += evidence.totalScore;
                result.stats.maximumTotalScoreOnEligiblePairs = std::max(
                    result.stats.maximumTotalScoreOnEligiblePairs,
                    evidence.totalScore);
            }
        }
    }
    if(result.stats.spatialAdjacentPairCount>0)
    {
        result.stats.meanHistogramSimilarityOnAdjacentPairs =
            histogramSimilaritySum/result.stats.spatialAdjacentPairCount;
    }
    if(result.stats.eligiblePairCount>0)
    {
        result.stats.meanTotalScoreOnEligiblePairs =
            eligibleScoreSum/result.stats.eligiblePairCount;
    }
    const std::chrono::steady_clock::time_point ragEnd =
        std::chrono::steady_clock::now();

    const std::chrono::steady_clock::time_point mergeStart =
        std::chrono::steady_clock::now();
    const int highMiddleCount = std::max(
        1,std::min(regionCount,static_cast<int>(
            std::ceil(mConfig.highMiddleFraction*regionCount))));
    while(true)
    {
        int bestTarget = -1;
        int bestDonor = -1;
        double bestScore = mConfig.mergeThreshold;
        for(int first=0; first<regionCount; ++first)
        {
            if(!groups[first].active ||
               groups[first].fixedRank>=highMiddleCount)
            {
                continue;
            }
            for(int second=first+1; second<regionCount; ++second)
            {
                if(!groups[second].active ||
                   groups[second].fixedRank>=highMiddleCount)
                {
                    continue;
                }
                const PairEvidence evidence = EvaluatePair(
                    groups[first],groups[second],componentFakeBoundary,
                    regionCount,mConfig);
                if(!evidence.candidate ||
                   evidence.totalScore<=mConfig.mergeThreshold)
                    continue;
                int target = first;
                int donor = second;
                if(groups[target].fixedRank>groups[donor].fixedRank)
                    std::swap(target,donor);
                const bool strictlyBetter = bestTarget<0 ||
                    evidence.totalScore>bestScore+
                        std::numeric_limits<double>::epsilon();
                const bool deterministicTie =
                    !strictlyBetter && bestTarget>=0 &&
                    std::abs(evidence.totalScore-bestScore)<=
                        std::numeric_limits<double>::epsilon() &&
                    (groups[target].fixedRank<groups[bestTarget].fixedRank ||
                     (groups[target].fixedRank==groups[bestTarget].fixedRank &&
                      groups[donor].fixedRank<groups[bestDonor].fixedRank));
                if(strictlyBetter || deterministicTie)
                {
                    bestScore = evidence.totalScore;
                    bestTarget = target;
                    bestDonor = donor;
                }
            }
        }
        if(bestTarget<0)
            break;
        MergeGroup(
            groups[bestTarget],groups[bestDonor],element,
            mConfig.adjacencyDilationRadius);
        ++result.stats.highMiddleMergeCount;
    }

    const double lowThreshold = 0.2*mConfig.mergeThreshold;
    for(int source=highMiddleCount; source<regionCount; ++source)
    {
        if(!groups[source].active)
            continue;
        int bestTarget = -1;
        double bestScore = lowThreshold;
        for(int target=0; target<regionCount; ++target)
        {
            if(!groups[target].active ||
               groups[target].fixedRank>=highMiddleCount)
            {
                continue;
            }
            const PairEvidence evidence = EvaluatePair(
                groups[source],groups[target],componentFakeBoundary,
                regionCount,mConfig);
            if(!evidence.candidate || evidence.totalScore<=lowThreshold)
                continue;
            const bool strictlyBetter = bestTarget<0 ||
                evidence.totalScore>bestScore+
                    std::numeric_limits<double>::epsilon();
            const bool deterministicTie =
                !strictlyBetter && bestTarget>=0 &&
                std::abs(evidence.totalScore-bestScore)<=
                    std::numeric_limits<double>::epsilon() &&
                groups[target].fixedRank<groups[bestTarget].fixedRank;
            if(strictlyBetter || deterministicTie)
            {
                bestScore = evidence.totalScore;
                bestTarget = target;
            }
        }
        if(bestTarget>=0)
        {
            MergeGroup(
                groups[bestTarget],groups[source],element,
                mConfig.adjacencyDilationRadius);
            ++result.stats.lowScoreMergeCount;
        }
        else
        {
            ++result.stats.unmergedLowScoreRegionCount;
        }
    }

    std::vector<int> componentToOutput(regionCount,0);
    std::vector<int> groupComponentCounts;
    std::vector<std::size_t> groupPixelCounts;
    int outputLabel = 0;
    for(int groupIndex=0; groupIndex<regionCount; ++groupIndex)
    {
        const Group &group = groups[groupIndex];
        if(!group.active)
            continue;
        ++outputLabel;
        groupComponentCounts.push_back(
            static_cast<int>(group.members.size()));
        groupPixelCounts.push_back(group.area);
        for(std::size_t member=0; member<group.members.size(); ++member)
            componentToOutput[group.members[member]] = outputLabel;
    }
    result.stats.outputRegionCount = outputLabel;
    result.stats.medianMergedGroupComponents = Median(groupComponentCounts);
    result.stats.maximumMergedGroupComponents = *std::max_element(
        groupComponentCounts.begin(),groupComponentCounts.end());
    result.stats.smallestMergedRegionPixels = *std::min_element(
        groupPixelCounts.begin(),groupPixelCounts.end());
    result.stats.largestMergedRegionPixels = *std::max_element(
        groupPixelCounts.begin(),groupPixelCounts.end());

    // A group with more than one component from the same initial region would
    // have crossed a real gradient edge. Disjoint-parent candidate gating
    // should make this invariant exactly zero.
    for(int groupIndex=0; groupIndex<regionCount; ++groupIndex)
    {
        if(!groups[groupIndex].active)
            continue;
        std::set<int> seenParents;
        for(std::size_t member=0;
            member<groups[groupIndex].members.size(); ++member)
        {
            const int parent =
                regions[groups[groupIndex].members[member]].parentInitialLabel;
            if(!seenParents.insert(parent).second)
                ++result.stats.crossGradientMergeViolationCount;
        }
    }

    std::vector<int> sourceLabelToOutput(
        static_cast<std::size_t>(maximumSourceLabel)+1,0);
    for(int regionIndex=0; regionIndex<regionCount; ++regionIndex)
    {
        sourceLabelToOutput[regions[regionIndex].sourceLabel] =
            componentToOutput[regionIndex];
    }
    for(int row=0; row<splitCoreLabels.rows; ++row)
    {
        const int *split = splitCoreLabels.ptr<int>(row);
        int *output = result.mergedLabels.ptr<int>(row);
        for(int col=0; col<splitCoreLabels.cols; ++col)
        {
            if(split[col]>0)
                output[col] = sourceLabelToOutput[split[col]];
        }
    }
    cv::compare(result.mergedLabels,0,result.mergedValidMask,cv::CMP_GT);
    result.stats.outputCorePixels = static_cast<std::size_t>(
        cv::countNonZero(result.mergedValidMask));
    const std::chrono::steady_clock::time_point mergeEnd =
        std::chrono::steady_clock::now();

    result.stats.available = true;
    result.stats.attributeMs = ElapsedMilliseconds(
        attributeStart,attributeEnd);
    result.stats.ragMs = ElapsedMilliseconds(ragStart,ragEnd);
    result.stats.mergeMs = ElapsedMilliseconds(mergeStart,mergeEnd);
    result.stats.totalMs = ElapsedMilliseconds(
        totalStart,std::chrono::steady_clock::now());
    return result;
}

} // namespace ORB_SLAM2
