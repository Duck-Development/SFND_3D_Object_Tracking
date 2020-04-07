
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 5);
    cv::imshow(windowName, topviewImg);

    

    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    boundingBox.kptMatches.clear();
    std::vector<double> dist;
    double distSum;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint prvKeyPoint = kptsPrev[it->queryIdx];
        cv::KeyPoint curKeyPoint = kptsCurr[it->trainIdx];
        if (boundingBox.roi.contains(prvKeyPoint.pt) && boundingBox.roi.contains(curKeyPoint.pt))
        {
            cv::Point2f diff = curKeyPoint.pt - prvKeyPoint.pt;
            double distant = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
            distSum += distant;
            dist.push_back(distant);
            boundingBox.kptMatches.push_back(*it);
            continue;
        }
    }
    auto mean = distSum / dist.size();
    for (size_t i = 0; i < dist.size(); i++)
    {
        if (dist[i] > mean)
        {
            boundingBox.kptMatches.erase(boundingBox.kptMatches.begin() + i);
            dist.erase(dist.begin() + i);
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{

    vector<double> distRatios;
    const auto minDist = 105.0; // minimal dis of KeyPoints
    for (auto outIT = kptMatches.begin()  ;  outIT != kptMatches.end()-1 ; ++outIT)
    { 
        auto kpOuterCurr = kptsCurr.at(outIT->trainIdx);
        auto kpOuterPrev = kptsPrev.at(outIT->queryIdx);

        for (auto inIT = outIT+1  ;  inIT != kptMatches.end(); ++inIT){
            auto kpInnerCurr = kptsCurr.at(inIT->trainIdx);
            auto kpInnerPrev = kptsPrev.at(inIT->queryIdx);

            
            auto distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            auto distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            // No devide by Zero and do not use points witch distance then less minDist
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { 

                distRatios.push_back(distCurr / distPrev);
            }
        } 
    }    

    if (distRatios.size() == 0)
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }


    auto meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();


    std::sort(distRatios.begin(), distRatios.end());
    double medianRatio = 0.0;
    auto half = distRatios.size() / 2;
    if (distRatios.size() % 2 == 0)
    {
        medianRatio = (distRatios.at(half) + distRatios.at(half - 1)) / 2;
    }
    else
    {
        medianRatio = distRatios[half];
    }
   
    
    TTC = -1 / ((1 - medianRatio) *frameRate); 
    std::cout << "###CAM_TTC:" << TTC <<std::endl;


    

}

double getStableDistLidar(std::vector<LidarPoint> lidarPoints)
{
    //convert to 1D array
    double distX = std::numeric_limits<double>::max();
    if (lidarPoints.size() > 0)
    {
        // Sort by x dist
        std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint lhs, LidarPoint rhs) {
        return lhs.x < rhs.x;  
     });

     distX = lidarPoints.at(lidarPoints.size()/2).x;




    }
    return distX;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                          std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    if (lidarPointsPrev.size() > 0 && lidarPointsCurr.size() > 0)
    {
        auto distAct = getStableDistLidar(lidarPointsCurr);
        auto distPre = getStableDistLidar(lidarPointsPrev);
        auto relSpeed = (distAct - distPre) * frameRate;
        
        if (relSpeed < 0)
        {
            TTC = (-distAct / relSpeed); // time is Positiv

            std::cout << "###LIDAR_TTC:" << TTC <<std::endl; 

        }
    }
    else
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
    }
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::map<int, int> mapper;
    for (auto pbbi = 0; pbbi < prevFrame.boundingBoxes.size(); ++pbbi)
    {
        mapper.clear();
        auto &pbb = prevFrame.boundingBoxes.at(pbbi);
        for (auto cmi = 0; cmi < matches.size(); ++cmi)
        {

            const auto pkpi = matches.at(cmi).queryIdx;
            const auto pkp = prevFrame.keypoints.at(pkpi);

            if (pbb.roi.contains(pkp.pt))
            {


                const auto ckpi = matches.at(cmi).trainIdx;
                const auto ckp = currFrame.keypoints.at(ckpi);
                for (auto cbbi = 0; cbbi < currFrame.boundingBoxes.size(); ++cbbi)
                {
                    const auto &cbb = currFrame.boundingBoxes.at(cbbi);
                    if (cbb.roi.contains(ckp.pt))
                    {
                        if (mapper.count(cbbi) == 0)
                        {
                            mapper[cbbi] = 0;
                        }
                        mapper[cbbi]++;
                    }
                }
            }
        }
        //
        int bestMatch = -1;
        int bestCount = 0;

        for (const auto &match : mapper)
        {
            if (match.second > bestCount)
            {
                bestCount = match.second;
                bestMatch = match.first;
            }
        }
        if (bestCount > 0)
        {
            bbBestMatches[pbb.boxID ] = currFrame.boundingBoxes.at(bestMatch).boxID;
            //std::cout << "boundingBox " << cbb.boxID <<  " beste mach bb " << prevFrame.boundingBoxes.at(bestMatch).boxID <<  " count "<<  bestCount << std::endl;
        }
        else
        {
            std::cout << "no Match for BBI:" << pbbi << std::endl;
        }
    }
    
}
