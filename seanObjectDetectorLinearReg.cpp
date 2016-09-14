#include "ObjectDetector.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "Ransac.hpp"
#include "yuv.hpp"
#include "J48Tree.hpp"

#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/BresenhamPtr.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/body.hpp"
#include "CameraToRR.hpp"
#include "utils/Timer.hpp"
#include "types/Cluster.hpp"
#include "types/FieldFeatureInfo.hpp"

#include "types/Point.hpp"
#include "types/XYZ_Coord.hpp"
#include "types/RobotInfo.hpp"

using namespace std;
using namespace boost::numeric::ublas;

static void floodFill(std::vector<Point>& currentCluster, int xPos, int yPos, const Fovea &imageFov, int ** registeredPoints);
static double * getLine(std::vector<Point>& clusterVector);
static double perpDist(double * line, Point p);
static int * checkRanges(std::vector<Point>& clusterVector);

ObjectDetector::ObjectDetector()
{

}




void ObjectDetector::findObject(
      VisionFrame &frame,
      const Fovea &topSaliency,
      const Fovea &botSaliency)
{

    //Initialise vectors
    std::vector<Point> currentCluster;
    std::vector<std::vector<Point> > allClusters;
    int xPos;
    int yPos;

    int imageWidth = topSaliency.bb.width();
    int imageHeight = topSaliency.bb.height();

    int ** registeredPoints = (int**)malloc(sizeof(int*)*imageWidth);
    unsigned int i;
    unsigned int x;
    for(i = 0; i < imageWidth ; i++){
        registeredPoints[i] = (int*)malloc(sizeof(int)*imageHeight);
        for(x = 0 ; x < imageHeight ; x++){
            registeredPoints[i][x] = 0;
        }
    }


    //loop through every coordinate in topFovea
    for(xPos = 0; xPos < imageWidth ; xPos++){
        for(yPos = 0 ; yPos < imageHeight ; yPos++){
            Colour c = topSaliency.colour(xPos, yPos);
            //If the color of the selected pixel is white, subject it to a flood fill
            //This will fill the currentCluster variable with all surround white pixels
            if(c == cWHITE && registeredPoints[xPos][yPos] == 0){
                floodFill(currentCluster, xPos, yPos, topSaliency, registeredPoints);
                if(currentCluster.size() > 3){
                    allClusters.push_back(currentCluster);
                }
                //Reset currentCluster for next use
                currentCluster.clear();
            }
        }
    }


    //fit lines to all clusters
    unsigned int counter1;
    unsigned int counter2;
    double distAvg = 0;
    double dist;
    double * line;
    int * ranges;
    printf("allCluster size before is %d\n", allClusters.size());
    int removed = 0;
    for(counter1 = 0;counter1 < allClusters.size();counter1++){
        line = getLine(allClusters[counter1]);
        ranges = checkRanges(allClusters[counter1]);
        distAvg = 0;
        for(counter2 = 0 ; counter2 < allClusters[counter1].size() ; counter2++){
            dist = perpDist(line, allClusters[counter1][counter2]);
            distAvg += dist;
            //printf("%f\n", distAvg/counter2);
        }
        //printf("distTotal = %f\n", distAvg);
        distAvg = distAvg / (allClusters[counter1].size());
        //printf("distAvg = %f\n", distAvg);
        //printf("distAvg of cluster %d is %f\n", counter1, distAvg);
        int badLine = 0;
        if(distAvg < 1){
            badLine = 1;
            printf("removing: %d (distAvg is %f)\n", counter1, distAvg);
        }else if(ranges[0] < 20 && ranges[1] < 20){
            printf("removing: %d (range is x = %d and y = %d)\n",counter1, ranges[0], ranges[1]);
            badLine = 1;
        }
        if(badLine == 1){
            removed++;
            allClusters.erase(allClusters.begin() +counter1);
        }
    }
    printf("removed %d\n", removed);
    printf("allCluster size after is %d\n", allClusters.size());
    for(counter1 = 0; counter1 < allClusters.size() ; counter1++){
        for(counter2 = 0 ; counter2 < allClusters[counter1].size() ; counter2++){
            debugPoints.push_back(topSaliency.mapFoveaToImage(allClusters[counter1][counter2]));
        }
    }



    //Some good debugging code for checking clusters
    /*
    char userInput;
    double * line;
    int y;
    double dist;
    unsigned int counter;
    int target;
    scanf("%d", &target);
    debugPoints.clear();
    double distAvg = 0;
    line = getLine(allClusters[target]);
    for(counter = 0; counter < allClusters[target].size() ; counter++){
      debugPoints.push_back(topSaliency.mapFoveaToImage(allClusters[target][counter]));
      dist = perpDist(line, allClusters[target][counter]);
      distAvg += dist;
    }
    for(counter = 0 ; counter < topSaliency.bb.width() ; counter++){
      y = line[0] * counter + line[1];
      debugPoints.push_back(topSaliency.mapFoveaToImage(Point((int)counter, y)));
    }
    distAvg = distAvg / allClusters[target].size();
    int * ranges = checkRanges(allClusters[target]);
    printf("cluster %d/%d is %d pixels big\n", target, allClusters.size(), allClusters[target].size());
    printf("its line of best fit is y = %fx + %f\n", line[0], line[1]);
    printf("its avg perpendicular dist is %f\n", distAvg);
    printf("ranges are x = %d y = %d\n", ranges[0], ranges[1]);
    printf("more specfically x = %d - %d, y = %d - %d\n", ranges[2], ranges[3], ranges[4], ranges[5]);
    scanf("%c", &userInput);
    target++;
    */
}
//This function, given one white pixel will fill a vector with all surrounding white pixels
//which are connected by other white pixels
static void floodFill(std::vector<Point>& currentCluster, int xPos, int yPos, const Fovea &imageFov, int ** registeredPoints){
    if(xPos < 0 || yPos < 0) return;
    Colour c = imageFov.colour(xPos, yPos);
    Point p = Point(xPos, yPos);
    if(c == cWHITE){
        if(registeredPoints[xPos][yPos] == 0){
            registeredPoints[xPos][yPos] = 1;
            currentCluster.push_back(p);
        }else{
            return;
        }
    }else{
        return;
    }
    floodFill(currentCluster, xPos - 1, yPos, imageFov, registeredPoints);
    floodFill(currentCluster, xPos + 1, yPos, imageFov, registeredPoints);
    floodFill(currentCluster, xPos, yPos - 1, imageFov, registeredPoints);
    floodFill(currentCluster, xPos, yPos + 1, imageFov, registeredPoints);

}

static double * getLine(std::vector<Point>& clusterVector){
    double * line = (double*)malloc(sizeof(double)*2);
    unsigned int i;
    double xSum = 0;
    double ySum = 0;
    double m;
    double b;
    int n = clusterVector.size();
    for(i = 0 ; i < clusterVector.size() ; i++){
        ySum += (double)clusterVector[i].y();
        xSum += (double)clusterVector[i].x();
    }
    double xAvg = xSum/(double)n;
    double yAvg = ySum/(double)n;
    double numerator = 0;
    double denominator = 0;
    for(i = 0 ; i < clusterVector.size() ; i++){
        numerator += (clusterVector[i].x() - xAvg)*(clusterVector[i].y() - yAvg);
        denominator += (clusterVector[i].x() - xAvg)*(clusterVector[i].x() - xAvg);
    }
    m = numerator/denominator;
    b = yAvg - m*xAvg;
    line[0] = m;
    line[1] = b;
    //printf("line is y = %fx + %f\n", line[0], line[1]);
    return line;
}

static double perpDist(double * line, Point p){
    double numerator = p.x() * line[0] - p.y() + line[1];
    double denominator = sqrt(1 + line[0]*line[0]);
    double dist = abs(numerator/denominator);
    return dist;
}

static int * checkRanges(std::vector<Point>& clusterVector){
  unsigned int counter;
  int * ranges = (int*)(malloc(sizeof(int)*6));
  int highestX = clusterVector[0].x();
  int smallestX = clusterVector[0].x();
  int highestY = clusterVector[0].y();
  int smallestY = clusterVector[0].y();
  for(counter = 0 ; counter < clusterVector.size() ; counter++){
    if(clusterVector[counter].x() > highestX){
      highestX = clusterVector[counter].x();
    }
    if(clusterVector[counter].x() < smallestX){
      smallestX = clusterVector[counter].x();
    }
    if(clusterVector[counter].y() > highestY){
      highestY = clusterVector[counter].y();
    }
    if(clusterVector[counter].y() < smallestY){
      smallestY = clusterVector[counter].y();
    }
  }
  ranges[0] = highestX - smallestX;
  ranges[1] = highestY - smallestY;
  ranges[2] = highestX;
  ranges[3] = smallestX;
  ranges[4] = highestY;
  ranges[5] = smallestY;
  return ranges;
}
