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

#define MATRIX_L 1280
#define MATRIX_H 960
#define SUBM_L 80
#define SUBM_H 80

static void floodFill(std::vector<Point>& currentCluster, int xPos, int yPos, const Fovea &topSaliency);

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


    //loop through every coordinate in topFovea
    for(xPos = 0; xPos < imageWidth ; xPos++){
        for(yPos = 0 ; yPos < imageHeight ; yPos++){
            Colour c = topSaliency.colour(xPos, yPos);
            //If the color of the selected pixel is white, subject it to a flood fill
            //This will fill the currentCluster variable with all surround white pixels
            if(c == cWHITE){
                floodFill(currentCluster, xPos, yPos, topSaliency);
                bool found = false;
                unsigned int i;
                unsigned int x;
                Point tempP = currentCluster[0];
                //Check to see if this cluster is already part of our known clusters vector
                for(i = 0 ; i < allClusters.size() ; i++){
                    for(x = 0 ; x < allClusters[i].size(); x++){
                        if(allClusters[i][x] == tempP){
                            found = true;
                        }
                    }
                }
                if(found == false){
                    allClusters.push_back(currentCluster);
                }
                //Reset currentCluster for next use
                currentCluster.clear();
            }
        }
    }

    //push debug points to the largest cluster
    std::vector<Point> biggestCluster;
    biggestCluster = allClusters[0];
    unsigned int i;
    for(i = 0;i < allClusters.size();i++){
        if(allClusters[i].size() > biggestCluster.size()){
            biggestCluster = allClusters[i];
        }
    }

    for(i = 0 ; i < biggestCluster.size() ; i++){
        debugPoints.push_back(topSaliency.mapFoveaToImage(biggestCluster[i]));
    }
}
//This function, given one white pixel will fill a vector with all surrounding white pixels
//which are connected by other white pixels
static void floodFill(std::vector<Point>& currentCluster, int xPos, int yPos, const Fovea &topSaliency){
    if(xPos < 0 || yPos < 0) return;
    unsigned int i;
    bool found;
    Colour c = topSaliency.colour(xPos, yPos);
    Point p = Point(xPos, yPos);
    if(c == cWHITE){
        found = false;
        for(i = 0 ; i < currentCluster.size(); i++){
            if(currentCluster[i] == p){
                found = true;
            }
        }
        if(found == false){
            currentCluster.push_back(p);
        }else{
            return;
        }
    }else{
        return;
    }
    floodFill(currentCluster, xPos - 1, yPos, topSaliency);
    floodFill(currentCluster, xPos + 1, yPos, topSaliency);
    floodFill(currentCluster, xPos, yPos - 1, topSaliency);
    floodFill(currentCluster, xPos, yPos + 1, topSaliency);

}
