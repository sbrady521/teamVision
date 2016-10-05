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
static void accumulate(std::vector<Point>& clusterVector, int **bin);
static void HoughTranform(int **bin, const Fovea &fov);


ObjectDetector::ObjectDetector()
{

}




void ObjectDetector::findObject(
      VisionFrame &frame,
      const Fovea &topSaliency,
      const Fovea &botSaliency)
{
    debugPoints.clear();
    //Initialise vectors
    std::vector<Point> whites;
    int xPos;
    int yPos;
    Point white;
    int imageWidth = topSaliency.bb.width();
    int imageHeight = topSaliency.bb.height();

    int ** registeredPoints = (int**)malloc(sizeof(int*)*imageWidth);
    unsigned int i;
    for(i = 0; i < imageWidth ; i++){
        registeredPoints[i] = (int*)calloc(imageHeight, sizeof(int));
    }


    //loop through every coordinate in topFovea
    for(xPos = 0; xPos < imageWidth ; xPos++){
        for(yPos = 0 ; yPos < imageHeight ; yPos++){
            Colour c = topSaliency.colour(xPos, yPos);
            if(c == cWHITE){
                white = Point(xPos, yPos);
                whites.push_back(white);
            }
        }
    }


    int **bin = (int**) calloc(181, sizeof(int*));
    for (int i = 0; i < 181; i++){
        bin[i] = (int*)calloc(201, sizeof(int));
    }
    accumulate(whites, bin);
    int r;
    int theta;
    int threshold = 90;
    int midx = 80;
    int midy = 60;
    double Pi = std::acos(-1);
    int xptr, yptr;
    Point p;
    for (r = 0; r < 200; r++){
        for (theta = 0; theta < 180; theta += 5){            
            if (bin[theta][r] > threshold){
                for(xptr = 0; xptr < 160; xptr++){
                    yptr = midy - ((r-100) - (xptr-midx)*std::cos(theta*Pi/180.0))/std::sin(theta*Pi/180.0);
                    //printf("Pushing xptr = %d, yptr = %d\n", xptr, yptr);
                    //printf("plotting y = (%d - %dcos(%d))/sin(%d)\n", r-200, xval, theta, theta);
                    //printf("%d %d\n", xptr, yptr);
                    p = Point(xptr, yptr);
                    debugPoints.push_back(topSaliency.mapFoveaToImage(p));
                    //printf("pushed xptr = %d, yptr = %d, and theta = %d, r = %d\n", xptr, yptr, theta, r);
                }
            }
        }
    }
    free(registeredPoints);
    free(bin);
}

static void accumulate(std::vector<Point>& clusterVector, int ** bin){
    Point p;
    double xval;
    double yval;
    double r;
    double theta;
    double midx = 80;
    double midy = 60;
    double Pi = std::acos(-1);
    for (int i = 0; i < clusterVector.size(); i++){
        p = clusterVector[i];
        xval = p.x() - midx;
        yval = midy - p.y();
        for(theta = 0; theta < 180; theta += 5){
           r = xval*std::cos(theta*Pi/180.0) + (yval)*std::sin(theta*Pi/180.0);
           //fprintf(stderr, "%lf %lf\n", r, theta);
           if (r > 100) continue;
           bin[(int)theta][(int)r+100]++;
        }
    }
}

static void HoughTranform(int **bin, const Fovea &fov){
    int r;
    int theta;
    int threshold = 5000;
    for (r = 0; r < 400; r++){
        for (theta = 0; theta < 180; theta += 5){
            if (bin[theta][r] > threshold){
                //pushLine(r, theta, fov);
            }
        }
    }
}
/*
void ObjectDetector::pushLine(int r, int theta, const Fovea &fov){
    double Pi = std::acos(-1);
    Point p;
    int xptr, yptr;
    for(xptr = 0; xptr <= 160; xptr++){
        yptr = ((r-100) - (xptr-100)*std::cos(theta*Pi/180.0))/std::sin(theta*Pi/180.0);
        //printf("plotting y = (%d - %dcos(%d))/sin(%d)\n", r-200, xval, theta, theta);
        //printf("%d %d\n", xptr, yptr);
        p = Point(xptr, yptr);
        debugPoints.push_back(fov.mapFoveaToImage(p));
    }
}*/

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
