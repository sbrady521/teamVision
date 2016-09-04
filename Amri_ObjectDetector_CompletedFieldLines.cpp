#include "ObjectDetector.hpp"

#include <algorithm>
#include <limits>
#include <vector>
#include <stack>
#include <math.h>

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

typedef int gmatrix[640][480];
typedef int subMatrix[80][80];

void floodfill(std::vector<Point> &debug, const Fovea top, int x, int y, std::vector<Point> &clustered, int testImage[][120], int height, int width);
void debugFF (std::vector<Point> &gathered);
void storeLastCluster(std::vector<Point> &debug);
double linregSlope (std::vector<Point> &clustered, double *overallAvgX, double *overallAvgY);
double linregIntercept(double averageX, double averageY, double clusterSlope);
double perpendicularErrorMargin (std::vector<Point> &cluster, double slope, double intercept);

//double linregErrorMargin();

/* ########################### */
/* Constructor and comparators */
/* ########################### */

ObjectDetector::ObjectDetector()
{
   
}

/* ################################### */
/*        Your code goes here      */
/* ################################### */

void ObjectDetector::findObject(
      VisionFrame &frame,
      const Fovea &topSaliency,
      const Fovea &botSaliency)
{
   /*
   // The fovea class can be accessed in the following ways:
   Fovea f;
   Colour c = f.colour(x,y);
   int greyVal = fovea.grey(x,y);
   Point edge = fovea.edge(x,y);

   // The point object is a 2D point on the image
   // This is how we can declare it
   Point p = Point(x,y);
   Point p(x,y);

   // To access parts of it, do
   int x = p.x();
   int y = p.y();

   // We have a list of debug points that can be displayed on offnao as orange dots
   // Add points to this list thus:
   debugPoints.push_back(pointWeWantToAdd);
   // These points must be in total image coordinates


   // The top saliency has size 160x120
   // The bottom 80x60

   // To create a new fovea, we do the following:
   // top is boolean, for whether this is in the top image or not
   bool top = true;

   // Density refers to downsampling factor from native resolution
   // native res: top - 1280x960. bottom - 640x480
   int density = 2;

   // A bounding box has a top left and a bottom right coordinate.
   // This is the edge of the region we want
   BBox box;
   box.a = Point(11,15);
   box.b = Point(25,36);
   // NOTE: the bounding box coordinates must be at the resolution that you want

   boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
         new FoveaT<hNone, eGrey>(box, density, 0, top));

   // Now process the fovea to create it. We need to pass in the frame object
   // as that has access to the raw image
   ballFovea->actuate(frame);

   // Then get a fovea from that
   Fovea f2 = ballFovea->asFovea();

   // Now we use f2 as above


   // We can convert from fovea to total coordinates thus:
   Point total = fovea.mapFoveaToImage(foveapoint);
   // and the reverse:
   Point foveaPoint = fovea.mapImageToFovea(total);
   */



   //LINEAR REGRESSION V3

   //INSTRUCTIONS
   /*

   LINREG for SALIENCY:
   Traverse image to get (un?)classified data!
   Want to first cluster points, say by radius of 3 pixels.
   Next, apply linreg to each cluster.
   Calculate margin of error on each line
   Then, above a certain error, is not a line.
   Look at results!

   GREYSCALE:
   Apply what you have seen in e-mails. 
   Gives practice using foveas too.

   EDGES:
   -
   */

   //INITIALISATION 

   
   int y = 0;
   int x = 0;
   int tWidth = topSaliency.bb.width(); //160
   int tHeight = topSaliency.bb.height(); //120
   int botHeight = botSaliency.bb.height(); //60
   int botWidth = botSaliency.bb.width(); //80
   int image[160][120] = {{0}};
   int botImage[80][60] = {{0}};
   double averageXValue = 0;
   double averageYValue = 0;
   double errorMargin = 0;
   double slope = 0;
   double intercept = 0;

   Colour whiteTest;
   Point p;
   std::vector<Point> botCluster;
   std::vector<Point> cluster;
   
   //printf("width = %d, height = %d\n", botWidth, botHeight);

   //BOTTOM SALIENCY
   //Have not yet changed this for linreg method
   //just pushing the white pixels currently ¯\_(ツ)_/¯

   //unfinished linreg application
   /*y = 0;
   while (y < botHeight) {
      x = 0;
      while (x < botWidth) {
         whiteTest = botSaliency.colour(x,y);
         floodfill(debugPoints, botSaliency, x, y, botCluster, botImage, botHeight, botWidth);
         x++;
         if (botCluster.size() < 3) {
            debugPoints.erase(debugPoints.end()-cluster.size(),debugPoints.end());
         } else {
            slope = linregSlope(botCluster, &averageXValue, &averageYValue);
            intercept = linregIntercept(averageXValue, averageYValue, slope);
            errorMargin = perpendicularErrorMargin(botCluster, slope, intercept);
            if (errorMargin < (-0.019) || errorMargin > 0.019) { 
               debugPoints.erase(debugPoints.end()-cluster.size(), debugPoints.end());
            }
            averageXValue = 0;
            averageYValue = 0;
            error Margin = 0;
         }
      }
      y++;
   }*/

   y = 0;
   while (y < botHeight) {
      x = 0;
      while (x < botWidth) {
         whiteTest = botSaliency.colour(x, y);
         p = Point(x,y);
         p = botSaliency.mapFoveaToImage(p);
         if (whiteTest == cWHITE) {
            debugPoints.push_back(p);
         }
         x++;
      }
      y++;
   }

   //TOP SALIENCY

   y = 0;
   while (y < tHeight) {
      x = 0;
      while (x < tWidth) {
         whiteTest = topSaliency.colour(x, y);
         p = Point(x, y);
         if (whiteTest == cWHITE && image[x][y] != 1) {
            floodfill(debugPoints, topSaliency, x, y, cluster, image, tHeight, tWidth);
            //printf("%d GAP\n", counter);
            //debugFF(cluster)
            //remove clusters of small size
            if (cluster.size() < 3) {
               debugPoints.erase(debugPoints.end()-cluster.size(),debugPoints.end());
            } else {
               slope = linregSlope(cluster, &averageXValue, &averageYValue);
               intercept = linregIntercept(averageXValue, averageYValue, slope);
               //printf("slope = %f, intercept = %f\n", slope, intercept);
               errorMargin = perpendicularErrorMargin(cluster, slope, intercept);
               //printf("cluster size %d AND ERROR MARGIN = %f\n", cluster.size(), errorMargin);
            }
            //arbitrary margins to determine whether line is legitimate
            //formed by a bit of trial and error.
            if (errorMargin < (-0.019) || errorMargin > 0.019) { 
             debugPoints.erase(debugPoints.end()-cluster.size(), debugPoints.end());
            }
            //free cluster to process more
            cluster.erase(cluster.begin(),cluster.end()); 
            averageXValue = 0;
            averageYValue = 0;
            errorMargin = 0;
         }
         x++;
      }
      y++;
   }
   //printf("SIZE = %d\n", debugPoints.size());
   //int vecSize = debugPoints.size();
   //debugPoints.erase(debugPoints.begin(),debugPoints.begin()+vecSize);

}

void floodfill(std::vector<Point> &debug, const Fovea top, int x, int y, std::vector<Point> &clustered, int testImage[][120], int height, int width) {
   Point p = Point(x,y);
   clustered.push_back(p);
   Point draw = top.mapFoveaToImage(p);
   debug.push_back(draw);
   testImage[x][y] = 1;

   //I have seen more efficient ways to write this.

   Colour cRight = top.colour(x + 1, y);
   Colour cDown = top.colour(x, y + 1);
   Colour cLeft = top.colour(x - 1, y);
   Colour cUp = top.colour(x, y - 1);

   if (cLeft == cWHITE && testImage[x-1][y] != 1) {
      floodfill(debug, top, x-1, y, clustered, testImage, height, width);
   } 
   if (cRight == cWHITE && testImage[x+1][y] != 1) {
      floodfill(debug, top, x+1, y, clustered, testImage, height, width);
   } 
   if (cUp == cWHITE && testImage[x][y-1] != 1) {
      floodfill(debug, top, x, y-1, clustered, testImage, height, width);
   } 
   if (cDown == cWHITE && testImage[x][y+1] != 1) {
      floodfill(debug, top, x, y+1, clustered, testImage, height, width);
   }
}

//function to display all values in a cluster. 
void debugFF (std::vector<Point> &gathered) {
   std::vector<Point>::iterator cI;
   for (cI = gathered.begin(); cI != gathered.end(); cI++) {
      Point temp = *cI;
      printf("x = %d, y = %d\n", temp.x(), temp.y());
   }
   printf("VECTOR FINISHED\n");
}

//if you feel like cheating
/*void storeLastCluster(std::vector<Point> &debug) {
   debug.erase(debug.begin(),debug.end());
}*/

//calculates slope component of linreg
double linregSlope (std::vector<Point> &clustered, double *overallAvgX, double *overallAvgY) {
   int numberOfPoints = clustered.size();
   double sumX = 0;
   double sumY = 0;
   double sumXY = 0;
   double sumXSquared = 0;
   double averageX = 0;
   double averageY = 0;
   double numerator = 0;
   double denominator = 0;
   double slope = 0;
   
   Point temp;
   std::vector<Point>::iterator cI;
   for (cI = clustered.begin(); cI != clustered.end(); cI++) {
      temp = *cI;
      sumXY += (temp.x() * temp.y());
      sumXSquared += (temp.x() * temp.x());
      sumY += temp.y();
      sumX += temp.x();
   }
   averageY = sumY/numberOfPoints;
   averageX = sumX/numberOfPoints;
   numerator = (sumXY - numberOfPoints*(averageX*averageY));
   double denomOne = numberOfPoints*averageX*averageY;
   denominator = (sumXSquared - denomOne);
   /*printf("DENOMINATOR: SUMXSQUARED(%f) - %f, so denominator = %f\n", sumXSquared, numberOfPoints*(averageX*averageY), denominator);
   printf("averageX = %f, averageY = %f\n", averageX, averageY);
   printf("sumXY = %f sumXSquared = %f\n", sumXY, sumXSquared);
   printf("sumXY = %f, numberOfPoints = %d\n", sumXY, numberOfPoints);
   printf("avg x = %f, avgy = %f, num = %f, den = %f\n", averageX, averageY, numerator, denominator);*/
   *overallAvgY = averageY;
   *overallAvgX = averageX;
   //printf("overallavgx %f overallavgy %f\n", overallAvgX, overallAvgY);
   return slope = (numerator/denominator);

}

//calculates intercept component of linreg
double linregIntercept(double averageX, double averageY, double clusterSlope) {
   double intercept = averageY - clusterSlope*averageX;
   return intercept;
}


//error margin
//works by finding perpendicular distance of points in a cluster
//to its line from linreg.
double perpendicularErrorMargin (std::vector<Point> &cluster, double slope, double intercept) {
   double errorSum = 0;

   std::vector<Point>::iterator cI;
   for (cI = cluster.begin(); cI != cluster.end(); cI++) {
      Point temp = *cI;
      int distance = (temp.x()*slope + (-1)*temp.y() + intercept)/(sqrt(slope*slope + (-1)*(-1)));
      errorSum += distance;
   }

   double errorMargin = errorSum/cluster.size();
   //printf("errorSum = %f, cluster size = %d\n", errorSum, cluster.size());
   //printf("error margin is %f\n", errorMargin);
   return errorMargin;
}
