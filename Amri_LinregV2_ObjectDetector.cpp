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

/* ########################### */
/* Constructor and comparators */
/* ########################### */

void linregErrorMargin (std::vector<Point> clusterData, int topHeight, int topWidth);

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
   int fWidth = topSaliency.bb.width();
   int fHeight = topSaliency.bb.height();
   int botHeight = botSaliency.bb.height();
   int botWidth = botSaliency.bb.width();
   bool boundBegins = false;
   bool whiteExists = false;
   bool dataExists = false;

   //linear regression variables
   int numberOfVariables = 0;

   int sumXY = 0;
   int sumX = 0;
   int sumY = 0;

   double averageX = 0;
   int realAverageX = 0;
   int sumAverageX = 0;
   int numberOfX = 0;

   double numerator = 0;
   double denominator = 0;
   double slope = 0;
   double intercept = 0;

   int sumXSquared = 0;

   y = 0;
   while (y < fHeight) {
      x = 0;
      while (x < fWidth) {
         Colour c = topSaliency.colour(x, y);
         if (c == cFIELD_GREEN) {
            boundBegins = true;
         } 
         if (c == cWHITE && boundBegins == true) {
            whiteExists = true;
            dataExists = true;
            sumX += x;
            numberOfX++;
         }
         if (c != cWHITE && whiteExists == true) {
            boundBegins = false;
            whiteExists = false;
         }
         x++;
      }
      if (dataExists == true) { 
         numberOfVariables++;
         dataExists = false;
         sumY += y;
         sumXSquared += x*x;
         sumXY += x*y;
         if (numberOfX > 0) {
            averageX = sumX/numberOfX;
            realAverageX = (int)averageX;
         }
         sumAverageX += realAverageX;
         sumX = 0;
         numberOfX = 0;
      }
      numberOfX = 0;
      y++;
   }

   int finalAverageX = sumAverageX/numberOfVariables;
   int finalAverageY = sumY/numberOfVariables;

   numerator = (sumXY - numberOfVariables*finalAverageY*finalAverageX);
   denominator = (sumXSquared - numberOfVariables*(finalAverageX*finalAverageX));
   slope = (numerator/denominator);
   intercept = finalAverageY - slope*finalAverageX;

   printf("intercept is %f and slope is = %f\n", intercept, slope);
   printf("numerator is = %f and denominator is = %f\n", numerator, denominator);
   x = 0;
   while (x < fWidth) {
      y = (intercept + slope*x);
      y = y + 0.5;
      int realY = (int)y;
      Point p = Point(x, realY); 
      p = topSaliency.mapFoveaToImage(p);
      debugPoints.push_back(p);
      x++;

   }
}
