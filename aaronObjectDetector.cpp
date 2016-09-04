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
   //Check the colour of every pixel in the saliency image
   //If the pixel is white and it is touching 2 or more green pixels, add it to the array of possible edge points
   int x,y;
   int imageWidth = topSaliency.bb.width();
   int imageHeight = topSaliency.bb.height();

   //This array contains 0s and 1s 
   //1 = that corresponding point in the image is a possible edge point
   //0 = that point is not an edge point
   int possibleEdgePoints[160][120] = {0}; //Since top saliency is 160x120

   for (y = 1; y < imageHeight-1; y++) {
      for (x = 1; x < imageWidth-1; x++) {
         int numTouching = 0;
         if (topSaliency.colour(x,y) == cWHITE) {
            if (topSaliency.colour(x,y-1) == cFIELD_GREEN) numTouching++; //check pixel above
            if (topSaliency.colour(x+1,y-1) == cFIELD_GREEN) numTouching++; //check top right
            if (topSaliency.colour(x+1,y) == cFIELD_GREEN) numTouching++; //check right
            if (topSaliency.colour(x+1,y+1) == cFIELD_GREEN) numTouching++; //check bottom right
            if (topSaliency.colour(x,y+1) == cFIELD_GREEN) numTouching++; //check bottom
            if (topSaliency.colour(x-1,y+1) == cFIELD_GREEN) numTouching++; //check bottom left
            if (topSaliency.colour(x-1,y) == cFIELD_GREEN) numTouching++; //check left
            if (topSaliency.colour(x-1,y-1) == cFIELD_GREEN) numTouching++; //check top left

            if (numTouching >= 2) {
               possibleEdgePoints[x][y] = 1;
            }
         }
      }
   }

   int sqroot = sqrt(imageWidth*imageWidth + imageHeight*imageHeight); //200
   int houghGraph[360][sqroot];

   for (y = 1; y < imageHeight-1; y++) {
      for (x = 1; x < imageWidth-1; x++) {
         if (possibleEdgePoints[x][y] == 1) {
            //Find r and theta for all possible lines through (x,y)
            //Increment the corresponding (theta,r) points on houghGraph

            //coordinates are with respect to the bottom left corner of the 2D array
            //i.e. bottom left corner is (0,0)
            int xCoord = x; 
            int yCoord = imageHeight - y;

            int theta, r;
            for (theta = 0; theta < 360; theta++) { //Theta is in degrees, not radians
               r = 0;
               while ((int)round(xCoord*cos(degToRad(theta)) + yCoord*sin(degToRad(theta))) != r) {
                  r++;
               }
               houghGraph[theta][r]++;
            }
         }
      }
   }

   //TODO - Loop through houghGraph, find the (theta,r) points with the highest value(s)
   //Push all points along those line(s) to offnao


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
   
}



