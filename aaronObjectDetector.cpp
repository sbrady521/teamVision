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
   //Check the colour of every pixel in the array
   //If the pixel is white and it is touching 2 or more non-white pixels, push that pixel
   int x,y;
   //int possibleEdges[x2][y2] = {0};

   for (y = 1; y < topSaliency.bb.height()-1; y++) {
      for (x = 1; x < topSaliency.bb.width()-1; x++) {
         int numTouching = 0;
         if (topSaliency.colour(x,y) == 6) {
            if (topSaliency.colour(x,y-1) != 6) numTouching++; //check pixel above
            if (topSaliency.colour(x+1,y-1) != 6) numTouching++; //check top right
            if (topSaliency.colour(x+1,y) != 6) numTouching++; //check right
            if (topSaliency.colour(x+1,y+1) != 6) numTouching++; //check bottom right
            if (topSaliency.colour(x,y+1) != 6) numTouching++; //check bottom
            if (topSaliency.colour(x-1,y+1) != 6) numTouching++; //check bottom left
            if (topSaliency.colour(x-1,y) != 6) numTouching++; //check left
            if (topSaliency.colour(x-1,y-1) != 6) numTouching++; //check top left

            if (numTouching >= 2) {
               Point p = Point(x,y);
               p = topSaliency.mapFoveaToImage(p);
               debugPoints.push_back(p);
            }
         }
      }
   }

   //If a possible edge is an isolated pixel. If it is, don't push it because it can't be an edge
   //Otherwise push it

   /*for (y = 1; y < y2-1; y++) {
      for (x = 1; x < x2-1; x++) {
         if (possibleEdges[x][y] == 1) {

         }
      }
   }*/

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



