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
   //Look for a white pixel with a non-white pixel below it
   //Check if the pixel to the right of the white pixel is also white
   //Check if the pixel to the right of the non-white pixel is also non-white
   //Repeat until you find 4 adjacent white pixels with 4 adjacent non-white pixels below them
   //This is considered to be an edge - push those 4 white pixels as debug points
   int x,y;
   int x2 = topSaliency.bb.width();
   int y2 = topSaliency.bb.height();
   for (y = 0; y < y2-1; y++) {
      for (x = 0; x < x2-3; x++) {
         Colour pixel = topSaliency.colour(x,y);
         Colour pixelBottom = topSaliency.colour(x,y+1);
         if (pixel == 6 && pixelBottom != 6) {
            Colour pixel = topSaliency.colour(x+1,y);
            Colour pixelBottom = topSaliency.colour(x+1,y+1);
            if (pixel == 6 && pixelBottom != 6) {
               Colour pixel = topSaliency.colour(x+2,y);
               Colour pixelBottom = topSaliency.colour(x+2,y+1);
               if (pixel == 6 && pixelBottom != 6) {
                  Colour pixel = topSaliency.colour(x+3,y);
                  Colour pixelBottom = topSaliency.colour(x+3,y+1);
                  Point p1 = Point(x,y);
                  Point p2 = Point(x+1,y);
                  Point p3 = Point(x+2,y);
                  Point p4 = Point(x+3,y);
                  p1 = topSaliency.mapFoveaToImage(p1);
                  p2 = topSaliency.mapFoveaToImage(p2);
                  p3 = topSaliency.mapFoveaToImage(p3);
                  p4 = topSaliency.mapFoveaToImage(p4);
                  debugPoints.push_back(p1);
                  debugPoints.push_back(p2);
                  debugPoints.push_back(p3);
                  debugPoints.push_back(p4);
                  printf("Points pushed\n");
               }
            }
         }
      }
   }

   //Get all possible combinations of 3 points with greyVal >= 200
   //Find area of triangle they make
   //If area < threshold, push the points

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



