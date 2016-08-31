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
  int whiteThreshold = 180;
  Point p;
  int mainMatrix[1280][960];
  int density = 1;
  bool top = true;
  BBox box;
  box.a = Point(0,0);
  box.b = Point(1280, 960);
  boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
        new FoveaT<hNone, eGrey>(box, density, 0, top));
  ballFovea->actuate(frame);
  Fovea f2 = ballFovea->asFovea();

  int xpoint;
  int ypoint;

  for(xpoint = 0 ; xpoint < f2.bb.width() ; xpoint++){
    for(ypoint = 0 ; ypoint < f2.bb.height() ; ypoint++){
      mainMatrix[xpoint][ypoint] = f2.grey(xpoint, ypoint);
    }
  }

  int subX;
  int subY;
  int totalGrey = 0;
  for(xpoint = 0 ; xpoint < f2.bb.width() ; xpoint += 80){
    for(ypoint = 0 ; ypoint < f2.bb.height() ; ypoint += 80){
      totalGrey = 0;
      for(subX = 0 ; subX < 80 ; subX++){
        for(subY = 0 ; subY < 80 ; subY++){
          totalGrey += mainMatrix[xpoint + subX][ypoint + subY];
        }
      }
      totalGrey = totalGrey / 6400;
      if(totalGrey >= 150 || totalGrey <= 50){
        continue;
      }else{
        for(subX = 0 ; subX < 80 ; subX++){
          for(subY = 0 ; subY < 80 ; subY++){
            if(mainMatrix[xpoint + subX][ypoint + subY] > whiteThreshold){
              p = f2.mapFoveaToImage(Point(xpoint + subX,ypoint + subY));
              debugPoints.push_back(p);
            }
          }
        }
      }
    }
  }



   // The fovea class can be accessed in the following ways:
   /*
   Fovea f;
   Colour c = f.colour(x,y);
   int greyVal = fovea.grey(x,y);
   Point edge = fovea.edge(x,y);

   // The point object is a 2D point on the image
   // This is how we can declare it
   Point p = Point(x,y);
   Point p(x,y);debugPoints.push_back()

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
