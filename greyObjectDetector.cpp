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

typedef int gmatrix[640][480];
typedef int subMatrix[80][80];
/* ########################### */
/* Constructor and comparators */
/* ########################### */

ObjectDetector::ObjectDetector()
{
   
}
//returns a submatrix of a matrix bounded
/*subMatrix** getSubMatrix(gmatrix m, int topLeftx, int topLefty, int botRightx, int botRighty){
   static subMatrix n;
   if ((topLeftx + SUBM_L != botRightx) ||topLefty + SUBM_H != botRighty){
      fprintf(stderr, "Invalid matrix dimensions\n");
   }
   int xcnt;
   int ycnt;
   for (xcnt = topLeftx; xcnt <= botRightx, xcnt++){
      for (ycnt = topLefty; ycnt <= botRighty, ycnt++){
         n[xnt][ycnt] = m[xcnt][ycnt];
      }
   }
   return n;
}*/


/* ################################### */
/*        Your code goes here      */
/* ################################### */

void ObjectDetector::findObject(
      VisionFrame &frame,
      const Fovea &topSaliency,
      const Fovea &botSaliency)  
{
   int xptr = 0;
   int yptr = 0;
   Point p;
   gmatrix greyMatrix;
   int greyVal;
   
   int whiteThreshold = 180;

   int density = 3;
   bool top = true;
   BBox box;
   box.a = Point(0,0);
   box.b = Point(640,480);
   boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
         new FoveaT<hNone, eGrey>(box, density, 0, top));
   ballFovea->actuate(frame);
   Fovea f2 = ballFovea->asFovea();

   int width = f2.bb.width();
   int height = f2.bb.height();
   printf("%d %d\n", width, height);

   for (xptr = 0; xptr < width; xptr++){
      for (yptr = 0; yptr < height; yptr++){
         greyVal = f2.grey(xptr,yptr);          //store grey values in a matrix
         greyMatrix[xptr][yptr] = greyVal;
      }
   }
   //after getting a grey matrix identify possible points which may be white (val <), possibly remove massive blobs of white
   //perhaps it might be better to take the average of all submatrixes???
   int xcnt;
   int ycnt;
   int avg;

   int subWidth = 80;
   int subHeight = 80;

   int upperThresh = 148;
   int lowerThresh = 50;

   for (xptr = 0; xptr < width; xptr += subWidth){
      for (yptr = 0; yptr < height; yptr += subHeight){
         avg = 0;
         for (xcnt = xptr; xcnt < xptr+subWidth; xcnt++){
            for (ycnt = yptr; ycnt < yptr+subHeight; ycnt++){
               //sub[xcnt][ycnt] = greyMatrix[xcnt][ycnt];
               avg += greyMatrix[xcnt][ycnt];
            }
         }
         avg = avg/(subWidth*subHeight);
         if (avg >= upperThresh || avg <= lowerThresh){
            continue;
         } else {
            for (xcnt = xptr; xcnt < xptr+subWidth; xcnt++){
               for (ycnt = yptr; ycnt < yptr+subHeight; ycnt++){
                  if(greyMatrix[xcnt][ycnt] < whiteThreshold) continue;
                  p = f2.mapFoveaToImage(Point(xcnt,ycnt));
                  debugPoints.push_back(p);
               }
            }
         }
      }
   }

   //join each point with every other point - find lines of similar gradients, suggests there may be a line there
   //or express the point in polar form, and propose a line eqtn : xcos(theta)+ycos(theta) = r
   //check other points and see if they have similar r values...
   /*int xptr = 0;
   int yptr = 0;
   Colour c;
   Colour nextc;
   Point p;
   while(xptr < topSaliency.bb.width()){
   yptr = 0;
      while(yptr < topSaliency.bb.height()){
         c = topSaliency.colour(xptr, yptr);
         nextc = topSaliency.colour(xptr, yptr+1);
         if(c == cWHITE){
            if (nextc == cFIELD_GREEN){
               p = Point(xptr, yptr);
               p = topSaliency.mapFoveaToImage(p);
               debugPoints.push_back(p);  
               yptr+=2;
               continue;   
            }
         } else if (c == cFIELD_GREEN){
            if (nextc == cWHITE){
               p = Point(xptr, yptr);
               p = topSaliency.mapFoveaToImage(p);
               debugPoints.push_back(p);  
               yptr+=2;
               continue; 
            }
         }
         yptr++;
      }
   xptr++;
   }*/


   /*// The fovea class can be accessed in the following ways:
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



