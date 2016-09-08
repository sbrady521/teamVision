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

void floodFill(int x, int y, std::vector<Point> &vector, int **bArray, Fovea image) {
   Point p = Point(x, y);
   if (image.colour(p) == cWHITE && bArray[y][x] != 1) {
      vector.push_back(p);
      bArray[y][x] = 1;
   } else {
      return;
   }

   floodFill(x+1, y, vector, bArray, image);
   floodFill(x-1, y, vector, bArray, image);
   floodFill(x, y+1, vector, bArray, image);
   floodFill(x, y-1, vector, bArray, image);
}

void ObjectDetector::findObject(
      VisionFrame &frame,
      const Fovea &topSaliency,
      const Fovea &botSaliency)
{
   int width = topSaliency.bb.width();
   int height = topSaliency.bb.height();
   std::vector< std::vector<Point> > clusers;
   int bArray[height][width];
   memset(bArray, 0, sizeof(bArray[0][0]) * height * width);



   int x, y, i, j;
   Point p;
   for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
         p = Point(x, y);
         if (topSaliency.colour(p) == cWHITE) {
            std::vector<Point> v;
            floodFill(x, y, v, &bArray, topSaliency);
            clusers.push_back(v);
         }
      }
   }

   for (i = 0; i < clusters.size(); i++) {
      printf("For cluster %d\n", i);
      for (j = 0; j < clusers[i].size(); j++) {
         printf("(%d, %d), ");
      }
      printf("\n===============\n");
   }

   // int arg1, arg2, add, sub, mul, quo, rem ;

   // printf( "Enter two integer numbers : " );
   // scanf( "%d%d", &arg1, &arg2 );


   /* Perform Addition, Subtraction, Multiplication & Division */
   // __asm__ ( "addl %%ebx, %%eax;" : "=a" (add) : "a" (arg1) , "b" (arg2) );
   // __asm__ ( "subl %%ebx, %%eax;" : "=a" (sub) : "a" (arg1) , "b" (arg2) );
   // __asm__ ( "imull %%ebx, %%eax;" : "=a" (mul) : "a" (arg1) , "b" (arg2) );
   // __asm__ ( "movq (%eax), %mm0;" );
   // __asm__ ( "movl $0x0, %%edx;"
   //          "movl %2, %%eax;"
   //          "movl %3, %%ebx;"
   //          "idivl %%ebx;" : "=a" (quo), "=d" (rem) : "g" (arg1), "g" (arg2) );
    // __asm__ ("mov    eax, 1;"
    //          "cpuid;"
    //          "mov    eax, edx;"
    //          "shr    eax, 23;"
    //          "and    eax, 1 ;" : "=a" (res));

   // printf( "%d + %d = %d\n", arg1, arg2, add );
   // printf( "%d - %d = %d\n", arg1, arg2, sub );
   // printf( "%d * %d = %d\n", arg1, arg2, mul );
   // printf( "%d / %d = %d\n", arg1, arg2, quo );
   // printf( "%d %% %d = %d\n", arg1, arg2, rem );

   // int x, y, i;
   // int xDirs[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
   // int yDirs[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
   // int width = topSaliency.bb.width();
   // int height = topSaliency.bb.height();
   // bool isLine;

   // for (x = 0; x < width; x++) {
   //    for (y = 0; y < height; y++) {
   //       Point p = Point(x, y);
   //       if (topSaliency.colour(p) == cWHITE) {
   //          isLine = false;
   //          for (i = 0; i < 9; i++) {
   //             Point cPoint = Point(x-xDirs[i], y-yDirs[i]);
   //             if (cPoint.x() >= 0 && cPoint.x() < width && 
   //                 cPoint.y() >=0 && cPoint.y() < height) {
   //                if (topSaliency.colour(cPoint) == cFIELD_GREEN) {
   //                   isLine = true;
   //                   debugPoints.push_back(topSaliency.mapFoveaToImage(cPoint));
   //                   break;
   //                }
   //             }
   //          }
   //       }
   //    }
   // }

   // // To create a new fovea, we do the following:
   // // top is boolean, for whether this is in the top image or not
   // bool top = true;

   // // Density refers to downsampling factor from native resolution
   // // native res: top - 1280x960. bottom - 640x480
   // int density = 2;

   // // A bounding box has a top left and a bottom right coordinate.
   // // This is the edge of the region we want
   // BBox box;
   // box.a = Point(0,0);
   // box.b = Point(640,480);
   // // NOTE: the bounding box coordinates must be at the resolution that you want

   // boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
   //       new FoveaT<hNone, eGrey>(box, density, 0, top));

   // // Now process the fovea to create it. We need to pass in the frame object
   // // as that has access to the raw image
   // ballFovea->actuate(frame);

   // // Then get a fovea from that
   // Fovea f2 = ballFovea->asFovea();

   // Point p;
   // int x, y;
   // for (x = 0; x < f2.bb.width(); x++) {
   //    for (y = 0; y < f2.bb.height(); y++) {
   //       p = f2.mapFoveaToImage(Point(x,y));
   //       debugPoints.push_back(p);
   //    }
   // }
}

/**
   // The fovea class can be accessed in the following ways:
   Fovea f;
   Colour c = f.colour(x,y);
   int greyVal = fovea.grey(x,y);
   Point edge = fovea.edge(x,y);

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
}

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

**///hello
