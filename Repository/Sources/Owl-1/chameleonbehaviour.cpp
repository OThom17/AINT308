#include <math.h>   // Library for random number generation
#include <iostream>
#include <fstream>

#include <sys/types.h>

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

using namespace std;
using namespace cv;

#define SCAN 1
#define TRACK 0

void Track(VideoCapture cap)
{
    // Function updates the left eye's positions to track an object using cross-corrolation

    const Mat OWLresult;// correlation result passed back from matchtemplate
    cv::Mat Frame;
    Mat Left, Right; // images

    if (!cap.read(Frame))
    {
        cout  << "Could not open the input video" << endl;
    }
    else
    {
        Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
        //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
        Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle

        //Rect target= Rect(320-32, 240-32, 64, 64); //defined in owl-cv.h
        //Mat OWLtempl(Right, target);
        OwlCorrel OWL;
        OWL = Owl_matchTemplate( Right,  Left, OWLtempl, target);
        /// Show me what you got
        Mat RightCopy;
        Right.copyTo(RightCopy);
        rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 );
        rectangle( Left, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
        rectangle( OWL.Result, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

        imshow("Owl-L", Left);
        imshow("Owl-R", RightCopy);
        imshow("Correl",OWL.Result );
    // P control
        double KPx=0.1; // track rate X
        double KPy=0.1; // track rate Y
        double LxScaleV = LxRangeV/(double)640; //PWM range /pixel range
        double Xoff= 320-(OWL.Match.x + OWLtempl.cols)/LxScaleV ; // compare to centre of image
        int LxOld=Lx;

        Lx=LxOld-Xoff*KPx; // roughly 300 servo offset = 320 [pixel offset


        double LyScaleV = LyRangeV/(double)480; //PWM range /pixel range
        double Yoff= (250+(OWL.Match.y + OWLtempl.rows)/LyScaleV)*KPy ; // compare to centre of image
        int LyOld=Ly;
        Ly=LyOld-Yoff; // roughly 300 servo offset = 320 [pixel offset

        cout << Lx << " " << Xoff << " " << LxOld << endl;
        cout << Ly << " " << Yoff << " " << LyOld << endl;
        //Atrous

        //Maxima

        // Align cameras

        // ZMCC disparity map

        // ACTION
    }

}


void chameleonMode(int type, VideoCapture cap)
{
    /* This function will make the owl behave as a chameleon.
     * These animals can move their eyes individually.
     * This allows for them to scan the horizon while simultaneously watching for prey
     * Behaviour must take place for 10 seconds
     */
    // Ry, Rx, Ly, Lx, Neck Servo Controllers

    if (type == SCAN)
    {
        // Assign random numbers to the eye servo positons for both eyes while the tracking is disabled
        Rx = RxLm + (rand() % RxRangeM);
        Ry = RyBm + (rand() % RyRangeM);
        Lx = LxLm + (rand() % LxRangeM);
        Ly = LyBm + (rand() % LyRangeM);
    }
    else if (type == TRACK)
    {
        Track(cap);                             // Update the position of the left eye to track the object
        Rx = RxLm + (rand() % RxRangeM);        // Assign random position to the right hand eye
        Ry = RyBm + (rand() % RyRangeM);
    }

}
