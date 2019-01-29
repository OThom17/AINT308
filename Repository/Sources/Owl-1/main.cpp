// owl.cpp : Defines the entry point for the console application.
/* Phil Culverhouse Oct 2016 (c) Plymouth UNiversity
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demosntration programs does the following:
 * a) loop 1 - take picture, check arrow keys
 *             move servos +5 pwm units for each loop
 *             draw 64x64 pixel square overlaid on Right image
 *             if 'c' is pressed copy patch into a template for matching with left
 *              exit loop 1;
 * b) loop 2 - perform Normalised Cross Correlation between template and left image
 *             move Left eye to centre on best match with template
 *             (treats Right eye are dominate in this example).
 *             loop
 *             on exit by ESC key
 *                  go back to loop 1
 *
 * First start communcations on Pi by running 'python PFCpacket.py'
 * Then run this program. The Pi server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 * NOTE: this program is just a demonstrator, the right eye does not track, just the left.
 */

#include <iostream>
#include <fstream>
#include <ctime>
#include <sys/types.h>
//#include <unistd.h>

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include <iostream> // for standard I/O
#include <string>   // for strings


using namespace std;
using namespace cv;

// Chameleon definitions
#define SCAN 1
#define TRACK 0
void chameleonMode(int type, VideoCapture cap);
void Track(VideoCapture capture);

std::clock_t start;


int main(int argc, char *argv[])
{
    char receivedStr[1024];
    ostringstream CMDstream; // string packet
    string CMD;
    int N;
    Rx = RxLm; Lx = LxLm;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;
    string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
    string PiADDR = "10.0.0.10";
    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);

    /***********************
     * LOOP continuously for testing
     ***********************/
    // RyC=RyC-40; LyC=LyC+40; // offset for cross on card
    // Initialise and centre values
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    const Mat OWLresult;                // correlation result passed back from matchtemplate
    cv::Mat Frame;                      // Init Mat containers
    Mat Left, Right;                    // images
    bool inLOOP=true;                   // run through cursor control first, capture a target then exit loop
    bool keyLock = false;               // Locks the
    int key = 0;

    while (inLOOP){
        CMDstream.str("");              // move servos to centre of field
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        string RxPacket= OwlSendPacket (u_sock, CMD.c_str());
        VideoCapture cap (source);                                     // Capture new frame from the video source
        if (!cap.isOpened())                                           // Throw an error if initialisation fails
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }
        while (inLOOP)                                                  // WHILE loop manages the stereo camera movement
        {
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
            }
            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1);                 // Note that Left/Right are reversed now
            // Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left= FrameFlpd( Rect(0, 0, 640, 480));                     // using a rectangle
            Right=FrameFlpd( Rect(640, 0, 640, 480));                   // using a rectangle
            Mat RightCopy;
            Right.copyTo(RightCopy);
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 );  // draw white rect
            imshow("Left",Left);imshow("Right", RightCopy);
            waitKey(10);                                                // display the images
            if (!keyLock)                                               // Lock the most recently added character until the timer has elapsed
            {
                key = waitKey(0);                                       // this is a pause long enough to allow a stable photo to be taken.
            }
            switch (key)
            {
            case'w':                                                    //up arrow
                Ry=Ry+5;Ly=Ly+5;
                break;
            case's':                                                    //down arrow
                Ry=Ry-5;Ly=Ly-5;
                break;
            case'a':                                                    //left arrow
                Rx=Rx-5;Lx=Lx-5;
                break;
            case'd':                                                    // right arrow
                Rx=Rx+5;Lx=Lx+5;
                break;
            case 'c':                                                   // lowercase 'c'
                OWLtempl= Right(target);
                imshow("templ",OWLtempl);
                waitKey(1);
                inLOOP=false;                                           // quit loop and start tracking target
                break; // left
            case 'f':
                if (waitKey(0) == 'c')
                {
                    chameleonMode(TRACK, cap);                          // Updates the left eye to track. Right eye random
                }
                else
                {
                    chameleonMode(SCAN, cap);                           // Updates the eye positions randomly
                }
                if (!keyLock)
                {
                    start = clock();                                    // Take the current time
                    keyLock = true;                                     // Set the lock
                }
                else if ((( clock() - start ) / (double) CLOCKS_PER_SEC) >= 10)  // Loop for 10 seconds
                {
                    keyLock = false;                                    // Reset the lock
                }
                break;
            default:
                key=key;
                //nothing at present
            }
                // Update the positions but sending packet
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
            if (0)
            {
                    for (int i=0;i<10;i++){
                        Rx=Rx-50; Lx=Lx-50;
                        CMDstream.str("");
                        CMDstream.clear();
                        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                        CMD = CMDstream.str();
                        RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                        //waitKey(100); // cut the pause for a smooth persuit camera motion
                    }
                }
            }                                                           // END cursor control loop
            destroyAllWindows();                                        // close windows down

            // WHILE loop manages the tracking using the left eye
            inLOOP=false; // Commented out the tracking loop to implement revised version
            while (inLOOP)
            {
                if (!cap.read(Frame))
                {
                    cout  << "Could not open the input video: " << source << endl;
                    break;
                }
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
                if (waitKey(10)== 27) inLOOP=false;
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

                // move to get minimise distance from centre of both images, ie verge in to targe
                // move servos to position
                CMDstream.str("");
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());
            } // end if ZMCC

            // move to get minimise distance from centre of both images, ie verge in to target
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
        } // end while outer loop
#ifdef _WIN32
        closesocket(u_sock);
#else
        close(clientSock);
#endif
        exit(0); // exit here for servo testing only
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
        // Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
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
