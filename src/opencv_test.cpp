#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <laneDetection.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <calibration.h>


using namespace cv;
using namespace std;

#include <image_transport/image_transport.h>


Mat videoFrame; // Video Frame.
Mat videoFrameUndistorted; // Video Frame (after calibration).
Mat videoFramePerspective; // Video Frame (after perspective transform).
Mat _videoFrameUndistorted;
Mat debugWindow(540, 1280, CV_8UC3, Scalar(0,0,0)); //The Debug window.
Size videoSize; // Input Variable Size.
Mat cameraMatrix, dist; //Calibration Matrix.
Mat perspectiveMatrix; //Homography Matrix.
Mat finalResult;
String coordinatetext = "";
Point2f perspectiveSrc[] = {Point2f(565,470), Point2f(721,470), Point2f(277,698), Point2f(1142,698)};
Point2f perspectiveDst[] = {Point2f(300,0), Point2f(980,0), Point2f(300,720), Point2f(980,720)};
stringstream ss;
float laneDistant = 0;


class OpenCVWebCam {
	public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher pub;
    bool image_captured;
    laneDetection LaneAlgo;

    //This variable stores the current frame


    OpenCVWebCam() :it_(nh_) {


        image_captured = false;


        //Start Homography

        image_sub = it_.subscribe("/jackal/camera/rgb/image_raw", 1, &OpenCVWebCam::imageCallback, this, image_transport::TransportHints("compressed"));

        //laneDetection LaneAlgo();

        //Applying lane detection algorithm


    }
		~OpenCVWebCam() {

		}


    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
            videoFrame = cv_bridge::toCvShare(msg, "bgr8")->image;
            //undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
            image_captured = true;
            cv::waitKey(30);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }




    }

};

int main(int argc, char** argv) {
	// set up ros
	ros::init(argc, argv, "opencv_test");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);

    //Get the Perspective Matrix.
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);

    //--------------Camera Calibration Start-----------------
    FileStorage fsRead;
    fsRead.open("Intrinsic.xml", FileStorage::READ);
    Mat src = imread("./camera_cal/calibration2.jpg");
    Mat dst;
    if (fsRead.isOpened() == false)
    {
        CameraCalibrator myCameraCalibrator;
        myCameraCalibrator.doCalibration(cameraMatrix, dist);
        FileStorage fs;
        fs.open("Intrinsic.xml", FileStorage::WRITE);
        fs << "CameraMatrix" << cameraMatrix;
        fs << "Dist" << dist;
        fs.release();
        fsRead.release();
        cout << "There is no existing intrinsic parameters XML file." << endl;
        cout << "Start calibraton......" << endl;
    }
    else
    {
        fsRead["CameraMatrix"] >> cameraMatrix;
        fsRead["Dist"] >> dist;
        fsRead.release();
    }
    //--------------Camera Calibration Finish-----------------

    OpenCVWebCam ros_pub_sub;
	ROS_INFO("Web");

    ros::spin();
//    if(ros_pub_sub.image_sub) {
//        laneDetection LaneAlgo(videoFrameUndistorted, perspectiveMatrix);
//        while (ros_pub_sub.image_captured) {
//            warpPerspective(videoFrameUndistorted, videoFrameUndistorted, perspectiveMatrix, videoSize);
//
//
//            LaneAlgo.laneDetctAlgo();
//
//            finalResult = LaneAlgo.getFinalResult();
//
//            //Detect the distance to lane center.
//            laneDistant = LaneAlgo.getLaneCenterDist();
//            if (laneDistant > 0) {
//                ss.str("");
//                ss.clear();
//                ss << abs(laneDistant) << "m " << " To the Right";
//                putText(finalResult, ss.str(), Point(50, 50), 0, 2, Scalar(0, 0, 255), 2);
//            } else {
//                ss.str("");
//                ss.clear();
//                ss << abs(laneDistant) << "m " << " To the Left";
//                putText(finalResult, ss.str(), Point(50, 50), 0, 2, Scalar(0, 0, 255), 2);
//            }
//
//            imshow("Real Time Execution", finalResult);
//            undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
//            _videoFrameUndistorted = videoFrameUndistorted.clone();
//            LaneAlgo.setInputImage(_videoFrameUndistorted);
//        }
//    }

//    else {
//
//
//
//    }
    cv::destroyWindow("view");
    return 0;
}
