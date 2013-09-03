#include <aruco/aruco.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace aruco;

void imageCallback(ros::NodeHandle & nh, MarkerDetector & detector, CameraParameters & param, bool & feedback, const sensor_msgs::ImageConstPtr & img)
{
    try
    {
        /* Get the image and setup the detector */
        std::vector<Marker> markers;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        if(param.CamSize != cv_ptr->image.size()) { param.resize(cv_ptr->image.size()); }
        double marker_size = 0.04;
        nh.getParam("ros_aruco/markersize", marker_size);

        bool rfeedback = true;
        nh.getParam("ros_aruco/feedback", rfeedback);
        if(feedback != rfeedback)
        {
            feedback = rfeedback;
            if(feedback)
            {
                cvNamedWindow("ros_aruco");
                cvStartWindowThread();   
            }
            else
            {
                cvDestroyAllWindows();
            }
        }

        detector.detect( cv_ptr->image, markers, param, marker_size );
        for(unsigned int i = 0; i < markers.size(); ++i)
        {
            if(feedback)
            {
                markers[i].draw(cv_ptr->image, cv::Scalar(0, 0, 255), 2);
                CvDrawingUtils::draw3dCube(cv_ptr->image, markers[i], param);
            }
        }

        if(feedback)
        {
            cv::imshow("ros_aruco", cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception & e)
    {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ros_aruco");

    ros::NodeHandle nh;
    
    std::string camparam;
    nh.getParam("ros_aruco/camparam", camparam);

    bool feedback = true;
    nh.getParam("ros_aruco/feedback", feedback);

    /* Initialize aruco tracker */
    CameraParameters CamParam;
    CamParam.readFromXMLFile(camparam.c_str());
    MarkerDetector MDetector;

    /* Initialize image transport, use remapped image */
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image", 1, boost::bind(&imageCallback, boost::ref(nh), boost::ref(MDetector), boost::ref(CamParam),boost::ref(feedback), _1));

    /* An OpenCV window for feedback */
    if(feedback)
    {
        cvNamedWindow("ros_aruco");
        cvStartWindowThread();   
    }

    /* Start spinning */
    ros::spin();

    return 0;
}
