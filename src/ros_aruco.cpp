#include <aruco/aruco.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "ros_aruco/Markers.h"

using namespace aruco;

void imageCallback(ros::NodeHandle & nh, ros::Publisher & pub, MarkerDetector & detector, CameraParameters & param, bool & feedback, const sensor_msgs::ImageConstPtr & img)
{
    try
    {
        /* Get the image and setup the detector */
        //This is an empty vector of markers. 
        //like an array but can grow dynamically.
        std::vector<Marker> markers;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        if(param.CamSize != cv_ptr->image.size()) { param.resize(cv_ptr->image.size()); }
        double marker_size = 0.04;
        nh.getParam(ros::this_node::getName() + "/markersize", marker_size);

        bool rfeedback = true;
        nh.getParam(ros::this_node::getName() + "/feedback", rfeedback);
        if(feedback != rfeedback)
        {
            feedback = rfeedback;
            if(feedback)
            {
                cvNamedWindow(ros::this_node::getName().c_str());
                cvStartWindowThread();   
            }
            else
            {
                cvDestroyAllWindows();
            }
        }


        detector.detect( cv_ptr->image, markers, param, marker_size );
        ros_aruco::MarkersPtr msg(new ros_aruco::Markers);
        msg->header = img->header;
        msg->count = markers.size();
        msg->ids.resize(markers.size());
        msg->T.resize(3*markers.size());
        msg->R.resize(3*markers.size());
        msg->px.resize(2*markers.size());
        for(unsigned int i = 0; i < markers.size(); ++i)
        {
            msg->ids[i] = markers[i].id;
            msg->T[3*i] = markers[i].Tvec.at<float>(0,0);
            msg->T[3*i+1] = markers[i].Tvec.at<float>(0,1);
            msg->T[3*i+2] = markers[i].Tvec.at<float>(0,2);
            msg->R[3*i] = markers[i].Rvec.at<float>(0,0);
            msg->R[3*i+1] = markers[i].Rvec.at<float>(0,1);
            msg->R[3*i+2] = markers[i].Rvec.at<float>(0,2);
            cv::Point2f center = markers[i].getCenter();
            msg->px[2*i] = center.x;
            msg->px[2*i+1] = center.y;
            if(feedback)
            {
                markers[i].draw(cv_ptr->image, cv::Scalar(0, 0, 255), 2);
                CvDrawingUtils::draw3dCube(cv_ptr->image, markers[i], param);
            }
        }
        pub.publish(msg);

        if(feedback)
        {
            if(img->encoding == "rgb8")
            {
                cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
            }
            cv::imshow(ros::this_node::getName(), cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception & e)
    {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, ros::this_node::getName());

    ros::NodeHandle nh;
    
    std::string camparam;
    nh.getParam(ros::this_node::getName() + "/camparam", camparam);

    bool feedback = true;
    nh.getParam(ros::this_node::getName() + "/feedback", feedback);

    /* Initialize aruco tracker */
    CameraParameters CamParam;
    CamParam.readFromXMLFile(camparam.c_str());
    MarkerDetector MDetector;

    /* Initialize image transport, use remapped image */
    image_transport::ImageTransport it(nh);
    ros::Publisher pub = nh.advertise<ros_aruco::Markers>(ros::this_node::getName() + "/markers", 10);
    image_transport::Subscriber sub = it.subscribe("image", 1, boost::bind(&imageCallback, boost::ref(nh), boost::ref(pub), boost::ref(MDetector), boost::ref(CamParam),boost::ref(feedback), _1));

    /* An OpenCV window for feedback */
    if(feedback)
    {
        cvNamedWindow(ros::this_node::getName().c_str());
        cvStartWindowThread();   
    }

    /* Start spinning */
    ros::spin();

    return 0;
}
