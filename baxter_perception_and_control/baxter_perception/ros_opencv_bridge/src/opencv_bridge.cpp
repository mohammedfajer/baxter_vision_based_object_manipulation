


/** ===========================================================================

    Copyright (C) 2019/2020 The University of Leeds and Mohammed Akram Fajer.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

 =========================================================================== */

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>


/* Author: Mohammed Fajer */

/**
  * This calss 'OpenCVBridge' is responsible for
  * converting ros simulated images to opencv images
  * and publishing the new converted opencv compatible image as a video
  * stream.
  */

class OpenCVBridge
{

    private:

      ros::NodeHandle node;

      image_transport::ImageTransport image_transport;

      image_transport::Subscriber image_subscriber;

      image_transport::Publisher image_publisher;

    public:

      OpenCVBridge() : image_transport( node )
      {

        image_subscriber = image_transport.subscribe("/camera/rgb/image_raw", 1, &OpenCVBridge::callback, this);

        image_publisher  = image_transport.advertise("/opencv_bridge/output_video", 1);
      }



      ~OpenCVBridge()
      {
        cv::destroyAllWindows();
      }

      void callback( const sensor_msgs::ImageConstPtr& msg )
      {

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
          cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );

        }
        catch( cv_bridge::Exception& e )
        {
          ROS_ERROR( "cv_bridge exception: %s", e.what() );
          return;
        }

        cv::imshow( "image", cv_ptr->image );
        cv::waitKey(3);

        image_publisher.publish( cv_ptr->toImageMsg() );

      }

};

int main( int argc, char** argv )
{

  ros::init( argc , argv , "opencv_bridge" );

  OpenCVBridge bridge;

  ros::spin();

  return 0;
}
