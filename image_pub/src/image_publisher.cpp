#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "chrono"
#include "thread"
#include "stb_image_write.h"
#include "DjiRtspImageSource.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION

static inline int64_t now()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static int write_data_to_file(const char* name, uint8_t* data, int size)
{
	FILE* fd = fopen(name, "wb");
	if(fd)
	{
		int w = (int)fwrite(data, 1, size, fd);
		fclose(fd);
		return w;
	}
	else
	{
		return -1;
	}
}

char rtsp_url[] = "rtsp://127.0.0.1:8554/live";
int main(int argc, char **argv) {
    if(argc < 1) return -1;
	if(argc == 1) 
	{
		std::cout << "Usage : " << argv[0] << " <url>" << std::endl;
	//	return -1;
	}
	int64_t ts = now();
	int64_t ts_ = now();
	int64_t fps_cnt = 0;

    // ros node
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle n;
	ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("camera/image", 1);

	// ros message
	sensor_msgs::Image image_msg;
	// image_msg.header.stamp = ros::Time::now(); // time stamp
    image_msg.header.frame_id = "camera_frame"; // frame id

    // start service
	DjiRtspImageSource service(rtsp_url);
	service.setImageCallback(nullptr, [&image_pub, &image_msg, &ts, &ts_, &fps_cnt](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {
		printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
		if(frmdata)
		{
			int64_t t = now();
			if(t - ts > 2000)
			{
				ts = t;
				char name[64];
				static int counter = 0;
				sprintf(name, "../pictures/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);

				// Convert frmdata to rgbData and cv::Mat
                uint8_t* rgbData = new uint8_t[width * height * 3];
				int result = DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
                cv::Mat rgbImage(height, width, CV_8UC3, rgbData);
                if (!rgbImage.data) {
                    ROS_ERROR("Could not open or find the image");
                    return;
                }

                // Convert the OpenCV image to a ROS sensor_msgs::Image message
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImage).toImageMsg();
                image_pub.publish(msg);	// Publish the message

				printf("Published success, fps:%f\n", (1000.0*(double)++fps_cnt) / ((double)t - (double)ts_));
				delete[] rgbData;
			}
		}
	});

	service.start();
    for(;;)
	// for(int i=0; i<30; i++)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	service.stop();
	std::cout << "done." << std::endl;
    return 0;
}
