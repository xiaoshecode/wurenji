#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <DjiRtspImageSource.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


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
int main(int argc, char** argv)
{
	if(argc < 1) return -1;
	if(argc == 1) 
	{
		std::cout << "Usage : " << argv[0] << " <url>" << std::endl;
	//	return -1;
	}
	int64_t ts = now();

	DjiRtspImageSource service(rtsp_url);
	service.setImageCallback(nullptr, [&ts](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {//最重要的回调函数
		printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);//打印图片信息
		if(frmdata)
		{
			int64_t t = now();
			if(t - ts > 1000)
			{
				ts = t;
				char name[64];
				static int counter = 0;
				sprintf(name, "pictures/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);//输出图片
				//if(pixfmt == 5) stbi_write_jpg(name, width, height, 3, frmdata, 80);
				uint8_t* rgbData = new uint8_t[width * height * 3];
				int result = DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
				printf("rtsp image source result is %d \r\n");
				stbi_write_jpg(name, width, height, 3, rgbData, 80);
				delete[] rgbData;
			}
		}
	});
	service.start();
	for(;;)//一直运行，可以修改
	//for(int i=0; i<30; i++)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	service.stop();
	std::cout << "done." << std::endl;
	return 0;
}
