#include "chrono"
#include "yolov8.hpp"
#include "opencv2/opencv.hpp"
#include "BYTETracker.h"
#include "STrack.h"
using namespace std;

int main(int argc, char** argv)
{
	cudaSetDevice(0);
	const std::string engine_file_path{ argv[1] };
	const std::string path{ argv[2] };
	std::vector<std::string> imagePathList;
	bool isReset{false};
	auto yolov8 = new YOLOv8(engine_file_path);
	auto tracker = new BYTETracker(15,500);
	yolov8->make_pipe(true);

	cv::Mat res, image;
	cv::Size size = cv::Size{640, 640};
	std::vector<Object> objs;	
	int num_frames = 0;
	int total_ms = 0;
	int fps;

	cv::namedWindow("Tracker", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture cap(path);
	if (!cap.isOpened())
	{
		printf("can not open %s\n", path.c_str());
		return -1;
	}

	while (cap.read(image))
	{
		auto trackerstart1 = std::chrono::system_clock::now();
		printf("=======run step=======\n");
		num_frames ++;
		objs.clear();
		yolov8->copy_from_Mat(image, size);
		// infer
		auto start = std::chrono::system_clock::now();
		yolov8->infer();
		auto end = std::chrono::system_clock::now();
        printf("infer time cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(end - start).count()/ 1000.);
		yolov8->postprocess(objs);			
		printf("reset tracker flag is : %d\n", isReset);
		// track
		auto trackerstart = std::chrono::system_clock::now();
		vector<STrack> output_stracks = tracker->update(objs, isReset);
		auto trackerend = std::chrono::system_clock::now();
        printf("tracker cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(trackerend - trackerstart).count()/ 1000.);

		for (int i = 0; i < output_stracks.size(); i++)
		{
			vector<float> tlwh = output_stracks[i].tlwh;
			bool vertical = tlwh[2] / tlwh[3] > 1.6; // 横纵比
			int id = output_stracks[i].track_id;
			if (id != 1 ){
				isReset =true;
			}
			Scalar color = tracker->get_color(id);
			cv::putText(image, cv::format("%d", id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, color, 2, LINE_AA);
			cv::rectangle(image, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), color, 2);	
		}
		cv::imshow("Tracker", image);
		if (cv::waitKey(10) == 'q')
		{
			break;
		}
	}
	cv::destroyAllWindows();
	delete yolov8;
	delete tracker;
	return 0;
}
