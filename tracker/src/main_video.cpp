#include "chrono"
#include "yolov8.hpp"
#include "opencv2/opencv.hpp"
#include "BYTETracker.h"
#include "STrack.h"

int main(int argc, char** argv)
{
	cudaSetDevice(0);
	const std::string engine_file_path{ argv[1] };
	const std::string path{ argv[2] };
	bool isReset{false};
	auto yolov8 = new YOLOv8(engine_file_path);
	auto tracker = new BYTETracker(15, 500);
	yolov8->make_pipe(true);

	cv::Mat img;
    cv::Size size = cv::Size{640, 640};
	std::vector<Object> objs;	

	cv::namedWindow("Tracker", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture cap(path);

	if (!cap.isOpened())
	{
		printf("ERROR: Can not open %s\n", path.c_str());
		return -1;
	}

	while (cap.read(img))
	{
        printf("=======run step=======\n");
		auto trackerstart1 = std::chrono::system_clock::now();
		objs.clear();
		yolov8->copy_from_Mat(img, size);
		// infer
		auto inferstart = std::chrono::system_clock::now();
		yolov8->infer();
		auto inferend = std::chrono::system_clock::now();
        printf("infer time cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(inferend - inferstart).count() / 1000.);
		yolov8->postprocess(objs);			
		printf("reset tracker flag is : %d\n", isReset);
		// track
		auto trackstart = std::chrono::system_clock::now();
		vector<STrack> output_stracks = tracker->update(objs, isReset);
		auto trackend = std::chrono::system_clock::now();
        printf("tracker cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(trackend - trackstart).count() / 1000.);

		for (int i = 0; i < output_stracks.size(); i++)
		{
			vector<float> tlwh = output_stracks[i].tlwh;
			bool vertical = tlwh[2] / tlwh[3] > 1.6; // 横纵比
			int id = output_stracks[i].track_id;
			if (id != 1){
				isReset = true;
			}
			Scalar color = tracker->get_color(id);
			cv::putText(img, cv::format("%d", id), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, color, 2, LINE_AA);
			cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), color, 2);	
		}
		cv::imshow("Tracker", img);
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
