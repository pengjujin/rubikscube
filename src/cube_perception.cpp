#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

void main(){
	Mat image = imread("../data/frame0000.jpg");
	namedWindow("test_window", WINDOW_AUTOSIZE);
	imshow("test_window", image);

	waitKey(0);
	return 0;
}