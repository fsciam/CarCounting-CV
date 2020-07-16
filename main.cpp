#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/bgsegm.hpp>
#include "vehicle.h"
#include "HungarianAlgorithm.h"

using namespace cv;
using namespace std;


int main(int argc, char ** argv)
{

	if(argc!=3)
	{
		cout<<"Usage is: "<<argv[0]<<" <video path> "<<endl;
		exit(-1);
	}
	namedWindow("Video",WINDOW_NORMAL);






	VideoCapture video_in(argv[1]);
	if(!video_in.isOpened())
	{
		cout<<"Impossible to open video: "<<argv[1]<<endl;
		exit(-1);
	}
	float IOUThresh=strtod(argv[2], NULL);

	Scalar colors[20];

  RNG rng;
  for(int i=0; i<20 ;i++)
  {
      colors[i]=cv::Scalar((int)rng.uniform(0,256),(int)rng.uniform(0,256),(int)rng.uniform(0,256));
  }
	int frame_height=video_in.get(CAP_PROP_FRAME_HEIGHT), frame_width=video_in.get(CAP_PROP_FRAME_WIDTH),frame_area=frame_height*frame_width;
	Point start(0,frame_height/2),end(frame_width,frame_height/2);
	Mat frame,mask,frame_smoothed,equalized;
	//create a background BackgroundSubtractor
  Ptr<BackgroundSubtractor> pBackSub=bgsegm::createBackgroundSubtractorCNT(int(video_in.get(CAP_PROP_FPS)/5), true,video_in.get(CAP_PROP_FPS)*60,true);
  Mat structuringElem = getStructuringElement(MORPH_RECT,Size( 7,7 ));
  int vehiclesCount=0;
	vector<Vehicle> vehicles;
	while(video_in.read(frame))
	{

    //preprocess image
		bilateralFilter(frame,frame_smoothed,9,150,150,BORDER_DEFAULT);

		//update the background model with a choosen learning rate
		pBackSub->apply(frame_smoothed, mask);

    morphologyEx( mask, mask, MORPH_OPEN,structuringElem);
    dilate(mask,mask, Mat(),Point(-1,-1),3);
		//Find the contours of the moving objects
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(mask, contours,hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

    //Find boundig boxes
		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		std::vector<cv::Rect> detections;
		for( size_t i = 0; i < contours.size(); i++ )
		{
				approxPolyDP( contours[i], contours_poly[i], 7, true );
				Rect detection= boundingRect( contours_poly[i] );
				if(detection.area()>3000 && detection.area()<frame_area)
					{
						detections.push_back(detection);
						//rectangle(frame,detection,Scalar(0,0,255),4);

					}
		}
		if(detections.size())
		{
			if(vehicles.size())
			{
				vector<pair<int,int>> updatePairs=makeBoundingBoxesPair(vehicles, detections, IOUThresh);
				int trackUpdated[vehicles.size()]={}, detectionUsed[detections.size()]={};
				for(auto p:updatePairs)
				{
					vehicles[p.first].update(detections[p.second]);
					trackUpdated[p.first]=1;
					detectionUsed[p.second]=1;
				}
				for(int i=vehicles.size();i>=0;i--)
				{
					if(!trackUpdated[i])
						vehicles.erase(vehicles.begin()+i);

				}
				for(int i=0;i<detections.size();i++)
				{
					if(!detectionUsed[i])
						vehicles.push_back(Vehicle(detections[i]));

				}
			}
			else
			{
				for(Rect r : detections)
					vehicles.push_back(Vehicle(r));
			}
		}

		for(Vehicle v:vehicles)
		{
			if(v.hasCrossedLine(start,end)) vehiclesCount++;

			int id= v.getId();
			Rect boundingBox=v.getBoundingBox();
			cv::Scalar color=colors[id%20];

      rectangle(frame,v.getBoundingBox(),color,4);
      putText(frame,to_string(id),cv::Point(boundingBox.x,boundingBox.y+15),cv::FONT_HERSHEY_SIMPLEX,1,color,3);

		}
    //draw line
    line(frame,start,end,Scalar(0,0,255),3);
    //print vehicles on screen
    cv::putText(frame,to_string(vehiclesCount),cv::Point(frame.size().width-40,40),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),4);

		imshow("Video",frame);



		if(waitKey(1)==27)
			break;

	}

	video_in.release();
	destroyAllWindows();
	return 0;
}
