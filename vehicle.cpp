#include "vehicle.h"

int Vehicle::vehiclesCount=0;


Vehicle::Vehicle(cv::Rect rect):
boundingBox(rect),id(vehiclesCount++)
{
  centroid=cv::Point(boundingBox.x+(boundingBox.width/2),boundingBox.y+(boundingBox.height/2));
  oldCentroid=centroid;
}

cv::Rect Vehicle::getBoundingBox()
{
  return boundingBox;
}
int Vehicle::getId()
{
  return id;
}
double Vehicle::computeCost(cv::Rect new_rect)
{
  double inter=(boundingBox & new_rect).area();
  double uni=(boundingBox | new_rect).area();
  return 1-inter/uni;
}
void Vehicle::update(cv::Rect new_position)
{
  boundingBox=new_position;
  oldCentroid=centroid;
  centroid=cv::Point(boundingBox.x+(boundingBox.width/2),boundingBox.y+(boundingBox.height/2));
}
bool Vehicle::hasCrossedLine(cv::Point start, cv::Point end)
{
  bool isAbove=centroid.y>(((end.y-start.y)/(end.x-start.x))*(centroid.x-start.x)+start.y);
  bool wasAbove=oldCentroid.y>(((end.y-start.y)/(end.x-start.x))*(oldCentroid.x-start.x)+start.y);

  return (isAbove && !wasAbove) || (!isAbove && wasAbove);
}
