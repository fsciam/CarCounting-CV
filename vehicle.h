#ifndef VEHICLES_H
#define VEHICLES_H

#include <opencv2/opencv.hpp>



class Vehicle
{
  public:
    /**
    * Constructor
    */
    Vehicle(cv::Rect rect);
    /**
    * Obj getter
    * @return obj tracked
    */
    cv::Rect getBoundingBox();
    /**
    * Obj getter
    * @return id of the object
    */
    int getId();
    /**
    * Return the cost of "pair" obj and o
    * @param o object
    * @return cost of the assignment
    */
    double computeCost(cv::Rect new_rect);

    void update(cv::Rect new_position);
    bool hasCrossedLine(cv::Point start, cv::Point end);
  private:
    static int vehiclesCount;
    int id;
    cv::Rect boundingBox;
    cv::Point centroid;
    cv::Point oldCentroid;



};

#endif
