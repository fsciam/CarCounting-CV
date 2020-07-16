#ifndef HUNGARIAN_ALGORITHM_H
#define HUNGARIAN_ALGORITHM_H

#include <opencv2/opencv.hpp>


#include "vehicle.h"
/**
* makeBoundingBoxesPair uses Hungarian Algorithm to pair new dections and object tracked
* @param vehicles  vector that contains trackers
* @param newDetections vector that contains the object detected
* @param minIOU new detection threshold
* @return index pairs vehicles-detection
*/
std::vector<std::pair<int,int>> makeBoundingBoxesPair(std::vector<Vehicle> vehicles, std::vector<cv::Rect> newDetections, float minIOU)
{
  int originalRows,originalColumns;

  originalRows=vehicles.size();
  originalColumns=newDetections.size();

  int n=std::max(originalRows,originalColumns);

  cv::Mat_<double> costMatrix(n,n);


  //build cost matrix
  for(int i=0;i<n;i++)
  {

    for(int j = 0;j < n; j++)
    {
      if(i<originalRows && j<originalColumns)
        costMatrix(i,j)= vehicles[i].computeCost(newDetections[j]);
      else
        costMatrix(i,j)=0;

    }
  }
  cv::Mat_<double> maskMatrix= cv::Mat_<double>::zeros(n,n);
  int rowCover[n]={},colCover[n]={};
  int markedRows[n]={}, markedColumn[n]={};
  cv::Mat_<double> originalCostMatrix=costMatrix.clone();
  int step=1;

  while(step)
  {
    if(step==1)
    {
      //For each row, find the min cost and subract it from each cost in the row
      for(int i=0; i<n; i++)
      {
        const  double* row = costMatrix.ptr<double>(i);
        double  minRowElement=*std::min_element(row,row+n);
        for(int j=0;j<n;j++)
        {
          costMatrix(i,j)-=minRowElement;
        }
      }
      step=2;
    }
    else if(step==2)
    {
      //For each column, find the min cost and subract it from each cost in the column

      for(int j=0; j<n; j++)
      {
        double minColumnElement=1;
        for(int i=0;i<n;i++)
        {
          if(costMatrix(i,j)<minColumnElement)
          {
            minColumnElement=costMatrix(i,j);
          }
        }
        for(int i=0;i<n;i++)
        {
          costMatrix(i,j)-=minColumnElement;
        }
      }
      step=3;
    }
    else if(step==3)
    {
      //find the minimum number of lines to cover all the zero in the cost measurementMatrix
      for(int i=0;i<n;i++)
      {
        for(int j=0;j<n;j++)
        {
          if(costMatrix(i,j)==0)
          {
            if(rowCover[i] || colCover[j])
            {
              maskMatrix(i,j)=2;
            }
            else
            {
              maskMatrix(i,j)=1;
              rowCover[i]=1;
              colCover[j]=1;
            }
          }
          else
          {
            maskMatrix(i,j)=0;
          }
        }
      }

      std::vector<int> rowsToBeChecked;





      for(int i=0;i<n;i++)
      {
        if(!rowCover[i])
        {

            rowsToBeChecked.push_back(i);
            markedRows[i]=1;

        }
      }

      while(!rowsToBeChecked.empty())
      {
        int row=rowsToBeChecked.back();
        rowsToBeChecked.pop_back();
        for(int j=0;j<n;j++)
        {
          if(maskMatrix(row,j)>0)
          {

            markedColumn[j]=1;

            for(int z=0;z<n;z++)
            {
              if(maskMatrix(z,j)==1 && !markedRows[z])
              {

                markedRows[z]=1;
                rowsToBeChecked.push_back(z);
              }
            }
          }
        }

      }
      int lines=0;
      for(int i=0;i<n;i++)
      {
        if(colCover[i])  lines++;
        if(!rowCover[i]) lines++;
      }

      if(lines==n)
        step=0;
      else
        step=4;

    }
    else if(step==4)
    {
      //find the minimum uncovered elements, subtract it from all
      //the uncovered elements and add to the elements covered twice
      int minUncovered=1;
      for(int i=0; i<n;i++)
      {
        for(int j=0;j<n;j++)
        {
          if(markedRows[i] && !markedColumn[j])
          {
              if(costMatrix(i,j)<minUncovered)
                minUncovered=costMatrix(i,j);
          }
        }
      }

      for(int i=0; i<n;i++)
      {
        for(int j=0;j<n;j++)
        {

          if(markedRows[i] && !markedColumn[j])
          {

            costMatrix(i,j)-=minUncovered;
          }
          else if(!markedRows[i] && markedColumn[j])
          {

              costMatrix(i,j)+=minUncovered;
          }
        }
      }

      for(int i=0;i<n;i++)
      {
        markedRows[i]=0;
        markedColumn[i]=0;
        rowCover[i]=0;
        colCover[i]=0;
      }
      step=3;
    }
  }

  std::vector<std::pair<int,int>> pairs;
  int detectionUsed[n]={};
  int trackUsed[n]={};

  for(int i=0; i<originalRows;i++)
  {
    for(int j=0;j<originalColumns;j++)
    {
      if(costMatrix(i,j)==0 && originalCostMatrix(i,j)<minIOU)
      {
         pairs.push_back(std::make_pair(i,j));
         detectionUsed[j]=1;
         trackUsed[i]=1;
      }
    }
  }


  return pairs;
}

#endif
