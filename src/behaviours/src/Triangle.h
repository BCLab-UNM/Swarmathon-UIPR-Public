#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Point.h"
#include "Controller.h"
class Triangle
{
public:
  void addVisitedPoint(Point point)
  {
      this->visitedPointsList.push_back(point);
  }
  void setAreaState(int areaState)
  {
      this->state = areaState;
  }
  
  int getId() const{
      return this->id;
  } 

  void setId(int id)
  {
      this->id = id;
  }
  
  vector<Point> getVisitedPointList() const{
      return this->visitedPointsList;
  }


private:
    std::vector<Point> visitedPointsList;
    int id;
    int state;
};


#endif