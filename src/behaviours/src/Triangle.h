#ifndef TRIANGLE_H
#define TRIANGLE_H

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

  vector<Point> getVisitedPointList(){
      return this->visitedPointsList;
  }

private:
    std::vector<Point> visitedPointsList;
    int id;
    int state;
};


#endif