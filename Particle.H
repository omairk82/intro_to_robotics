#ifndef PARTICLE_H
#define PARTICLE_H
 
#include <cmath>
#include <nemo/Nemo.H>
 
//#####################################################################
double randomDouble()
{
  return double(rand()) / (double(RAND_MAX) + 1.0);
}
// #####################################################################
double randomDoubleFromNormal(const double s)
{
  double sum = 0;
  for(int i=0; i<12; i++){
    sum += randomDouble()*2*s - s;
  }
  return sum/2;
}
// #####################################################################
class Particle{
 
  public:
 
    Particle(Point2D<int> start = Point2D<int>(0,0), double w = 0.0)
    {
 
      coord = start;
      weight = w;
    }
    ~Particle(){}
 
    // Moves the particle position with noise added to angle and distance
    void moveParticle(double angle, double distance, double variance){
 
      double realAngle   = angle + randomDoubleFromNormal(variance);
      double realDistance = distance + randomDoubleFromNormal(variance);
 
      coord.i = int(double(coord.i) + realDistance* cos(realAngle * M_PI/180.0));
      coord.j = int(double(coord.j) + realDistance* sin(realAngle * M_PI/180.0));
 
    }
 
    Point2D<int> getPosition(){return coord;}
    // Get Particle's probability
    double getWeight(){return weight;}
    // Modifies the Particle's probability
    void setWeight(double w){weight = w;}
 
  private:
    Point2D<int> coord; // coordinates of this particle
    double weight;  // probability that this particle represents correct location
 
};
#endif
