#ifndef PUTIL_H
#define PUTIL_H
#include <iostream>
#include <fstream>
#include <nemo/Nemo.H>
#include "Particle.H"
 
// Compute distance from point to line
// #####################################################################
double distToEdge(int direction,Point2D<double>point){
	double distance = 0.0;
	if (direction == 0)
		distance = point.j;
	else if (direction == 1)
		distance = 360.0 - point.i;
        else if (direction == 2)
		distance = 360.0 - point.j;
	else if (direction == 3)
		distance = point.i;
  return distance;
 
}
// Compute probability given variance s,mean u
// #####################################################################
double getProbability(double u, double s, double x){
  double coefficient = 1.0/sqrt(2.0 * M_PI * s*s);
  return coefficient * exp(-(x-u)*(x-u)/(2.0 * s*s));
}
// Draw particle on an image
// #####################################################################
Image<PixRGB<byte> > drawParticle(Image<PixRGB<byte> > &img, std::vector<Particle> &points){
 
  for (int j=0; j<img.getHeight(); j++){
    for (int i=0; i<img.getWidth(); i++){
      img.setVal(i, j, PixRGB<byte>(255,255,255));
    }
  }
  for (uint s=0; s<points.size(); s++){
    if(img.coordsOk(points[s].getPosition()))
      img.setVal(points[s].getPosition(), PixRGB<byte>(255,0,0));
  }
  return img;
}
#endif
