/*
 * HSVHistogram.h
 *
 *  Created on: 28/04/2016
 *      Author: Paco
 */

#ifndef HSVHISTOGRAM_H_
#define HSVHISTOGRAM_H_

#include "ros/ros.h"

#include <list>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#define MAXH 360.0
#define MAXS 1.0
#define MAXV 1.0
#define MINVALIDV 0.1
#define MAXVALIDV 0.9
#define MINVALIDS 0.1

class HSVHistogram {
public:

	HSVHistogram();
	HSVHistogram(const HSVHistogram& other);
	virtual ~HSVHistogram();

	void add(pcl::PointXYZRGB& pointcolor);
	void add(pcl::PointXYZHSV& pointcolor);
	void add(const HSVHistogram& histogram);

	/*
	 * Return the probability the histogram you pass and the histogram
	 * actual are the same histogram.
	 */
	float similarity(const HSVHistogram& histogram) const;

	/*
	 * Copy the histogram in a new place in memory and
	 * return a pointer to the new histogram the old
	 * histogram is not touched.
	 */
	HSVHistogram& operator=(const HSVHistogram& other);

	// This method plus point to point two histograms.
	friend HSVHistogram operator+(HSVHistogram lhs, HSVHistogram rhs)
	{
		lhs.npoints += rhs.npoints;
		for(int i = 0; i < HSVHistogram::SIZEH; i++) {
			for(int j = 0; j < HSVHistogram::SIZES; j++) {
				lhs.HS[i][j] += rhs.getHS(i , j);
 			}
		}
		lhs.setProbs();
		return lhs;
	}

	// This method multiply the histogram to one scalar.. A = a*A
	friend HSVHistogram operator*(HSVHistogram lhs , float factor)
	{
		for(int i = 0; i < HSVHistogram::SIZEH; i++) {
			for(int j = 0; j < HSVHistogram::SIZES; j++) {
				lhs.HS[i][j] *= factor;
			}
		}
		lhs.setProbs();
		return lhs;
	}
	int getNumPoints() const { return npoints; };
	void print() const;

private:

	void inizialize();
	void setProbs();

	/*
	 * Number of points that represent a hue and saturation
	 * range.
	 */
	int getHS(int idH, int idS) const 		{ return HS[idH][idS];	  };
	float getHSPROB(int idH , int idS) const { return HSPROB[idH][idS]; };

	void printHSVScale() const;
	void printHSVNorm() const;

	static const int SIZEH = 20;
	static const int SIZES = 10;

	int HS[SIZEH][SIZES];
	float HSPROB[SIZEH][SIZES];
	int npoints;
};

#endif /* HSVHISTOGRAM_H_ */
