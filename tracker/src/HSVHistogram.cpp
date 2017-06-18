/*
 * HSVHistogram.cpp
 *
 *  Created on: 28/04/2016
 *      Author: Paco
 */

#include "../include/HSVHistogram.h"

/*
 * Create a new histogram initialize to 0. :).
 */
HSVHistogram::HSVHistogram()
: npoints(0)
{
	inizialize();
}

/*
 * Create a histogram from one histogram.
 */
HSVHistogram::HSVHistogram(const HSVHistogram& other)
: npoints(other.npoints)
{
	memcpy(HS, other.HS, SIZEH * SIZES * sizeof(int));
	memcpy(HSPROB , other.HSPROB , SIZEH * SIZES * sizeof(float));
}

HSVHistogram::~HSVHistogram()
{
	;
}

/*
 * Copy the numbers of points whose represent the histogram
 * and copy the histogram matrix 20*10 in other site of memory
 * like a new histogram.
 */

HSVHistogram&
HSVHistogram::operator=(const HSVHistogram& other)
{
	inizialize();

	npoints = other.npoints;
	memcpy(HS, other.HS, SIZEH*SIZES*sizeof(int));
	memcpy(HSPROB , other.HSPROB , SIZEH*SIZES*sizeof(float));

	setProbs();
	return *this;
}

void
HSVHistogram::inizialize()
{
	for(int i = 0; i < SIZEH; i++) {
		for(int j = 0; j < SIZES; j++) {
			HS[i][j] = 0;
			HSPROB[i][j] = 0.0;
		}
	}
}

void
HSVHistogram::setProbs()
{
	int p = 0;

	for(int i = 0; i < SIZEH; i++) {
		for(int j = 0; j < SIZES; j++) {
			p += HS[i][j];
		}
	}
	for(int i = 0; i < SIZEH; i++) {
		for(int j = 0; j < SIZES; j++) {
			if (p != 0) {
				HSPROB[i][j] =  static_cast<float> (HS[i][j]) / float(p);
			}
		}
	}
	npoints = p;
}

/*
 * Add a new point to the histogram with a point cloud
 * in RGB parameters.
 */
void
HSVHistogram::add(pcl::PointXYZRGB& pointcolor)
{
	pcl::PointXYZHSV pointcolorhsv;

	PointXYZRGBtoXYZHSV(pointcolor, pointcolorhsv);
	add(pointcolorhsv);
}

void
HSVHistogram::add(pcl::PointXYZHSV& pointcolor)
{
	if(pointcolor.v < MINVALIDV || pointcolor.v > MAXVALIDV || pointcolor.s < MINVALIDS) {
		return;
	}
	int idxH, idxS;

	idxH = pointcolor.h/(MAXH/SIZEH);  // 360.0 / 20.0 18.0 -> Range in 0 .. 20 of hue... steps... :).
	idxS = pointcolor.s/(MAXS/SIZES);  // 1 / 10.0 -> Range 0 .. 10 of saturation... steps... :).

	HS[idxH][idxS]++;
	npoints++;
	setProbs(); // SetProbs HSVNormalized per point.
}


void
HSVHistogram::add(const HSVHistogram& histogram)
{
	for(int i = 0; i < SIZEH; i++) {
		for(int j = 0; j < SIZES; j++) {
			HS[i][j] += histogram.getHS(i , j);
		}
	}
	npoints += histogram.getNumPoints();
	setProbs(); // Set probs to all the HSPROB in the added histogram
				// to my actual histogram.
}



/*
 * Compare to histogram this and other to see if there are similarities
 * in the two histograms.
 */
float
HSVHistogram::similarity(const HSVHistogram& histogram) const
{
	float sim = 0.0;

	for (int i = 0; i < SIZEH; i++) {
		for (int j = 0; j < SIZES; j++) {
			sim += sqrt(HSPROB[i][j] * histogram.getHSPROB(i , j));
		}
	}
	return sim;
}

void
HSVHistogram::printHSVScale() const
{
	for(int i = 0; i < SIZEH; i++) {
		fprintf(stderr, "[%3.0lf] ", i*MAXH/SIZEH);
		for(int j = 0; j < SIZES; j++) {
			fprintf(stderr, "%10d\t", HS[i][j]);
		}
		fprintf(stderr, "%s", "\n");
	}
}

void
HSVHistogram::printHSVNorm() const
{
	float total = 0.0;

	for(int i = 0; i < SIZEH; i++) {
		fprintf(stderr, "[%3.0lf] ",  i*MAXH/SIZEH);
		for(int j = 0; j < SIZES; j++) {
			fprintf(stderr, "%3.9lf\t", HSPROB[i][j]);
			total += HSPROB[i][j];
		}
		fprintf(stderr, "%s", "\n");
	}
	fprintf(stderr , "==========================\n");
	fprintf(stderr , "Total : %3.3lf\n" , total);
	fprintf(stderr , "==========================\n");
}

void
HSVHistogram::print() const
{
	fprintf(stderr, "\n\n%s\n\n", "--- HISTOGRAM ---");
	fprintf(stderr , "HSV Num points : %10d\n" , npoints);

	this->printHSVScale();
	fprintf(stderr, "%s" , "\n\n\n\n");
	this->printHSVNorm();
}
