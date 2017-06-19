/*
 * Pblob.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: javi
 */

#include "../include/Pblob.h"


const int RADIOUS = 1;
const float DISTANCE = 0.9;
const int MINSIZE = 10000;
const float INITPROB = 0.5;
const int DIMENSIONS = 3;

const int X = 0;
const int Y = 1;
const int Z = 2;

Pblob::Pblob(const Pblob* p) :
	points_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	darknet_box_ = p->darknet_box_;
	hsv_ = p->hsv_;
	pcl::copyPointCloud(*(p->points_), *points_);
	probability_ = p->probability_;
	location_ = p->location_;
}

Pblob::Pblob(darknet_ros_msgs::BoundingBox db) :
	points_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	darknet_box_ = db;
	hsv_ = HSVHistogram();
	probability_ = INITPROB;
	location_ = std::vector<float>(DIMENSIONS);
}

Pblob::~Pblob()
{
	points_->clear();
	location_.clear();
}

bool
Pblob::isVisible(std::vector<pcl::PointXYZRGB> limits)
{
	return true;
}

bool
Pblob::isDefined() const
{
	return points_->size() != 0;
}

int
Pblob::size() const
{
	return points_->size();
}

/**
 * The plob, is not in the image I remember
 * it was there.
 */
void
Pblob::hidden()
{
	decrement_hidden();
}

/**
 * The element is not in the location I think
 * it was.
 */
void
Pblob::dissapear()
{
	decrement_dissapear();
}

/**
 * I have seen this plob now.
 */
void
Pblob::visible(const Pblob* plob)
{
	update(plob);
	increment_visible();
}

void
Pblob::set_location()
{
	float xmean, ymean, zmean;
	int num = 0;

	xmean = ymean = zmean = 0.0f;
	pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
	for (it = points_->begin(); it != points_->end(); it++) {
		xmean += it->x;
		ymean += it->y;
		zmean += it->z;
		num++;
	}
	xmean /= num;
	ymean /= num;
	zmean /= num;

	location_[X] = xmean;
	location_[Y] = ymean;
	location_[Z] = zmean;
}

cv::Point*
Pblob::get_center_point() const
{
	return new cv::Point((darknet_box_.xmax + darknet_box_.xmin) / 2, (darknet_box_.ymax + darknet_box_.ymin) / 2);
}

bool
Pblob::is_size_object() const
{
	return points_->size() > MINSIZE;
}

void
Pblob::fill_out_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb, cv::Point* p)
{
	if(is_limit_cloud(pcrgb, p) || is_limit_box(p)) {
		return;
	}
	int posdata = get_idx_pcl(pcrgb, p->x , p->y);
	if (std::isnan(pcrgb->at(posdata).x) || is_marked(pcrgb->at(posdata))) {
		return;
	}
	add_point(pcrgb->at(posdata));
	mark_point(pcrgb->at(posdata));
	for (int i = -RADIOUS; i <= RADIOUS; i++) {
		for (int j = -RADIOUS; j <= RADIOUS; j++) {
			if (i == 0 && j == 0) {
				continue;
			}
			cv::Point* np = new cv::Point(p->x + i, p->y + j);

			if(is_limit_cloud(pcrgb, np)) {
				delete(np);
				continue;
			}
			int posdataN = get_idx_pcl(pcrgb, np->x, np->y);
			if (std::isnan(pcrgb->at(posdataN).x) || is_marked(pcrgb->at(posdataN))) {
				delete(np);
				continue;
			}
			float distance = get_distance(pcrgb->at(posdataN), pcrgb->at(posdata));
			if (distance < DISTANCE) {
				fill_out_object(pcrgb, np);
			}
			delete(np);
		}
	}
}

void
Pblob::merge_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcrgb_out) const
{
	pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;

	for (it = points_->begin(); it != points_->end(); it++) {
		pcrgb_out->push_back(*it);
	}
}

float
Pblob::similiarity(const Pblob* plob) const
{
	return hsv_.similarity(plob->hsv_);
}

void
Pblob::increment_visible()
{
	probability_ *= 2.1;

	if (probability_ > 1.0) {
		probability_ = 1.0;
	}
}

void
Pblob::decrement_hidden()
{
	probability_ *= 0.90;

	if (probability_ < 0.1) {
		probability_ = 0.1;
	}
}

void
Pblob::decrement_dissapear()
{
	probability_ *= 0.3;

	if (probability_ < 0.1) {
		probability_ = 0.1;
	}
}

void
Pblob::update(const Pblob* plob)
{
	points_->clear();

	darknet_box_ = plob->darknet_box_;
	location_ = plob->location_;
	hsv_= plob->hsv_;
	pcl::copyPointCloud(*(plob->points_), *points_);
}

void
Pblob::mark_point(pcl::PointXYZRGB& point)
{
	point.r = 255;
	point.g = 255;
	point.b = 255;
}

bool
Pblob::is_marked(pcl::PointXYZRGB point)
{
	return point.r == 255 && point.g == 255 && point.b == 255;
}

float
Pblob::get_distance (pcl::PointXYZRGB src, pcl::PointXYZRGB dest)
{
	return sqrt(pow(src.x - dest.x, 2) + pow(src.y - dest.y, 2) + pow(src.z - dest.z , 2));
}

bool
Pblob::is_limit_box(cv::Point* p)
{
	return p->x < darknet_box_.xmin || p->x > darknet_box_.xmax ||
			p->y < darknet_box_.ymin || p->y > darknet_box_.ymax;
}

void
Pblob::add_point(pcl::PointXYZRGB& point)
{
	points_->push_back(point);
	hsv_.add(point);
}

bool
Pblob::is_limit_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgb, cv::Point* p)
{
	return p->x < 0 || p->x >= pcrgb->width || p->y < 0 || p->y >= pcrgb->height ||
			get_idx_pcl(pcrgb, p->x, p->y) < 0 || get_idx_pcl(pcrgb, p->x, p->y) > pcrgb->size();
}

int
Pblob::get_idx_pcl(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgb, int x , int  y)
{
	return y * pcrgb->width + x;
}

void
Pblob::print_location() const
{
	fprintf(stderr, "Location (%f, %f, %f)\n", location_[X], location_[Y], location_[Z]);
}

void
Pblob::print_dark() const
{
	fprintf(stderr, "\n%s\n", "--- Darknet_box ---");

	fprintf(stderr, "Class : %s\n", darknet_box_.Class.c_str());
	fprintf(stderr, "Point min : (%ld, %ld)\n", darknet_box_.xmin, darknet_box_.ymin);
	fprintf(stderr, "Point max : (%ld, %ld)\n", darknet_box_.xmax, darknet_box_.ymax);
	fprintf(stderr, "Probability : %lf\n", darknet_box_.probability);
}

void
Pblob::print() const
{
	fprintf(stderr, "Num points : %ld\n",  points_->size());
	if (points_->size() == 0) {
		fprintf(stderr, "%s\n",  "Pblob empty!!!");
	}

	print_dark();
	print_location();

	fprintf(stderr, "Dims :%ld\n", points_->size());
	fprintf(stderr, "Probability Octomap : %f\n", probability_);
	// hsv_.print();
}
