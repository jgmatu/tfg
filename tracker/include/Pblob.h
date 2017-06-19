/*
 * Pblob.h
 *
 *  Created on: Jun 3, 2017
 *      Author: javi
 */

#ifndef PBLOB_H_
#define PBLOB_H_

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <opencv2/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>


#include "../include/HSVHistogram.h"
#include "../include/Categories.h"

class Pblob {

public:

	const float MINPROB = 0.6;

	Pblob(const Pblob*);
	Pblob(darknet_ros_msgs::BoundingBox);
	virtual ~Pblob();

	cv::Point*
	get_center_point() const;

	cv::Point*
	get_init_point_to_cluster(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr);

	void
	fill_out_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb, cv::Point* p);

	void
	set_location();


	darknet_ros_msgs::BoundingBox
	get_box() const { return darknet_box_; }

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	cloud() const {return points_;};

	bool
	is_likely() const {return probability_ > MINPROB; };

	void
	merge_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcrgb_out) const;

	void
	publishPointCloud(ros::Publisher, std::string);

	float
	similiarity(const Pblob* pblob) const;

	bool
	isDefined() const;

	bool
	is_size_object() const;

	int
	size() const;

	void
	print() const;

	bool
	isVisible(std::vector<pcl::PointXYZRGB> limits);

	/**
	 * The plob, is not in the image I remember
	 * it was there.
	 */
	void hidden();

	/**
	 * The element is not in the location I think
	 * it was.
	 */
	void dissapear();

	/**
	 * I have seen this plob now.
	 */
	void visible(const Pblob*);

private:

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_;
	darknet_ros_msgs::BoundingBox darknet_box_;
	HSVHistogram hsv_;
	float probability_;
	std::vector<float> location_; // location from odom in x y z for octomap... // Mean location...

	float
	get_distance (pcl::PointXYZRGB src, pcl::PointXYZRGB dest);

	bool
	is_marked(pcl::PointXYZRGB point);

	void
	mark_point(pcl::PointXYZRGB& point);

	bool
	is_limit_box(cv::Point*);

	bool
	is_limit_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr, cv::Point*);

	void
	add_point(pcl::PointXYZRGB& point);

	cv::Point*
	getMinPoint(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgb);

	cv::Point*
	getMaxPoint(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgb);

	int
	get_idx_pcl(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgb, int x , int  y);

	void
	print_dark() const;

	void
	print_location() const;

	void
	update(const Pblob&);

	void
	increment_visible();

	void
	decrement_dissapear();

	void
	decrement_hidden();

	void
	update(const Pblob*);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	copy_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

};

#endif /* PBLOB_H_ */
