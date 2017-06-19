/*
 * node_tracker.cpp
 *
 *  Created on: May 31, 2017
 *      Author: javi
 */

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "../include/OctoMap.h"
#include "../include/Pblob.h"
#include "../include/MapPlobs.h"
#include "../include/OctoMaps.h"
#include "../include/Strings.h"

const float THRESHOLD = 0.7;

class Tracker {

public:

	Tracker() :
		nh_(),
		octomaps_(),
		robot_frame_("camera_link"),
		instants_(),
		permanents_(),
		boxes_()
	{
		darknet_sub_ = nh_.subscribe("/darknet_ros/YOLO_BoundingBoxes", 1, &Tracker::darknet_cb, this);
		cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &Tracker::cloud_cb, this);

		cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered/darkNet", 0, true);
		set_octomaps_publishers();
	}

	~Tracker()
	{
		;
	}

	void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcf(new pcl::PointCloud<pcl::PointXYZRGB>);
		tf::StampedTransform l2r;

		try {
			tfListener_.lookupTransform(cloud_in->header.frame_id, robot_frame_, ros::Time(0), l2r);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s" , ex.what());
			return;
		}

		pcl::fromROSMsg(*cloud_in, *pcrgb);
		try {
			pcl_ros::transformPointCloud(robot_frame_, *pcrgb, *pcf, tfListener_);
		} catch(tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
			return;
		}

		instants_.set(pcf, boxes_);
		storage_permanents(get_limits(pcf));

		octomaps_.refresh(permanents_);
		octomaps_.publish(octo_pubs_, cloud_in->header.frame_id);

		instants_.clear();
		boxes_.clear();

		ROS_WARN("*** PERMANENTS ***");
		permanents_.print();
		ROS_WARN("*** PERMANENTS ***");
	}

	void darknet_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
	{
		std::vector<darknet_ros_msgs::BoundingBox>::const_iterator it;

		for (it = msg->boundingBoxes.begin(); it != msg->boundingBoxes.end(); it++) {
			boxes_.push_back(*it);
		}
	}

	std::vector<pcl::PointXYZRGB>
	get_limits(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcf)
	{
		std::vector<pcl::PointXYZRGB> points;

		return points;
	}

	void
	storage_permanents(std::vector<pcl::PointXYZRGB> limits)
	{
		if(instants_.empty()) {
			// No plobs detected in this callback from CNN.
			permanents_.set_all_hidden();
			return;
		}

		if(permanents_.empty()) {
			// Permanents is empty, storage all instants plobs detected
			// by the CNN.
			permanents_.insert_all(instants_);
			return;
		}

		// Default behavior. Storage plobs, that no have any plob on this category,
		// and add or update plobs of one category that we have already added.

		std::vector<std::string> categories;
		std::vector<std::string>::const_iterator it;

		categories = instants_.get_categories();
		for (it = categories.begin(); it != categories.end(); it++) {
			if (permanents_.find_category(*it)) {
				insert_by_features(*it, limits);
			} else {
				insert_all_category(*it);
			}
		}
	}

	void
	insert_all_category(std::string category)
	{
		std::map<std::string, Pblob*> category_plob = instants_.get_plobs(category);

		std::map<std::string, Pblob*>::iterator it;
		for (it = category_plob.begin(); it != category_plob.end(); it++) {
			permanents_.insert(it->second);
		}
	}

	void
	insert_by_features(std::string category, std::vector<pcl::PointXYZRGB> limits)
	{
		std::map<std::string, Pblob*> instants;
		std::map<std::string, Pblob*> permanents;

		instants = instants_.get_plobs(category);
		permanents = permanents_.get_plobs(category);

		std::map<std::string, std::map<std::string, Pblob*>> results;

		results = similarity(instants, permanents);

		permanents_.update(results["update"], limits);
		permanents_.add(results["add"]);
	}

	std::map<std::string, std::map<std::string, Pblob*>>
	similarity(const std::map<std::string, Pblob*>& instants, const std::map<std::string, Pblob*>& permanents) const
	{
		std::map<std::string, Pblob*> update;
		std::map<std::string, Pblob*> add;

		std::map<std::string, Pblob*>::const_iterator itI;

		for (itI = instants.begin(); itI != instants.end(); itI++) {
			float max = std::numeric_limits<float>::min();

			std::map<std::string, Pblob*>::const_iterator itJ;
			std::string key_update = "?";

			for (itJ = permanents.begin(); itJ != permanents.end(); itJ++) {
				float sim = itJ->second->similiarity(itI->second);

				if (sim > max) {
					max = sim;
					key_update = itJ->first;
				}
			}
			if (max <= 0.0 || max > 1.01) {
				continue;
			}
			if (max > THRESHOLD) {
				update[key_update] = itI->second;
			} else {
				add[itI->first] = itI->second;
			}
		}
		std::map<std::string, std::map<std::string, Pblob*>> result;

		result["add"] = add;
		result["update"] = update;
		return result;
	}


	void set_octomaps_publishers()
	{
		Categories cates;
		std::vector<std::string> categories = cates.get_categories();

		std::vector<std::string>::iterator it;
		for (it = categories.begin(); it != categories.end(); it++) {
			Strings* s = new Strings(*it);
			char topic_id[256];
			std::string key = s->get_key(*it);

			sprintf(topic_id, "/octomap/%s", key.c_str());
			octo_pubs_[key] = (nh_.advertise<octomap_msgs::Octomap>(topic_id, 0, true));
			delete(s);
		}
	}

	void
	print_results(const std::map<std::string, std::map<std::string, Pblob*>>& results)
	{
		std::map<std::string, std::map<std::string, Pblob*>>::const_iterator it;


		ROS_ERROR(" **** TRACE RESULTS **** ");
		for (it = results.begin(); it != results.end(); it++) {

			ROS_ERROR("*** KEY : %s ***", it->first.c_str());
			std::map<std::string, Pblob*>::const_iterator itJ;
			for (itJ = it->second.begin(); itJ != it->second.end(); itJ++){
				itJ->second->print();
			}
		}
	}

	void
	print_boxes()
	{
		std::vector<darknet_ros_msgs::BoundingBox>::const_iterator it;

		for (it = boxes_.begin(); it != boxes_.end(); it++) {
			fprintf(stderr, "Box class : %s\n", it->Class.c_str());
		}
	}

private:

	ros::NodeHandle nh_;

	ros::Subscriber darknet_sub_;
	ros::Subscriber cloud_sub_;
	ros::Publisher cloud_pub_;

	std::map<std::string, ros::Publisher> octo_pubs_;

	tf::TransformListener tfListener_;

	MapPlobs permanents_;
	MapPlobs instants_;

	OctoMaps octomaps_;

	std::vector<darknet_ros_msgs::BoundingBox> boxes_;
	std::string robot_frame_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tracker");
	Tracker tracker;

	ros::Rate loop_rate(120);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
