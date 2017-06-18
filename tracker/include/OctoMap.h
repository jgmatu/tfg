/*
 * OctoMap.h
 *
 *  Created on: Jun 7, 2017
 *      Author: javi
 */

#ifndef OCTOMAP_H_
#define OCTOMAP_H_

#include <ros/ros.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include "../include/MapPlobs.h"
#include "../include/HSLTool.h"
#include "../include/Categories.h"

class OctoMap {

public:

	OctoMap();
	virtual ~OctoMap();

	void
	insert_plobs(const std::map<std::string, Pblob*> pblobs);

	void
	publish_octomap(ros::Publisher, const std::string&) const;

private:

	const int BLACKWHITE = 2;
	const int STEP = 20;

	void
	init_octree(octomap::ColorOcTree&);

	void
	insert_plob(const Pblob*);

	void
	insert_plobs_category(const MapPlobs&, const std::string&);

	octomap::ColorOcTreeNode::Color
	tag_cluster_in_pcl(const std::string& category, float likely);

	octomap::ColorOcTreeNode::Color
	get_color(Categories::Category category);

	octomap::ColorOcTree octree_;
	octomap::ColorOcTreeNode::Color color_;
	std::string category_;
};

#endif /* OCTOMAP_H_ */
