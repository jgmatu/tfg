/*
 * OctoMap.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: javi
 */

#include "../include/OctoMap.h"


OctoMap::OctoMap() :
	octree_(0.02)
{
	init_octree(octree_);
}

void
OctoMap::init_octree(octomap::ColorOcTree& octree)
{
	octree.setOccupancyThres(0.5);
	octree.setProbHit(0.7);
	octree.setProbMiss(0.3);
	octree.setClampingThresMin(0.2);
	octree.setClampingThresMax(0.95);
}

OctoMap::~OctoMap()
{
	octree_.clear();
}


octomap::ColorOcTreeNode::Color
OctoMap::tag_cluster_in_pcl(const std::string& category)
{
	HSLTool hsl = HSLTool();
	Categories categories = Categories();

	return hsl.get_color(categories.get_category(category));
}

void
OctoMap::insert_plob(const Pblob* plob)
{
	pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
	std::string category = plob->get_box().Class;
	boost::algorithm::to_upper(category);

	for(it = plob->cloud()->begin(); it != plob->cloud()->end(); it++) {
		octomap::point3d point(it->x, it->y, it->z);

		octomap::OcTreeKey key;
		if (octree_.coordToKeyChecked(point, key)) {
			octree_.updateNode(key, true)->setColor(tag_cluster_in_pcl(category));
			octree_.updateNode(key, true)->setValue(plob->get_box().probability);
		}
	}
}

void
OctoMap::insert_plobs(const std::map<std::string, Pblob*> plobs)
{
	std::map<std::string, Pblob*>::const_iterator it;

	for (it = plobs.begin(); it != plobs.end(); it++) {
		if (it->second->is_likely()) {
			insert_plob(it->second);
		}
	}
}

void
OctoMap::publish_octomap(ros::Publisher octo_pub, const std::string& robot_frame) const
{
	octomap_msgs::Octomap map;

	if (octomap_msgs::fullMapToMsg(octree_, map)) {
		map.header.frame_id = robot_frame;
		map.header.stamp = ros::Time::now();

		octo_pub.publish(map);

		ROS_INFO("OctoMap published...");
	} else {
		ROS_ERROR("Error serializing OctoMap...");
	}
}
