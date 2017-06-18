/*
 * OctoMap.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: javi
 */

#include "../include/OctoMap.h"


const int RANGE_BRIGTNESS = 100;

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
OctoMap::get_color(Categories::Category category)
{
	HSLTool* hsltool = new HSLTool();
	std::vector<float> hsl(3);

	if (category == Categories::Category::AEROPLANE) {
		hsl[0] = 0;
		hsl[1] = 0;
		hsl[2] = 0;
	} else if (category == Categories::Category::BICYCLE) {
		hsl[0] = 0;
		hsl[1] = 0;
		hsl[2] = 1.0f;
	} else {
		hsl[0] = (static_cast<int>(category) - BLACKWHITE) * STEP;
		hsl[1] = 1.0f;
		hsl[2] = 0.5f;
	}
	std::vector<unsigned char> rgb = hsltool->HSL2RGB(hsl);
	return octomap::ColorOcTreeNode::Color(rgb[0], rgb[1], rgb[2]);
}


octomap::ColorOcTreeNode::Color
OctoMap::tag_cluster_in_pcl(const std::string& category, float likely)
{
	return octomap::ColorOcTreeNode::Color();
}

void
OctoMap::insert_plob(const Pblob* plob)
{
	pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
	std::string category = plob->get_box().Class;
	boost::algorithm::to_upper(category);
	float likely = plob->get_probability();

	for(it = plob->cloud()->begin(); it != plob->cloud()->end(); it++) {
		octomap::point3d point(it->x, it->y, it->z);

		octomap::OcTreeKey key;
		if (octree_.coordToKeyChecked(point, key)) {
			octree_.updateNode(key, true)->setColor(tag_cluster_in_pcl(category, likely));
			octree_.updateNode(key, true)->setValue(likely);
		}
	}
}

void
OctoMap::insert_plobs(const std::map<std::string, Pblob*> plobs)
{
	std::map<std::string, Pblob*>::const_iterator it;

	for (it = plobs.begin(); it != plobs.end(); it++) {
		insert_plob(it->second);
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
