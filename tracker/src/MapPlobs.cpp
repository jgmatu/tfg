/*
 * MapPlobs.cpp
 *
 *  Created on: Jun 10, 2017
 *      Author: javi
 */

#include "../include/MapPlobs.h"
#include <string>

MapPlobs::MapPlobs() :
	plobs_(),
	categories_()
{
	;
}

MapPlobs::~MapPlobs()
{
	clear();
}

void
MapPlobs::clear()
{
	std::map<std::string, Pblob*>::iterator it;

	for (it = plobs_.begin(); it != plobs_.end(); it++) {
		delete(it->second);
	}
	plobs_.clear();
	categories_.clear();
}

std::string
MapPlobs::get_new_key_plob(const std::string cateogory) const
{
	std::string num = std::to_string(categories_.at(cateogory));

	return cateogory + num;
}

int
MapPlobs::size() const
{
	std::map<std::string, int>::const_iterator it;
	int total = 0;

	for (it = categories_.begin(); it != categories_.end(); it++) {
		total += it->second;
	}
	return total;
}

bool
MapPlobs::empty() const
{
	return plobs_.empty();
}

void
MapPlobs::set(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb,
		const std::vector<darknet_ros_msgs::BoundingBox>& boxes)
{
	std::vector<darknet_ros_msgs::BoundingBox>::const_iterator it;

	for (it =  boxes.begin(); it != boxes.end(); it++) {
		Pblob *plob = new Pblob(*it);
		cv::Point* point = plob->get_center_point();

		if (std::isnan(point->x) || std::isnan(point->y)) {
			delete(point);
			continue;
		}
		plob->fill_out_object(pcrgb, point);
		delete(point);

		plob->set_location();
		if (plob->is_size_object()) {
			insert(plob);
		} else {
			delete(plob);
		}
	}
}

void
MapPlobs::set_all_hidden()
{
	std::map<std::string, Pblob*>::iterator it;

	for (it = plobs_.begin(); it != plobs_.end(); it++) {
		it->second->hidden();
	}
}

void
MapPlobs::publish(ros::Publisher cloud_pub, std::string frame_id)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out = get_cloud();

	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*pcrgb_out, cloud_out);

	cloud_out.header.frame_id = frame_id;
	cloud_out.header.stamp = ros::Time::now();
	cloud_pub.publish(cloud_out);
}

bool
MapPlobs::find_category (const std::string& category) const
{
	std::map<std::string, int>::const_iterator it;

	for (it = categories_.begin(); it != categories_.end(); it++) {
		if (categories_.find(it->first) != categories_.end()) {
			return true;
		}
	}
	return false;
}

std::map<std::string, Pblob*>
MapPlobs::get_plobs(std::string category) const
{
	std::map<std::string, Pblob*> cate_plobs;

	std::map<std::string, Pblob*>::const_iterator it;
	for (it = plobs_.begin(); it != plobs_.end(); it++) {
		if (it->first.find(category) != std::string::npos) {
			cate_plobs[it->first] = it->second;
		}
	}
	return cate_plobs;
}

std::vector<std::string>
MapPlobs::get_categories() const
{
	std::vector<std::string> categories;

	std::map<std::string, int>::const_iterator it;
	for (it = categories_.begin(); it != categories_.end(); it++) {
		categories.push_back(it->first);
	}
	return categories;
}

void
MapPlobs::add(std::map<std::string, Pblob*>& plobs)
{
	std::map<std::string, Pblob*>::iterator it;

	for (it = plobs.begin(); it != plobs.end(); it++) {
		insert(it->second);
	}
}

void
MapPlobs::update(std::map<std::string, Pblob*>& plobs, std::vector<pcl::PointXYZRGB> limits)
{
	std::map<std::string, Pblob*>::const_iterator it;

	for (it = plobs_.begin(); it != plobs_.end(); it++) {
		std::map<std::string, Pblob*>::iterator itU = plobs.find(it->first);

		if (itU == plobs.end()) {
			if (it->second->isVisible(limits)) {
				it->second->dissapear();
			} else {
				it->second->hidden();
			}

		} else {
			it->second->visible(itU->second);
		}
	}
}

void
MapPlobs::insert_all(MapPlobs& plobs)
{
	std::map<std::string, Pblob*>::iterator it;

	for (it = plobs.plobs_.begin(); it != plobs.plobs_.end(); it++) {
		insert(it->second);
	}
}

void
MapPlobs::insert(const Pblob* instant)
{
	if (!instant->isDefined()) {
		return;
	}

	std::string category = instant->get_box().Class;
	std::map<std::string, int>::const_iterator it = categories_.find(category);

	if (it == categories_.end()) {
		categories_[category] = 1;
	} else {
		categories_[category]++;
	}
	Pblob* permanent = new Pblob(instant);
	plobs_[get_new_key_plob(category)] = permanent;
}

std::map<std::string, Pblob*>::iterator
MapPlobs::erase(std::map<std::string, Pblob*>::iterator it)
{
	std::string category = it->second->get_box().Class;

	categories_[category]--;
	if (categories_[category] <= 0) {
		categories_.erase(category);
	}
	return plobs_.erase(it);
}

void
MapPlobs::print() const
{
	print_categories();
	print_plobs();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
MapPlobs::get_cloud() const
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::map<std::string, Pblob*>::const_iterator it;

	for (it = plobs_.begin(); it != plobs_.end(); it++) {
		it->second->merge_cloud(pcrgb_out);
	}
	return pcrgb_out;
}

void
MapPlobs::print_trackers(const std::map<std::string,std::map<std::string, Pblob*>>& trackers)
{
	std::map<std::string, Pblob*>::const_iterator it;

	ROS_INFO("--- Plobs news to add ---");
	for (it = trackers.at("add").begin(); it != trackers.at("add").end(); it++) {
		ROS_INFO("*** ADD TRACE ***");
		ROS_INFO("Key : %s", it->first.c_str());
		it->second->print();
	}

	ROS_INFO("--- Plobs news to update ---");
	for (it = trackers.at("update").begin(); it != trackers.at("update").end(); it++) {
		ROS_INFO("*** UPDATE TRACE ***");
		ROS_INFO("Key : %s", it->first.c_str());
		it->second->print();
	}
}

void
MapPlobs::print_plobs() const
{
	std::map<std::string, Pblob*>::const_iterator it;

	fprintf(stderr, "%s\n", "--- Plobs ---");
	for (it = plobs_.begin(); it != plobs_.end(); it++) {
		fprintf(stderr, "Key : %s\n", it->first.c_str());
		it->second->print();
	}
}

void
MapPlobs::print_categories() const
{
	std::map<std::string, int>::const_iterator it;

	fprintf(stderr, "%s\n", "--- Categories ---");
	for (it = categories_.begin(); it != categories_.end(); it++) {
		fprintf(stderr, "Category : %s\n", it->first.c_str());
		fprintf(stderr, "Num plobs : %d\n", it->second);
	}
}

void
print_table_sims (const Eigen::MatrixXf& simtable)
{
	std::cerr << std::endl;
	std::cerr << "simtable :\t" << std::endl;
	std::cerr << std::endl;

	for (int i = 0; i < simtable.rows(); ++i) {
		for (int j = 0 ; j < simtable.cols() ; j++) {
			fprintf(stderr, "\t%6.6lf\t", simtable(i, j));
		}
		std::cerr << std::endl;
	}
	std::cerr << std::endl;
}
