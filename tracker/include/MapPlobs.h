/*
 * MapPlobs.h
 *
 *  Created on: Jun 10, 2017
 *      Author: javi
 */

#ifndef SRC_MAPPLOBS_H_
#define SRC_MAPPLOBS_H_

#include "../include/Pblob.h"

class MapPlobs {
public :

	MapPlobs();
	MapPlobs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const std::vector<darknet_ros_msgs::BoundingBox>&);

	virtual ~MapPlobs();

	void
	insert(const Pblob* pblob);

	void
	add(std::map<std::string, Pblob*>& plobs);

	void
	update(std::map<std::string, Pblob*>& plobs);

	void
	insert_all(MapPlobs&);

	int
	size() const;

	bool
	empty() const;

	void
	clear();

	void
	set(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb, const std::vector<darknet_ros_msgs::BoundingBox>& boxes);

	void
	set_all_hidden();

	void
	publish(ros::Publisher cloud_pub, std::string frame_id);

	std::map<std::string, Pblob*>
	get_plobs(std::string category) const;

	std::vector<std::string>
	get_categories() const;

	bool
	find_category (const std::string& category) const;


	void
	print() const;


private :

	void
	erasePlobsMinSize();

	std::map<std::string, Pblob*>::iterator
	erase(std::map<std::string, Pblob*>::iterator it);

	std::string
	get_new_key_plob(const std::string) const;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	get_cloud() const;

	void
	print_categories() const;

	void
	print_plobs() const;

	void
	print_trackers(const std::map<std::string,std::map<std::string, Pblob*>>& trackers);

	std::map<std::string, Pblob*> plobs_;
	std::map<std::string, int> categories_;

};

#endif /* SRC_MAPPLOBS_H_ */
