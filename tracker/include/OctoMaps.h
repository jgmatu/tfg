/*
 * OctoMaps.h
 *
 *  Created on: Jun 16, 2017
 *      Author: javi
 */

#ifndef SRC_OCTOMAPS_H_
#define SRC_OCTOMAPS_H_

#include "../include/Pblob.h"
#include "../include/OctoMap.h"

class OctoMaps {
public :
	OctoMaps();
	virtual ~OctoMaps();

	void
	refresh(const MapPlobs& permanents);

	void
	publish(const std::map<std::string, ros::Publisher>& octo_pubs, std::string robot_frame);

private :

	void
	insert_octomap(std::string category, std::map<std::string, Pblob*> plobs);

	void
	insert_plobs_category(const MapPlobs& permanents, const std::string& category);

	std::map<std::string, OctoMap*> octomaps_;

};

#endif /* SRC_OCTOMAPS_H_ */
