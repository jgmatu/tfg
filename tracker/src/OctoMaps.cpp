/*
 * OctoMaps.cpp
 *
 *  Created on: Jun 16, 2017
 *      Author: javi
 */


// The idea is create octomaps by category, the objects are
// permanents with a linear relation with the time considering
// the plobs detected by the CNN but... How I know I have seen
// one object in the last time?? By the similarity of the object
// and by its position... This is the key to get the octomaps of
// objects...
#include "../include/OctoMaps.h"


OctoMaps::OctoMaps() :
	octomaps_()
{
	;
}

OctoMaps::~OctoMaps()
{
	std::map<std::string, OctoMap*>::iterator it;

	for (it = octomaps_.begin(); it != octomaps_.end(); it++) {
		delete(it->second);
	}
	octomaps_.clear();
}

void
OctoMaps::refresh(const MapPlobs& permanents)
{
	std::vector<std::string> categories;
	std::vector<std::string>::const_iterator it;

	categories = permanents.get_categories();
	for (it = categories.begin(); it != categories.end(); it++) {
		insert_octomap(*it, permanents.get_plobs(*it));
	}
}

void
OctoMaps::publish(const std::vector<ros::Publisher>& octo_pubs)
{
	;
}


void
OctoMaps::insert_octomap(std::string category, std::map<std::string, Pblob*> plobs)
{
	OctoMap* octomap = new OctoMap();

	octomap->insert_plobs(plobs);
	octomaps_[category] = octomap;
}


