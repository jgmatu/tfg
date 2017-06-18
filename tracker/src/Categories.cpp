/*
 * Categories.cpp
 *
 *  Created on: Jun 16, 2017
 *      Author: javi
 */


#include "../include/Categories.h"


bool
Categories::is_static(const std::string& category)
{
	std::map<Category, std::string>::const_iterator it;

	for (it = statics_.begin(); it != statics_.end(); it++) {
		if (category == it->second) {
			return true;
		}
	}
	return false;
}

bool
Categories::is_dynamic(const std::string& category)
{
	std::map<Category, std::string>::const_iterator it;

	for (it = dynamics_.begin(); it != dynamics_.end(); it++) {
		if (category == it->second) {
			return true;
		}
	}
	return false;
}

std::vector<std::string>
Categories::get_categories()
{
	std::vector<std::string> categories;
	std::map<Category, std::string>::const_iterator it;

	for (it = statics_.begin(); it != statics_.end(); it++) {
		categories.push_back(it->second);
	}
	for (it = dynamics_.begin(); it != dynamics_.end(); it++) {
		categories.push_back(it->second);
	}
	return categories;
}


