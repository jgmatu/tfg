/*
 * Strings.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: javi
 */

#include "../include/Strings.h"

#include <boost/algorithm/string.hpp>
#include <string>

Strings::Strings(const std::string& s)
{
	string_ = s;
}

Strings::~Strings()
{
	string_.clear();
}


bool
Strings::replace(const std::string& from, const std::string& to)
{
    size_t start_pos = string_.find(from);
    if(start_pos == std::string::npos) {
        return false;
    }
    string_.replace(start_pos, from.length(), to);
    return true;
}

std::string
Strings::get_string()
{
	return string_;
}

void
Strings::upper()
{
	boost::to_upper(string_);
	string_ = boost::to_upper_copy<std::string>(string_);
}

void
Strings::lower()
{
	boost::to_lower(string_);
	string_ = boost::to_lower_copy<std::string>(string_);
}

std::string
Strings::get_key(const std::string& category)
{
	replace(" ", "_");
	lower();

	return get_string();
}
