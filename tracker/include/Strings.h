/*
 * Strings.h
 *
 *  Created on: Jun 19, 2017
 *      Author: javi
 */

#ifndef SRC_STRINGS_H_
#define SRC_STRINGS_H_

#include <ros/ros.h>

class Strings {
public:

	Strings(const std::string& s);
	virtual ~Strings();

	bool
	replace(const std::string& from, const std::string& to);

	std::string
	get_string();

	void
	upper();

	void
	lower();

	std::string
	get_key(const std::string& category);

private:

	std::string string_;
};



#endif /* SRC_STRINGS_H_ */
