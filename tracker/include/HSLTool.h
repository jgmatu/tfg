/*
 * ColorsTool.h
 *
 *  Created on: Jun 14, 2017
 *      Author: javi
 */

#ifndef SRC_COLORSTOOL_H_
#define SRC_COLORSTOOL_H_


#include <ros/ros.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

class HSLTool {

public:

	HSLTool();
	HSLTool(float h, float s, float l);
	HSLTool(octomap::ColorOcTreeNode::Color c);

	virtual ~HSLTool();

	std::vector<unsigned char>
	scaleLight(float likely);

	std::vector<unsigned char>
	HSL2RGB(std::vector<float> hsl);

	std::vector<float>
	RGB2HSL(std::vector<unsigned char> rgb);

private:

	octomap::ColorOcTreeNode::Color color_;
	std::vector<float> hsl_;

	int
	argmax(const std::vector<float>& color);

	std::vector<float>
	getRGBPrim(float h, float C, float X);

	std::vector<unsigned char>
	getRGB(std::vector<float> rgbprim, float m);

	std::vector<unsigned char>
	getColor(const octomap::ColorOcTreeNode::Color& c);

	float
	getSat(float increment, float light);

	float
	getLigth(float cmax, float cmin);


	float
	getHue(std::vector<float> color , int cmaxIdx, float increment);

	std::vector<float>
	getInitColor(std::vector<unsigned char> c);
};

#endif /* SRC_COLORSTOOL_H_ */
