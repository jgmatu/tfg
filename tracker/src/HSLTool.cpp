/*
 * ColorsTool.cpp
 *
 *  Created on: Jun 14, 2017
 *      Author: javi
 */

#include "../include/HSLTool.h"

enum class Color {RED, GREEN, BLUE, NUM_COLORS};

HSLTool::HSLTool()
{
	color_ = octomap::ColorOcTreeNode::Color();
	hsl_ = std::vector<float>(static_cast<int>(Color::NUM_COLORS));
}


HSLTool::HSLTool(float h, float s, float l)
{
	color_ = octomap::ColorOcTreeNode::Color();
	hsl_ = std::vector<float>(static_cast<int>(Color::NUM_COLORS));

	hsl_[0] = h;
	hsl_[1] = s;
	hsl_[2] = l;
}

HSLTool::HSLTool(octomap::ColorOcTreeNode::Color c)
{
	color_ = octomap::ColorOcTreeNode::Color(c.r, c.g, c.b);
	hsl_ = std::vector<float>(static_cast<int>(Color::NUM_COLORS));
}

HSLTool::~HSLTool()
{
	;
}


std::vector<float>
HSLTool::RGB2HSL(std::vector<unsigned char> rgb)
{
	std::vector<float> color = getInitColor(rgb);

	float cmax = *std::max_element(color.begin(), color.end());
	float cmin = *std::min_element(color.begin(), color.end());

	int idx = argmax(color);

	float increment = cmax - cmin;
	float light = getLigth(cmax, cmin);

	std::vector<float> hsl(static_cast<int>(Color::NUM_COLORS));

	hsl[0] = getHue(color, idx, increment);
	hsl[1] = getSat(increment, light);
	hsl[2] = light;
	return hsl;
}

std::vector<unsigned char>
HSLTool::HSL2RGB(std::vector<float> hsl)
{
	float C = (1 - fabs(2 * hsl[2] - 1) * hsl[1]);
	float X = C * (1 - fabs(static_cast<int>(hsl[0] / 60.0f) % 2 - 1));
	float m = hsl[2] - C / 2.0f;

	return getRGB(getRGBPrim(hsl[0], C, X), m);
}

octomap::ColorOcTreeNode::Color
HSLTool::get_color(Categories::Category category)
{
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
	std::vector<unsigned char> rgb = HSL2RGB(hsl);
	return octomap::ColorOcTreeNode::Color(rgb[0], rgb[1], rgb[2]);
}


std::vector<unsigned char>
HSLTool::scaleLight(float likely)
{
	std::vector<unsigned char> color = getColor(color_);
	std::vector<float> hsl = RGB2HSL(color);

	hsl[2] = 1 - likely;
	return HSL2RGB(hsl);
}

float
HSLTool::getHue(std::vector<float> color , int cmaxIdx, float increment)
{
	if (increment == 0) {
		return 0.0f;
	}
	float hue = 0.0f;

	switch (cmaxIdx) {
	case static_cast<int>(Color::RED) :
		hue = 60.0f * ((static_cast<int>((color[static_cast<int>(Color::GREEN)] - color[static_cast<int>(Color::BLUE)]) / increment)) % 6);
		break;
	case static_cast<int>(Color::GREEN) :
		hue = 60.0f * (((color[static_cast<int>(Color::BLUE)] - color[static_cast<int>(Color::RED)]) / increment) + 2);
		break;
	case static_cast<int>(Color::BLUE) :
		hue = 60.0f * (((color[static_cast<int>(Color::RED)] - color[static_cast<int>(Color::GREEN)]) / increment) + 4);
		break;
	default:
		ROS_ERROR("BAD HUE VALUE!!");
	}
	return hue;
}

float
HSLTool::getLigth(float cmax, float cmin)
{
	return (cmax + cmin) / 2;
}

float
HSLTool::getSat(float increment, float light)
{
	float sat = 0;

	if (increment != 0) {
		sat = increment / (1 - fabs(2 * light - 1));
	}
	return sat;
}

int
HSLTool::argmax(const std::vector<float>& color)
{
	int i, idx = 0;
	float max = 0;

	std::vector<float>::const_iterator it;
	for (it = color.begin(), i = 0; it != color.end(); it++, i++) {
		if (*it > max) {
			max = *it;
			idx = i;
		}
	}
	return idx;
}

std::vector<float>
HSLTool::getRGBPrim(float h, float C, float X)
{
	std::vector<float> rgbprim(static_cast<int>(Color::NUM_COLORS));

	if (h >= 0 && h < 60) {
		rgbprim[0] = C;
		rgbprim[1] = X;
		rgbprim[2] = 0;
	} else if (h >= 60 && h < 120) {
		rgbprim[0] = X;
		rgbprim[1] = C;
		rgbprim[2] = 0;
	} else if (h >= 120 && h < 180) {
		rgbprim[0] = 0;
		rgbprim[1] = C;
		rgbprim[2] = X;
	} else if (h >= 180 && h < 240) {
		rgbprim[0] = 0;
		rgbprim[1] = X;
		rgbprim[2] = C;
	} else if (h >= 240 && h < 300) {
		rgbprim[0] = X;
		rgbprim[1] = 0;
		rgbprim[2] = C;
	} else if (h >= 300 && h <= 360) {
		rgbprim[0] = C;
		rgbprim[1] = 0;
		rgbprim[2] = X;
	} else {
		ROS_ERROR("ERROR CONVERTING COLOR");
	}
	return rgbprim;
}

std::vector<unsigned char>
HSLTool::getRGB(std::vector<float> rgbprim, float m)
{
	std::vector<unsigned char> rgb(static_cast<int>(Color::NUM_COLORS));

	for (int i = 0 ; i < static_cast<int>(Color::NUM_COLORS); i++) {
		rgb[i] = (rgbprim[i] + m) * 255;
	}
	return rgb;
}


std::vector<unsigned char>
HSLTool::getColor(const octomap::ColorOcTreeNode::Color& c)
{
	std::vector<unsigned char> color(static_cast<int>(Color::NUM_COLORS));

	color[static_cast<int>(Color::RED)] = c.r;
	color[static_cast<int>(Color::GREEN)] = c.g;
	color[static_cast<int>(Color::BLUE)] = c.b;
	return color;
}

std::vector<float>
HSLTool::getInitColor(std::vector<unsigned char> c)
{
	std::vector<float> color(static_cast<int>(Color::NUM_COLORS));

	color[static_cast<int>(Color::RED)] = c[static_cast<int>(Color::RED)] / 255.0f;
	color[static_cast<int>(Color::GREEN)] = c[static_cast<int>(Color::GREEN)] / 255.0f;
	color[static_cast<int>(Color::BLUE)] = c[static_cast<int>(Color::BLUE)] / 255.0f;
	return color;
}
