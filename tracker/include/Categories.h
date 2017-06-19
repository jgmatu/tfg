#ifndef SRC_CATEGORIES_H_
#define SRC_CATEGORIES_H_


#include <ros/ros.h>

class Categories {

public:

	enum class Category {AEROPLANE, BICYCLE, BIRD, BOAT, BOTTLE, BUS, CAR, CAT, CHAIR, COW, DINING_TABLE,
		DOG, HORSE, MOTORBIKE, PERSON, POTTED_PLANT, SHEEP, SOFA, TRAIN, TV_MONITOR, NUM_CATEGORIES};


	bool is_static(const std::string& category);

	bool is_dynamic(const std::string& category);

	std::vector<std::string> get_categories();

	Category get_category(std::string category);

private :

	std::map<Category, std::string> statics_ = {
			{Category::CHAIR		, "CHAIR"},
			{Category::DINING_TABLE	, "DINING TABLE"},
			{Category::POTTED_PLANT	, "POTTED_PLANT"},
			{Category::SOFA			, "SOFA"},
			{Category::TV_MONITOR	, "TV MONITOR"}
	};

	std::map<Category, std::string> dynamics_ = {
			{Category::AEROPLANE	, "AEROPLANE"},
			{Category::BICYCLE		, "BICYCLE"},
			{Category::BIRD			, "BIRD"},
			{Category::BOAT			, "BOAT"},
			{Category::BOTTLE		, "BOTTLE"},
			{Category::BUS			, "BUS"},
			{Category::CAR			, "CAR"},
			{Category::CAT			, "CAT"},
			{Category::COW			, "COW"},
			{Category::DOG			, "DOG"},
			{Category::HORSE		, "HORSE"},
			{Category::MOTORBIKE	, "MOTORBIKE"},
			{Category::PERSON		, "PERSON"},
			{Category::SHEEP		, "SHEEP"},
			{Category::TRAIN		, "TRAIN"}
	};

};

#endif /* SRC_COLORSTOOL_H_ */
