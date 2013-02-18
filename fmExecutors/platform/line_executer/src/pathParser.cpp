/*
 * parse-yaml.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: soeni05
 */

#include "pathParser.h"


void operator >> (const YAML::Node& node, geometry_msgs::PoseStamped &pose) {
	node["position"][0] >> pose.pose.position.x;
	node["position"][1] >> pose.pose.position.y;
	node["position"][2] >> pose.pose.position.z;
}

pathParser::pathParser(std::string filepath){

	path = new std::vector<geometry_msgs::PoseStamped>;

	std::ifstream fin(filepath.c_str());
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	try{
		for(unsigned i=0;i<doc.size();i++) {
			  geometry_msgs::PoseStamped pose;
			  doc[i] >> pose;
			  path->push_back(pose);
		}
	} catch(YAML::ParserException& e) {
		std::cout << e.what() << "\n";
	}
}

