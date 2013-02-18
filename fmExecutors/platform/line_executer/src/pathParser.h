/*
 * pathParser.h
 *
 *  Created on: Apr 27, 2012
 *      Author: soeni05
 */

#ifndef PATHPARSER_H_
#define PATHPARSER_H_


#include <ros/ros.h>
#include <yaml.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <fstream>

class pathParser{
public:
	std::vector<geometry_msgs::PoseStamped> * path;
	pathParser(std::string filepath);
};



#endif /* PATHPARSER_H_ */
