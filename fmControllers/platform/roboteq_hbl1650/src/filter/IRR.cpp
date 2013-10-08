/*
 * IRR.cpp
 *
 *  Created on: Jun 13, 2013
 *      Author: martin
 */

#include "filter/IRR.h"

IRR::IRR(int n) {
	x.assign(n,0);
	y.assign(n,0);
}

double IRR::update(double input) {

	double B = 0;
	double A = 0;

	x.push_front(input);
	x.pop_back();



	for(unsigned int j = 1; j < y.size(); j++ ) {
				A += a[j]*y.at(j-1);
			}

	for(unsigned int i = 0; i < x.size(); i++ ) {
		B += b[i]*x.at(i);
	}
	y.pop_back();
	y.push_front((B-A));

	return y.front();
}

IRR::~IRR() {
	// TODO Auto-generated destructor stub
}
