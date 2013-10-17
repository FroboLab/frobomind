/*
 * IRR.h
 *
 *  Created on: Jun 13, 2013
 *      Author: martin
 */

#ifndef IRR_H_
#define IRR_H_
#include <deque>
using namespace std;

const double a[] = {1.0000, -4.0558, 7.4504,   -7.9099,    5.2005,   -2.1060,    0.4846,   -0.0487};
const double b[] = {0.0001,    0.0008,    0.0025,    0.0041,    0.0041,    0.0025,    0.0008,    0.0001};

class IRR {


public:
	IRR(int n);

	double update(double input);

	virtual ~IRR();

private:
	deque<double> x;
	deque<double> y;
};

#endif /* IRR_H_ */
