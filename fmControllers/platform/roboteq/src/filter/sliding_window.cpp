#include "filter/sliding_window.hpp"

SlidingWindowFilter::SlidingWindowFilter()
{
	// Init pointers
	oldest = newest = 0;
	initialised = false;
}

SlidingWindowFilter::~SlidingWindowFilter()
{

}

void SlidingWindowFilter::init(int value)
{
	// Init data array
	for(int i = 0 ; i < FILTER_SIZE ; i++)
	{
		data[i].header.stamp = ros::Time::now();
		data[i].data = value;
	}
	initialised = true;
}

void SlidingWindowFilter::push(msgs::IntStamped msg)
{
	if(initialised)
	{
		// Copy message
		data[oldest].header.stamp = msg.header.stamp;
		data[oldest].data = msg.data;

		// Move pointers
		newest = oldest;
		oldest++;
		if(oldest == FILTER_SIZE)
			oldest = 0;
	}
	else
		init(0);
}

double SlidingWindowFilter::get(void)
{
	double out = 0;
	if(initialised)
	{
		// Calculate change/time
		int d_data = data[newest].data - data[oldest].data;
		double d_time = (data[newest].header.stamp - data[oldest].header.stamp).toSec();
		out = ((double)d_data) / d_time;
	}
	return out;
}
