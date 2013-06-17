#include "roboteq/hbl2350.hpp"

Channel::Channel( ) : velocity_filter(8) // do not change!!
{
	down_time = 0;
	current_setpoint = 0;
}

void Channel::onHallFeedback(ros::Time time, int feedback)
{
	message.hall.header.stamp = time;
	message.hall.data = feedback;

	// TODO: This is a temporary hack to make hall values relative
	hall_value += feedback - last_hall;
	last_hall = feedback;
	// end hack..

	//std::cout << "Channel:" << ch << " Hall value: " << feedback << " Relative: " << hall_value << std::endl;
	publisher.hall.publish(message.hall);
}

void Channel::onPowerFeedback(ros::Time time, int feedback)
{
	message.power.header.stamp = time;
	message.power.data = feedback;
	publisher.power.publish(message.power);
}

void Channel::onTemperatureFeedback(ros::Time time, int feedback)
{
	message.temperature.header.stamp = time;
	message.temperature.data = feedback;
	publisher.temperature.publish(message.temperature);
}

void Channel::onCmdVel(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	velocity = msg->twist.linear.x;
	// limit velocity
	if(velocity >  max_velocity_mps) velocity = max_velocity_mps;
	if(velocity < -max_velocity_mps) velocity = -max_velocity_mps;

	last_twist_received = ros::Time::now();
}

void Channel::onDeadman(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
		last_deadman_received = ros::Time::now();
}

void Channel::onTimer(const ros::TimerEvent& e, RoboTeQ::status_t& status)
{
	std::stringstream ss, out; /* streams for holding status message and command output */

	if(status.online) /* is set when controller answers to FID request */
	{
		ss << "controller_online ";
		if(status.initialised) /* is set when init function completes */
		{
			ss << "controller_initialised ";
			if(status.responding) /* is set if the controller publishes serial messages */
			{
				ss << "controller_responding ";
				if(status.cmd_vel_publishing) /* is set if someone publishes twist messages */
				{
					ss << "cmd_vel_publishing ";
					if(status.deadman_pressed) /* is set if someone publishes true on deadman topic */
					{
						ss << "deadman_pressed ";

						if(status.emergency_stop)
						{
							/* release emergency stop */
							transmit("!MG\r");
							status.emergency_stop = false;
							current_setpoint = 0;
						}

						/* Get new output */
						double period = (ros::Time::now() - last_regulation).toSec();
						double feedback = (( (double)hall_value)*ticks_to_meter)/period;
						if(ch == 1)
						{
							feedback *= -1;
						}
						// reset hall_value after read, so we get the relative hall_ticks since last read
						hall_value = 0;

						double fb_sm = velocity_filter.update(feedback);
						vel_msg.data = fb_sm;
						vel_publisher.publish(vel_msg);
						// calculate desired change in output using pid regulator
						double sp_change = regulator.output_from_input(velocity, fb_sm , period);

						// the output to maintain on the motor is then
						current_setpoint += sp_change;

						if(current_setpoint >  max_output) current_setpoint = max_output;
						if(current_setpoint < -max_output) current_setpoint = -max_output;

						//std::cout << " vel " << velocity << " fb " << feedback << " fb_sm " << fb_sm;
						//std::cout << " sp_ch " << sp_change << " cur_sp " << current_setpoint << std::endl;

						last_regulation = ros::Time::now();

						/* Convert to roboteq format */
						int output = (current_setpoint);

						/* Send motor output command  */
						out << "!G " << ch << " " << output << "\r";
						transmit(out.str());

					}
					else /* deadman not pressed */
					{
						/* Set speeds to 0 and activate emergency stop */
						transmit("!EX\r");
						status.emergency_stop = true;
						transmit("!G 1 0\r");
						transmit("!G 2 0\r");
						current_setpoint = 0;
						velocity = 0;
						regulator.reset_integrator();
					}
				}
				else /* Cmd_vel is not publishing */
				{
					/* Set speeds to 0 and activate emergency stop */
					transmit("!EX\r");
					status.emergency_stop = true;
					transmit("!G 1 0\r");
					transmit("!G 2 0\r");
					current_setpoint = 0;
					velocity = 0;
					regulator.reset_integrator();

				}
			}
			else /* controller is not responding */
			{
				ROS_INFO_THROTTLE(5,"%s: Controller is not responding",ros::this_node::getName().c_str());
				down_time++;
				if(down_time > 10)
				{
					/* Try to re-connect and re-initialise */
					transmit("?FID\r");
					down_time = 0;
				}
			}
		}
		else /* Controller is not initialised */
		{
			ROS_INFO("%s: Controller is not initialised",ros::this_node::getName().c_str());
			//initController("standard");
			//status.initialised = true;'
			transmit("?FID\r");
		}
	}
	else /* controller is not online */
	{
		ROS_INFO_THROTTLE(5,"%s: Controller is not yet online",ros::this_node::getName().c_str());
		/* Try to re-connect and re-initialise */
		transmit("?FID\r");
	}

	/* Publish the status message */
	//status_out.header.stamp = ros::Time::now();
	//status_out.data = ss.str();
	//status_publisher.publish(status_out);
}

Circular_queue::Circular_queue()
{
	size = 5;
	head = tail = 0;
	store = new double[size];
	for(int i = 0 ; i < size ; i++)
		store[i] = 0;
}

Circular_queue::Circular_queue( int sz )
{
	size = sz;
	head = tail = 0;
	store = new double[size];

}

Circular_queue::~Circular_queue()
{
	delete store;
}

void Circular_queue::push_back( double item )
{
	if( tail >= size - 1 )
		tail=0;//resize();

	store[tail++] = item;
}

double Circular_queue::pop_front( void )
{
	if( head != tail )
		return store[head++];
	else
		throw("Fault. Queue is empty");
}

void Circular_queue::resize( void )
{
	double * temp_p = new double[ size * 2 ];
	for( int i = 0 ; i < size ; i++ )
		temp_p[i] = store[i];
	delete store;
	store = temp_p;
	size *= 2;
}

bool Circular_queue::isEmpty( void )
{
	return head == tail;
}

double Circular_queue::average( void )
{
	double out = 0;

	for(int i = 0 ; i < size ; i++)
		out += store[i];

	return out/size;
}
