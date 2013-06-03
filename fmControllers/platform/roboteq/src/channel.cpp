#include "roboteq/hbl2350.hpp"

Channel::Channel( )
{
	down_time = 0;
}

void Channel::onHallFeedback(ros::Time time, int feedback)
{
	message.hall.header.stamp = time;
	message.hall.data = feedback;

	// TODO: This is a temporary hack to make hall values relative
	hall_value = feedback - last_hall;
	last_hall = feedback;
	// end hack..

	//std::cout << "Hall value: " << feedback << " Relative: " << hall_value << std::endl;
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
	last_twist_received = ros::Time::now();
}

void Channel::onDeadman(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
		last_deadman_received = ros::Time::now();
}

void Channel::onTimer(const ros::TimerEvent& e, RoboTeQ::status_t status)
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
						}

						/* Get new output */
						double period = (ros::Time::now() - last_regulation).toSec();
						double feedback = hall_value*ticks_to_mps;
						double setpoint = regulator.output_from_input(velocity, feedback , period);
						last_regulation = ros::Time::now();

						/* Convert to roboteq format */
						int output = ((setpoint)*roboteq_max)/8; //TODO: Hard coded max for Pichi

						/* Send motor speed */
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
					regulator.reset_integrator();
				}
			}
			else /* controller is not responding */
			{
				ROS_INFO("%s: Controller is not responding",ros::this_node::getName().c_str());
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
			initController("standard");
			status.initialised = true;
		}
	}
	else /* controller is not online */
	{
		ROS_INFO("%s: Controller is not yet online",ros::this_node::getName().c_str());
		/* Try to re-connect and re-initialise */
		transmit("?FID\r");
	}

	/* Publish the status message */
	status_out.header.stamp = ros::Time::now();
	status_out.data = ss.str();
	status_publisher.publish(status_out);
}

