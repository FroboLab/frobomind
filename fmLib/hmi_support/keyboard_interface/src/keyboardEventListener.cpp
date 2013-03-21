/****************************************************************************
 # FroboMind frobit_interface.cpp
 # Copyright (c) 2012, Kent Stark Olsen <keols09@student.sdu.dk>
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #	* Redistributions of source code must retain the above copyright
 #  	notice, this list of conditions and the following disclaimer.
 #	* Redistributions in binary form must reproduce the above copyright
 #  	notice, this list of conditions and the following disclaimer in the
 #  	documentation and/or other materials provided with the distribution.
 #	* Neither the name FroboMind nor the
 #  	names of its contributors may be used to endorse or promote products
 #  	derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 # Change Log:
 # 21-Mar 2013 Leon: Fixed SIGTERM issue
 ****************************************************************************/
#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <ros/callback_queue.h>

int kfd = 0;
struct termios cooked, raw;
void quit(int sig);

void listenForKeyEvents()
{
	char key = 0;
	std_msgs::Char msg;
	ros::NodeHandle globalNodeHandler;
	ros::Publisher keyPublisher = globalNodeHandler.advertise<std_msgs::Char>("/fmHMI/keyboard", 100);

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);

	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Listening for keyboard events and publish these events...");
	signal(SIGABRT, quit);
	signal(SIGINT, quit);
	signal(SIGTERM, quit);
	while (ros::ok())
	{
		// Get events from keyboard
		if (read(kfd, &key, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		//	Publish keyboardEvent
		ROS_DEBUG("%c", key);
		msg.data = key;
		keyPublisher.publish(msg);
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
	}
}

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyboardEventListener");
	listenForKeyEvents();

	return 0;
}
