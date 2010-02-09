/*
 * controller.h
 *
 *  Created on: Feb 9, 2010
 *      Author: joseph
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//#define STAUBLI_IP "http://192.168.1.254:5653/" //physical CS8
#define STAUBLI_IP "http://128.59.20.63:5653/"	//ANTIGUA
//#define STAUBLI_IP "http://128.59.19.61:5653/"	//Thomas' computer


//Declare functions
void print(double e);
int GetPosition(staubli::command::Response *res);
int GetJoints(staubli::command::Response *res);
int MoveJoints(staubli::command::Request  *req,
			   staubli::command::Response *res);
bool commandhandler(staubli::command::Request  &req,
					staubli::command::Response &res );

#endif /* CONTROLLER_H_ */
