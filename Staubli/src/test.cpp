
#include "TX60L.h"
#include <iostream>
#include <algorithm>


void print(double e){
	std::cout << " " << e;
}

int main(){
	TX60L robot;
	//robot.Login("http://192.168.1.254:5653/", "default", "");
	//robot.Login("http://localhost:5653/", "default", "");
        robot.Login("http://128.59.20.63:5653/", "default", ""); //ANTIGUA
	//robot.Login("http://128.59.19.61:5653/", "default", ""); //Thomas' computer
	robot.Power(true);
	std::vector<double> j, min, max,r,targetPos,targetJoint;
	std::vector<int> robo;
	j.clear();

	//Move each joint with 35, 40, 45, 20, 90, 0
	j.push_back(DEGREE_2_RADIAN(54.18));
	j.push_back(DEGREE_2_RADIAN(-30.31));
	j.push_back(DEGREE_2_RADIAN(56.6));
	j.push_back(DEGREE_2_RADIAN(-8.81));
	j.push_back(DEGREE_2_RADIAN(-23.6));
	j.push_back(DEGREE_2_RADIAN(-62.0));
	robot.ResetMotion();
	robot.MoveJoints(j);
	
	//Get current joint
	std::vector<double> j2;
	j2.resize(6);
	//std::cout << "Hello";

	if(robot.GetRobotJoints(j2)){
		for_each(j2.begin(), j2.end(), print);
		std::cout <<"\n";
	}else
		std::cout <<"wrong\n";

	//Publish current position
	
	
	return 0;
	
	//------------- Test V3 ------------------------
	//forward kinematics
	j.clear();
	robot.GetRobotJoints(j);
	robot.ForwardKinematics(j,targetPos);    // Unit : meter, radian
	for_each(targetPos.begin(), targetPos.end(), print);
	return 0;
	//C# sample : forwardKine(35.00, 40.00, 45.00, 20.00, 90.00, 0.00) = 557.32 , 443.89 , 280.11 , -160.74 , -7.42 , 145.38, LEFTY, POSITIVE, POSITIVE
	
	//reverse kinematics
	targetPos.push_back(0.53534);
	targetPos.push_back(0.12191);
	targetPos.push_back(0.550);
	targetPos.push_back(DEGREE_2_RADIAN(-44.76));
	targetPos.push_back(DEGREE_2_RADIAN(67.37));
	targetPos.push_back(DEGREE_2_RADIAN(58.06));
        std::vector<double> currentJnts;
	robot.GetRobotJoints(currentJnts);
	std::cout<<"Current Joints : ";
	for_each(currentJnts.begin(), currentJnts.end(), print);
	std::cout<<"\n";
        robot.InverseKinematics(targetPos,currentJnts, targetJoint);
	std::cout<<"Target Joint : ";
	for_each(targetJoint.begin(), targetJoint.end(), print);
	std::cout<<"\n";
	robot.ResetMotion();
	robot.MoveJoints(targetJoint);
	return 0;
	//C# sample : reverseKine(500.00, 400.00, 300.00, 0.00, 0.00, 0.00) = 36.87 , 30.76 , 73.87 , 0.00 , -104.64 , -36.87

	targetPos.push_back(0.53534);
	targetPos.push_back(0.12191);
	targetPos.push_back(0.550);
	targetPos.push_back(DEGREE_2_RADIAN(-44.76));
	targetPos.push_back(DEGREE_2_RADIAN(67.37));
	targetPos.push_back(DEGREE_2_RADIAN(58.06));
	robot.MoveLine(targetPos);
	return 0;
}

