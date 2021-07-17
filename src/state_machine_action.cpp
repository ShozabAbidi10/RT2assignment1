#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PositionAction.h>
#include <typeinfo>
#include <string>

int start_ = -1;
int count_ = 0;
std::string Check_ = "non";


bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start_ = 1;
    	count_ = 1;
    }
    else{
    	start_ = 0;
    }
    return true;
}

typedef actionlib::SimpleActionClient<rt2_assignment1::PositionAction> StateMachineClient;


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   // Telling the action clinet that we want to spin the thread by default.
   StateMachineClient ac("/go_to_point", true);
  
    
   ROS_INFO("Waiting for action server to start.");
   // wait for the action server to start
   ac.waitForServer(); //will wait for infinite time

   ROS_INFO("Action server started, sending goal."); 
    
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   
   rt2_assignment1::PositionGoal goal;
   
   // send a goal to the action
   
   while(ros::ok())
   {
   	ros::spinOnce();
   	actionlib::SimpleClientGoalState state = ac.getState();
   	
   	if(start_ == 1)
   	{
   	    if(count_ == 1)
            {
                client_rp.call(rp);
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;  //this goal is keeps on changing because while is active.
                ac.sendGoal(goal);
                count_++;
                std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
            } 
   	    Check_ = state.toString().c_str();
   	    if(Check_ == "SUCCEEDED")
   	    {
   	        std::cout << "\nPosition Reached! " << std::endl;  
   	        client_rp.call(rp);
   	        goal.x = rp.response.x;
                goal.y = rp.response.y;
                goal.theta = rp.response.theta;  //this goal is keeps on changing because while is active.       
                ac.sendGoal(goal);
                std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
            }   
            //std::cout << state.toString().c_str() << std::endl;
            //std::cout << typeid(state.toString().c_str()).name() << std::endl;
            //std::cout << typeid(Check_).name() << std::endl;
            //std::cout << typeid("SUCCEEDED").name() << std::endl;
        
        }
   	else if(start_ == 0)
   	{
   	    ac.cancelAllGoals();
   	    std::cout << "Current Goal is Cancelled!" << std::endl;
   	    count_ = 0;
   	    start_ = -1;
   	}
   }  	
  
   
   return 0;
}
