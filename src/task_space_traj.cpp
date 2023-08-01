//  ---------------------- Doxygen info ----------------------
//! \file 01_RMLPositionSampleApplication.cpp
//!
//! \brief
//! Test application number 1 for the Reflexxes Motion Libraries
//! (basic position-based interface)
//!
//! \date March 2014
//!
//! \version 1.2.6
//!
//! \author Torsten Kroeger, <info@reflexxes.com> \n
//!
//! \copyright Copyright (C) 2014 Google, Inc.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see
//! <http://www.gnu.org/licenses/>.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <ur5e_control/Plan.h>
#include <trajectory_msgs/JointTrajectory.h> 
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <turtlesim/Pose.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.1
#define NUMBER_OF_DOFS                          1


//*************************************************************************
// Main function to run the process that contains the test application
//
// This function contains source code to get started with the Type II
// Reflexxes Motion Library. Only a minimum amount of functionality is
// contained in this program: a simple trajectory for a
// three-degree-of-freedom system is executed. This code snippet
// directly corresponds to the example trajectories shown in the
// documentation.
//*************************************************************************


ur5e_control::Plan plan;
bool plan_available = false;
bool turtle_pos_received = false;
int number_of_points = 0;


turtlesim::Pose turtle_pos;
int control_mode = 1;

void get_pos(const turtlesim::Pose & _data){
	turtle_pos = _data;
	turtle_pos_received = true;
}

void get_plan(const ur5e_control::Plan & _data){
	plan = _data;
	plan_available = true;
	number_of_points = plan.points.size();
	std::cout << "received a plan with " << number_of_points << " points"<< std::endl;
}



void initialize_plan(RMLPositionInputParameters  *_IP){
		_IP->CurrentPositionVector->VecData      [0] =    plan.points[0].linear.x; //x

		_IP->CurrentVelocityVector->VecData      [0] =    0.0;


		_IP->CurrentAccelerationVector->VecData  [0] =    0.0;


		_IP->MaxVelocityVector->VecData          [0] =    0.1      ;


		_IP->MaxAccelerationVector->VecData      [0] =    0.1      ;



		_IP->MaxJerkVector->VecData              [0] =    0.1      ;


		//setting the target velocities and positions
		_IP->TargetPositionVector->VecData       [0] =   plan.points[1].linear.x;


		_IP->TargetVelocityVector->VecData       [0] =   0.0;


		//determine which Degrees of freedom should be calculated
		_IP->SelectionVector->VecData            [0] =   true        ;


}

int main(int argc, char * argv[])
{

    ros::init(argc,argv,"task_space_traj");
    ros::NodeHandle nh_;


    int loop_freq = 10;
    float dt = (float) 1/loop_freq;
    ros::Rate loop_rate(loop_freq);
    ros::Publisher reflexxes_pub = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);

    ros::Subscriber plan_sub = nh_.subscribe("/plan",1,get_plan);
    ros::Subscriber pos_sub = nh_.subscribe("/turtle1/pose" ,1, get_pos);
    

    geometry_msgs::Twist ref; // for publishing the reference traj
    // some default codes form the solver

    // ********************************************************************
    // Variable declarations and definitions

    int                         ResultValue                 =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;

    RMLPositionFlags            Flags                                   ;
    
    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                            ,   CYCLE_TIME_IN_SECONDS   );

    IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );

    OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

    // ********************************************************************
    // Set-up a timer 
    
    // ********************************************************************
    
    
    // ********************************************************************
    // Set-up the input parameters

    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // an RMLPositionInputParameters::CurrentPositionVector vector object.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    // The internal data structures make use of native C data types
    // (e.g., IP->CurrentPositionVector->VecData is a pointer to
    // an array of NUMBER_OF_DOFS double values), such that the Reflexxes
    // Library can be used in a universal way.

	// wait for a plan
	while (!plan_available){
		loop_rate.sleep();
		
		ros::spinOnce();
	}
	initialize_plan(IP);
	
    // ********************************************************************
    // Starting the control loop
    int ctr = 2;
    while(ros::ok()){
	    if (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
	    {

			// ****************************************************************
			// Wait for the next timer tick
			// (not implemented in this example in order to keep it simple)
			// ****************************************************************

			// Calling the Reflexxes OTG algorithm
			ResultValue =   RML->RMLPosition(       *IP
				                                ,   OP
				                                ,   Flags       );

			if (ResultValue < 0)
			{
				printf("An error occurred (%d).\n", ResultValue );
				break;
			}
			if (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED && (ctr <= number_of_points) ){
				//setting the target velcoity and positions
				int next_wp;
				
				next_wp = ctr % number_of_points;
				std::cout << "moving to point:" << next_wp << std::endl;
				
				IP->TargetPositionVector->VecData       [0] =   plan.points[next_wp].linear.x;

				// update the start point via the current robot positionsn
				if (turtle_pos_received){
					IP->CurrentPositionVector->VecData[0] = turtle_pos.x;

				}
			
				ctr ++;
				ResultValue =   RML->RMLPosition(       *IP
					                                ,   OP
					                                ,   Flags       );
			}
			// ****************************************************************
			// Here, the new state of motion, that is
			//
			// - OP->NewPositionVector
			// - OP->NewVelocityVector
			// - OP->NewAccelerationVector
			//
			// can be used as input values for lower level controllers. In the
			// most simple case, a position controller in actuator space is
			// used, but the computed state can be applied to many other
			// controllers (e.g., Cartesian impedance controllers,
			// operational space controllers).
			// ****************************************************************

			// ****************************************************************
			// Feed the output values of the current control cycle back to
			// input values of the next control cycle

			*IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
			*IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
			*IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

			// update a piece of trajectoy based on the recent calculations
			ref.linear.x = IP->CurrentPositionVector->VecData[0];


			//dbg.angular.x = ctr;
			reflexxes_pub.publish(ref);
		
	    }
		
		loop_rate.sleep();
		
		ros::spinOnce();
		

	    // ********************************************************************
	    // Deleting the objects of the Reflexxes Motion Library end terminating
	    // the process
    }
    delete  RML         ;
    delete  IP          ;
    delete  OP          ;

    exit(EXIT_SUCCESS)  ;
}


