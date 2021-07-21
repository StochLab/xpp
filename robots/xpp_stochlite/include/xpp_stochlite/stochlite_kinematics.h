/*
 * stochlite_kinematics.h
 *
 * Created : 4 May, 2021
 * Author  : Aditya Shirwatkar
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "serial3r_kinematics.h"

namespace stochlite_control
{
	/* Class that wraps around the libTrajectoryGenerator for kinematics
	*
	* This class implements the kinematics (forward and inverse) for the Stochlite Robot
	*/
  	class StochliteKinematics {

		protected:
			const double BODY_LENGTH = 0.38;  // hip-to-hip length of the body (in metres)
			const double BODY_WIDTH  = 0.24;  // hip-to-hip width of the body (in metres) 

			const double ABD_LEN    = 0.096; // length of abduction link (metres)
			const double THIGH_LEN  = 0.146; // length of thigh link (metres)
			const double SHANK_LEN  = 0.172 ; // length of shank link (metres)

			const double BASE_MASS = 1.582;
			const double ABD_MASS = 0.037;
			const double THIGH_MASS = 0.32;
			const double SHANK_MASS = 0.055;

			const double LEG_MASS = ABD_MASS + THIGH_MASS + SHANK_MASS;
			const double ROBOT_MASS = LEG_MASS*4 + BASE_MASS;
			
			char branch_fl, branch_fr, branch_bl, branch_br; // leg configuration resulting from ik, ">" or "<"

			const int num_legs = 4;
			const int num_motors_per_leg = 3;

			Serial3RKinematics *serial_3r; // serial3r object for using its functions
		
		public:
			/* Constructor for Class StochliteKinematics
			*
			* \param[in] front: '>' or '<' leg configuration, default is '>' 
			* \param[in] back: '>' or '<' leg configuration, default is '>' 
			*/
			StochliteKinematics(char front, char back);

			std::vector<std::string> fl_joint_names;
			std::vector<std::string> fr_joint_names;
			std::vector<std::string> bl_joint_names;
			std::vector<std::string> br_joint_names;

			/* 3D rotation about the X-axis.
			*
			* This function performs a rotation on the input vector about the
			* x-axis by a specified angle.
			*
			* \param[in] angle : Angle of rotation about the x-axis.
			*
			* \param[in] vec_in : 3D input vector.
			*
			* \return : Returns the 3D vector which is rotated about the x-axis.
			*/
			std::vector<double> rotX(double angle, std::vector<double> vec_in)
			{
			assert(vec_in.size() == 3);

			std::vector<double> vec_out;
			vec_out.resize(3);

			vec_out[0] = vec_in[0];
			vec_out[1] = cos(angle) * vec_in[1] - sin(angle) * vec_in[2];
			vec_out[2] = sin(angle) * vec_in[1] + cos(angle) * vec_in[2];

			return vec_out;
			}

			/* Function to get a vector of strings of joints names in the following order, 
			*  FL_abd, FL_hip, FL_knee, FR_abd, FR_hip, ....
			*/
    		std::vector<std::string> getJointNames();

			/* \param[in] target_foot_positions: a vector of foot_positions = (FL_x, FL_y, FL_z, FR_x, ..., BR_z) 
			*  \param[in] target_joint_positions: a reference to the vector of joint_positions = (FL_abd, FL_hip, FL_knee, FR_abd, ..., BR_knee)
			*  \return an integer to indicate if ik output is valid or not (0 means good output, else something is wrong)
			*/
			int inverseKinematics(std::vector<double> target_foot_positions, std::vector<double>& target_joint_positions) const;
			
			/* \param[in] target_joint_positions: a vector of joint_positions = (FL_abd, FL_hip, FL_knee, FR_abd, ..., BR_knee) 
			*  \param[in] target_foot_positions: a reference to a vector of foot_positions = (FL_x, FL_y, FL_z, FR_x, ..., BR_z) 
			*/
    		void forwardKinematics(std::vector<double> target_joint_positions, std::vector<double>& target_foot_positions) const;

			void getCoM(std::vector<double> target_joint_positions, std::vector<double>& robot_com);

  	};

	StochliteKinematics::StochliteKinematics(char front = char('>'), char back = char('>')) {
		
		branch_fl = front;
		branch_fr = front;
		branch_bl = back;
		branch_br = back;

		serial_3r = new Serial3RKinematics({ABD_LEN, THIGH_LEN, SHANK_LEN});

	}

	std::vector<std::string> StochliteKinematics::getJointNames() {
		
		std::vector<std::string> joint_names;
		std::vector<std::string> temp;

		for (size_t i = 0; i < 4; i++)
		{
			if (i == 0) {temp = fl_joint_names;}
			if (i == 1) {temp = fr_joint_names;}
			if (i == 2) {temp = bl_joint_names;}
			if (i == 3) {temp = br_joint_names;}
			
			for (size_t j = 0; j < 3; j++)
			{
				joint_names.push_back(temp[j]);
			}
			
		}	
      
		return joint_names;

	}

  	void StochliteKinematics::forwardKinematics(std::vector<double> target_joint_positions, std::vector<double>& target_foot_positions) const {
		
		std::vector<double> fl_pos (3*1); std::vector<double> fl_ang (1*3);
		std::vector<double> fr_pos (3*1); std::vector<double> fr_ang (1*3);
		std::vector<double> bl_pos (3*1); std::vector<double> bl_ang (1*3);
		std::vector<double> br_pos (3*1); std::vector<double> br_ang (1*3);

		fl_ang = std::vector<double>(target_joint_positions.begin() + 0 + 0, target_joint_positions.begin() + 3 + 0);
		fr_ang = std::vector<double>(target_joint_positions.begin() + 0 + 3, target_joint_positions.begin() + 3 + 3);
		bl_ang = std::vector<double>(target_joint_positions.begin() + 0 + 6, target_joint_positions.begin() + 3 + 6);
		br_ang = std::vector<double>(target_joint_positions.begin() + 0 + 9, target_joint_positions.begin() + 3 + 9);

		serial_3r->forwardKinematics("fl",fl_ang, fl_pos);
		serial_3r->forwardKinematics("fr",fr_ang, fr_pos);
		serial_3r->forwardKinematics("bl",bl_ang, bl_pos);
		serial_3r->forwardKinematics("br",br_ang, br_pos);

		target_foot_positions[0 + 0] = fl_pos[0]; target_foot_positions[0 + 3] = fr_pos[0];
		target_foot_positions[1 + 0] = fl_pos[1]; target_foot_positions[1 + 3] = fr_pos[1];
		target_foot_positions[2 + 0] = fl_pos[2]; target_foot_positions[2 + 3] = fr_pos[2];

		target_foot_positions[0 + 6] = bl_pos[0]; target_foot_positions[0 + 9] = br_pos[0];
		target_foot_positions[1 + 6] = bl_pos[1]; target_foot_positions[1 + 9] = br_pos[1];
		target_foot_positions[2 + 6] = bl_pos[2]; target_foot_positions[2 + 9] = br_pos[2];

	}

	int StochliteKinematics::inverseKinematics(std::vector<double> target_foot_positions, std::vector<double>& target_joint_positions) const {

		// FL FR BL BR
		std::vector<double> fl_pos (3*1); std::vector<double> fl_ang (1*3);
		std::vector<double> fr_pos (3*1); std::vector<double> fr_ang (1*3);
		std::vector<double> bl_pos (3*1); std::vector<double> bl_ang (1*3);
		std::vector<double> br_pos (3*1); std::vector<double> br_ang (1*3);

		fl_pos = std::vector<double>(target_foot_positions.begin() + 0 + 0, target_foot_positions.begin() + 3 + 0);
		fr_pos = std::vector<double>(target_foot_positions.begin() + 0 + 3, target_foot_positions.begin() + 3 + 3);
		bl_pos = std::vector<double>(target_foot_positions.begin() + 0 + 6, target_foot_positions.begin() + 3 + 6);
		br_pos = std::vector<double>(target_foot_positions.begin() + 0 + 9, target_foot_positions.begin() + 3 + 9);
		

		int valid_ik_fl = serial_3r->inverseKinematics("fl", fl_pos, branch_fl, fl_ang);
		int valid_ik_fr = serial_3r->inverseKinematics("fr", fr_pos, branch_fr, fr_ang);
		int valid_ik_bl = serial_3r->inverseKinematics("bl", bl_pos, branch_bl, bl_ang);
		int valid_ik_br = serial_3r->inverseKinematics("br", br_pos, branch_br, br_ang);

		target_joint_positions[0 + 0] = fl_ang[0]; target_joint_positions[0 + 3] = fr_ang[0];
		target_joint_positions[1 + 0] = fl_ang[1]; target_joint_positions[1 + 3] = fr_ang[1];
		target_joint_positions[2 + 0] = fl_ang[2]; target_joint_positions[2 + 3] = fr_ang[2];
		                                                                                     
		target_joint_positions[0 + 6] = bl_ang[0]; target_joint_positions[0 + 9] = br_ang[0];
		target_joint_positions[1 + 6] = bl_ang[1]; target_joint_positions[1 + 9] = br_ang[1];
		target_joint_positions[2 + 6] = bl_ang[2]; target_joint_positions[2 + 9] = br_ang[2];

		int valid_ik = (valid_ik_fl + valid_ik_fr + valid_ik_bl + valid_ik_br);

		return valid_ik;
    }

	void StochliteKinematics::getCoM(std::vector<double> target_joint_positions, std::vector<double>& robot_com) {

		std::vector<double> com_shank (3);
		std::vector<double> com_thigh (3);
		std::vector<double> com_abd (3);

		std::vector<double> fl_com (3*1); std::vector<double> fl_ang (1*3);
		std::vector<double> fr_com (3*1); std::vector<double> fr_ang (1*3);
		std::vector<double> bl_com (3*1); std::vector<double> bl_ang (1*3);
		std::vector<double> br_com (3*1); std::vector<double> br_ang (1*3);

		fl_ang = std::vector<double>(target_joint_positions.begin() + 0 + 0, target_joint_positions.begin() + 3 + 0);
		fr_ang = std::vector<double>(target_joint_positions.begin() + 0 + 3, target_joint_positions.begin() + 3 + 3);
		bl_ang = std::vector<double>(target_joint_positions.begin() + 0 + 6, target_joint_positions.begin() + 3 + 6);
		br_ang = std::vector<double>(target_joint_positions.begin() + 0 + 9, target_joint_positions.begin() + 3 + 9);

		double l1, l2, l3;

		l1 = ABD_LEN/2;
		l2 = THIGH_LEN/2;
		l3 = SHANK_LEN/2;

		// FL CoM
		
		com_abd[0] = 0;
		com_abd[1] = -l1;
		com_abd[2] = 0;
		com_abd = rotX(fl_ang[0], com_abd);

		com_thigh[0] = -l2*sin(fl_ang[1]);
		com_thigh[1] = -2*l1;
		com_thigh[2] = -l2*cos(fl_ang[1]);
		com_thigh = rotX(fl_ang[0], com_thigh);

		com_shank[0] = 2*-l2*sin(fl_ang[1]) + -l3*sin(fl_ang[1] + fl_ang[1]);
		com_shank[1] = -2*l1;
		com_shank[2] = 2*-l2*cos(fl_ang[1]) + -l3*cos(fl_ang[1] + fl_ang[1]);
		com_shank = rotX(fl_ang[0], com_shank);

		fl_com[0] = (ABD_MASS * com_abd[0] + THIGH_MASS * com_thigh[0] + SHANK_MASS * com_shank[0])/LEG_MASS;
		fl_com[1] = (ABD_MASS * com_abd[1] + THIGH_MASS * com_thigh[1] + SHANK_MASS * com_shank[1])/LEG_MASS;
		fl_com[2] = (ABD_MASS * com_abd[2] + THIGH_MASS * com_thigh[2] + SHANK_MASS * com_shank[2])/LEG_MASS;

		// FR CoM
		
		com_abd[0] = 0;
		com_abd[1] = l1;
		com_abd[2] = 0;
		com_abd = rotX(fr_ang[0], com_abd);

		com_thigh[0] = -l2*sin(fr_ang[1]);
		com_thigh[1] = 2*l1;
		com_thigh[2] = -l2*cos(fr_ang[1]);
		com_thigh = rotX(fr_ang[0], com_thigh);

		com_shank[0] = 2*-l2*sin(fr_ang[1]) + -l3*sin(fr_ang[1] + fr_ang[1]);
		com_shank[1] = 2*l1;
		com_shank[2] = 2*-l2*cos(fr_ang[1]) + -l3*cos(fr_ang[1] + fr_ang[1]);
		com_shank = rotX(fr_ang[0], com_shank);

		fr_com[0] = (ABD_MASS * com_abd[0] + THIGH_MASS * com_thigh[0] + SHANK_MASS * com_shank[0])/LEG_MASS;
		fr_com[1] = (ABD_MASS * com_abd[1] + THIGH_MASS * com_thigh[1] + SHANK_MASS * com_shank[1])/LEG_MASS;
		fr_com[2] = (ABD_MASS * com_abd[2] + THIGH_MASS * com_thigh[2] + SHANK_MASS * com_shank[2])/LEG_MASS;

		// BL CoM
		
		com_abd[0] = 0;
		com_abd[1] = -l1;
		com_abd[2] = 0;
		com_abd = rotX(bl_ang[0], com_abd);

		com_thigh[0] = -l2*sin(bl_ang[1]);
		com_thigh[1] = -2*l1;
		com_thigh[2] = -l2*cos(bl_ang[1]);
		com_thigh = rotX(bl_ang[0], com_thigh);

		com_shank[0] = 2*-l2*sin(bl_ang[1]) + -l3*sin(bl_ang[1] + bl_ang[1]);
		com_shank[1] = -2*l1;
		com_shank[2] = 2*-l2*cos(bl_ang[1]) + -l3*cos(bl_ang[1] + bl_ang[1]);
		com_shank = rotX(bl_ang[0], com_shank);

		bl_com[0] = (ABD_MASS * com_abd[0] + THIGH_MASS * com_thigh[0] + SHANK_MASS * com_shank[0])/LEG_MASS;
		bl_com[1] = (ABD_MASS * com_abd[1] + THIGH_MASS * com_thigh[1] + SHANK_MASS * com_shank[1])/LEG_MASS;
		bl_com[2] = (ABD_MASS * com_abd[2] + THIGH_MASS * com_thigh[2] + SHANK_MASS * com_shank[2])/LEG_MASS;

		// BR CoM
		
		com_abd[0] = 0;
		com_abd[1] = l1;
		com_abd[2] = 0;
		com_abd = rotX(br_ang[0], com_abd);

		com_thigh[0] = -l2*sin(br_ang[1]);
		com_thigh[1] = 2*l1;
		com_thigh[2] = -l2*cos(br_ang[1]);
		com_thigh = rotX(br_ang[0], com_thigh);

		com_shank[0] = 2*-l2*sin(br_ang[1]) + -l3*sin(br_ang[1] + br_ang[1]);
		com_shank[1] = 2*l1;
		com_shank[2] = 2*-l2*cos(br_ang[1]) + -l3*cos(br_ang[1] + br_ang[1]);
		com_shank = rotX(br_ang[0], com_shank);

		br_com[0] = (ABD_MASS * com_abd[0] + THIGH_MASS * com_thigh[0] + SHANK_MASS * com_shank[0])/LEG_MASS;
		br_com[1] = (ABD_MASS * com_abd[1] + THIGH_MASS * com_thigh[1] + SHANK_MASS * com_shank[1])/LEG_MASS;
		br_com[2] = (ABD_MASS * com_abd[2] + THIGH_MASS * com_thigh[2] + SHANK_MASS * com_shank[2])/LEG_MASS;

		// Robot CoM

		robot_com[0] = LEG_MASS*(0 + fl_com[0] + fr_com[0] + bl_com[0] + br_com[0])/ROBOT_MASS; // 0 represents the base CoM
		robot_com[1] = LEG_MASS*(0 + fl_com[1] + fr_com[1] + bl_com[1] + br_com[1])/ROBOT_MASS; // 0 represents the base CoM
		robot_com[2] = LEG_MASS*(0 + fl_com[2] + fr_com[2] + bl_com[2] + br_com[2])/ROBOT_MASS; // 0 represents the base CoM

	}

}

#endif /* __STOCHLITE_KINEMATICS__ */
