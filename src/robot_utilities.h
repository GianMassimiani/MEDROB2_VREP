#pragma once
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "luaFunctionData.h"
#include "v_repLib.h"
#include "chai3d.h"
#include "utility.h"

Matrix4f linkCoordTransform(float a, float alpha, float d, float theta);
Matrix4f cinematicaDiretta(VectorXf a_vec, VectorXf alpha_vec, VectorXf d_vec, VectorXf joint_coord);
Matrix4f cinematicaDiretta(VectorXf a_vec, VectorXf alpha_vec, VectorXf d_vec, VectorXf joint_coord, int i_pos);

MatrixXf jacobianoGeometrico(VectorXf a_vec, VectorXf alpha_vec, VectorXf d_vec, VectorXf joint_coord);
void setDHParameter(int DOF_number, VectorXf& a_vec, VectorXf& alpha_vec, VectorXf& d_vec);
MatrixXf LWRGeometricJacobian(std::vector<float>& lwr_joint_q);

void computeNullSpaceVelocity(VectorXf& config_q_dot,
	VectorXf& des_vel, VectorXf& des_pose, VectorXf& curr_pose,
	MatrixXf& J, MatrixXf& Kp);


void computeDLSVelocity(VectorXf& config_q_dot,
	VectorXf& des_vel, VectorXf& des_pose, VectorXf& curr_pose,
	MatrixXf& J, MatrixXf Kp, float mu);