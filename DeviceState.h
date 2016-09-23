#pragma once
#include <iostream>
#include <Eigen/LU>
#include <math.h>
class DeviceState
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	DeviceState();
	~DeviceState();
	
	inline float* getSimPos(void) { return eigen2SimFloat(pos); };
	inline float* getSimVel(void) { return eigen2SimFloat(vel); };
	inline float** getSimRot(void) { return eigen2SimMatrix(rot); };

	float* getEulerAngles(void);

	void print(void);

	Eigen::Vector3f pos;
	Eigen::Vector3f vel;
	Eigen::Matrix3f rot;

protected:
	float* eigen2SimFloat(const Eigen::Vector3f eigen);
	float** eigen2SimMatrix(const Eigen::Matrix3f eigen);
};

