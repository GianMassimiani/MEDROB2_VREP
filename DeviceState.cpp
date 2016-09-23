#include "DeviceState.h"

using namespace std;
using namespace Eigen;

DeviceState::DeviceState()
{
	pos.setZero();
	vel.setZero();
	rot.setZero();
}


DeviceState::~DeviceState()
{
}

float* DeviceState::eigen2SimFloat(const Eigen::Vector3f eigen)
{
	float temp[3];
	temp[0] = eigen(0);
	temp[1] = eigen(1);
	temp[2] = eigen(2);

	return temp;
}

float** DeviceState::eigen2SimMatrix(const Eigen::Matrix3f eigen)
{
	float** temp;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			temp[i][j] = eigen(i, j);
		}
	}
	return temp;
}

void DeviceState::print(void)
{
	cout << "Position:" << endl;
	cout << pos << endl << endl;
	cout << "Velocity:" << endl;
	cout << vel << endl << endl;
	cout << "Rotation matrix:" << endl;
	cout << rot << endl << endl;
}

float* DeviceState::getEulerAngles(void)
{
	float euler_angles[3];
	euler_angles[0] = atan2(rot(2, 1), rot(2, 2));
	euler_angles[1] = asin(rot(2, 0));
	euler_angles[2] = -atan2(rot(1, 0), rot(0, 0));

	return euler_angles;
}