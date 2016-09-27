#include "utility.h"


void printSimTransform(const float* sim_T)
{
	for (int i = 0; i < 12; i++)
	{
		if (i % 4 == 0 && i != 0)
			cout << "\t" << endl;
		cout << sim_T[i] << "\t";
	}
	cout << endl;
}


void printSimMatrix(const float* sim_M, const int r, const int c)
{
	for (int i = 0; i < r*c; i++)
	{
		if (i % c == 0 && i != 0)
			cout << "\t" << endl;
		cout << sim_M[i] << "\t";
	}
	cout << endl;
}

// This function takes a Eigen Matrix4f as input and returns a 
// float[12] array (since the last row [0 0 0 1] is not taken 
// in consideration).
void eigen2SimTransf(const Matrix4f& in, float* out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			out[4 * i + j] = in(i, j);
		}
	}
}

void sim2EigenTransf(const float* in, Matrix4f& out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			out(i, j) = in[4 * i + j];
		}
	}
	out.block<1, 4>(3, 0) << 0, 0, 0, 1;
}

void eigen2SimRot(const Matrix3f& in, float* out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out[3 * i + j] = in(i, j);
		}
	}
}

void sim2EigenRot(const float* in, Matrix3f& out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out(i, j) = in[3 * i + j];
		}
	}
}

void eigen2SimVec3f(const Vector3f& in, float* out) 
{
	out[0] = in(0);
	out[1] = in(1);
	out[2] = in(2);
}
void sim2EigenVec3f(const float* in, Vector3f& out)
{
	out(0) = in[0];
	out(1) = in[1];
	out(2) = in[2];
}

void simComposeTransform(const float* in_rot, const float* in_pos, float* out_T)
{
	Matrix4f temp_T;
	Matrix3f temp_R;
	Vector3f temp_P;

	sim2EigenRot(in_rot, temp_R);
	sim2EigenVec3f(in_pos, temp_P);

	temp_T.setIdentity();
	temp_T.block<3, 3>(0, 0) = temp_R;
	temp_T.block<3, 1>(0, 3) = temp_P;

	eigen2SimTransf(temp_T, out_T);
}

void simDecomposeTransform(const float* in_T,  float* out_rot, float* out_pos)
{
	Matrix4f temp_T;
	Matrix3f temp_R;
	Vector3f temp_P;

	sim2EigenTransf(in_T, temp_T);
	temp_R = temp_T.block<3, 3>(0, 0);
	temp_P = temp_T.block<3, 1>(0, 3);

	eigen2SimRot(temp_R, out_rot);
	eigen2SimVec3f(temp_P, out_pos);
}

void simMultiplyVec3fByScalar(float* v, float k)
{
	v[0] = v[0] * k;
	v[1] = v[1] * k;
	v[2] = v[2] * k;
}