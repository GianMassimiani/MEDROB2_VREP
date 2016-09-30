#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/LU>
#include <math.h>

struct Layer
{
	std::string name;
	float	thick;
	float	K;
	float	B;
	bool	is_perforated;
};

class Tissue
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Tissue();
	Tissue(std::vector<std::string>& name_vec, 
		std::vector<float>& thickness_vec, 
		std::vector<float>& K_vec, 
		std::vector<float>& B_vec);
	~Tissue();

private:
	void initLayer(Layer& l, std::string name, float t, float k, float b);

	std::vector<Layer> _layers;
	int _N;

};
