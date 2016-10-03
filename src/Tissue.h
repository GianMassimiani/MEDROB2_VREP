#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/LU>
#include <math.h>

struct Layer
{
	std::string _name;
	float	_thick;
	float	_K;
	float	_B;
	bool	_is_perforated;
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

	void setLayer(std::string name, float t, float k, float b);
	void getLayerParams(std::string name, float out_t, float out_k, float out_b);
	bool checkPerforation(std::string name);

private:
	void initLayer(Layer& l, std::string name, float t, float k, float b);

	std::vector<Layer> _layers;
	int _N;

};
