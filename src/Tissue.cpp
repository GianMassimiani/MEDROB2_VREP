#include "Tissue.h"

Tissue::Tissue()
{

	for (int i = 0; i < _N; i++)
		initLayer(_layers[i], "L", 0, 0, 0);
}
Tissue::Tissue(std::vector<std::string>& name_vec, 
	std::vector<float>& thickness_vec, 
	std::vector<float>& K_vec, 
	std::vector<float>& B_vec)
{
	for (int i = 0; i < _N; i++)
		initLayer(_layers[i], name_vec[i],thickness_vec[i], K_vec[i], B_vec[i]);
}

Tissue::~Tissue()
{
}
void Tissue::initLayer(Layer& l, std::string name,float t, float k, float b)
{
	l.thick = t;
	l.name = name;
	l.K = k;
	l.B = b;
	l.is_perforated = false;
}

