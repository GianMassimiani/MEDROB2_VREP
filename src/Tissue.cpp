#include "Tissue.h"

using namespace std;

Tissue::Tissue()
{
	_N = 1;
	for (int i = 0; i < _N; i++)
		initLayer(_layers[i], "L_" + to_string(i), 0, 0, 0);
}

Tissue::Tissue(std::vector<std::string>& name_vec, 
	std::vector<float>& thickness_vec, 
	std::vector<float>& K_vec, 
	std::vector<float>& B_vec)
{
	_N = name_vec.size();
	int t_size = thickness_vec.size();
	int k_size = K_vec.size();
	int b_size = B_vec.size();

	if (_N != t_size || _N != b_size || _N != k_size)
	{
		cerr << "Error! Input vectors have different sizes" << endl;
		return;
	}

	for (int i = 0; i < _N; i++)
		initLayer(_layers[i], name_vec[i],thickness_vec[i], K_vec[i], B_vec[i]);
}

Tissue::~Tissue()
{
}
void Tissue::initLayer(Layer& l, std::string name,float t, float k, float b)
{
	l._thick = t;
	l._name = name;
	l._K = k;
	l._B = b;
	l._is_perforated = false;
}

void Tissue::setLayer(std::string name, float t, float k, float b)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		cerr << "Error, no such tissue layer!" << endl;
		return;
	}

	initLayer(_layers[idx], name, t, k, b);
}

void Tissue::getLayerParams(std::string name, float out_t, float out_k, float out_b)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		cerr << "Error, no such tissue layer!" << endl;
		return;
	}

	out_t = _layers[idx]._thick;
	out_k = _layers[idx]._K;
	out_b = _layers[idx]._B;
}

bool Tissue::checkPerforation(std::string name)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		cerr << "Error, no such tissue layer!" << endl;
		return false;
	}

	if (_layers[idx]._is_perforated)
		return true;
	else
		return false;
}

/*
tissue_params[0] <<
1.0,
331,
3;
tissue_params[1] <<
1.0,
83,
1;
tissue_params[2] <<
1.0,
497,
3;
tissue_params[3] <<
1.0,
2483,
0;
*/
