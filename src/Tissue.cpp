#include "Tissue.h"

using namespace std;

Tissue::Tissue()
{
	_N = 0;
	_center_pos.setZero();
}

Tissue::~Tissue()
{
	// placeholder
}


void Tissue::addLayer(std::string name, float t, float k, float b)
{
	Layer temp_l;
	temp_l._thick = t;
	temp_l._name = name;
	temp_l._K = k;
	temp_l._B = b;
	temp_l._is_perforated = false;

	_layers.push_back(temp_l);
	_N++;

	// ADD CUBES HERE
}

void Tissue::getLayerParams(std::string name, float& out_t, float& out_k, float& out_b)
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

void Tissue::printTissue(void)
{
	for (int i = 0; i < _N; i++)
	{
		cout << "Layer " + _layers[i]._name << endl;
		cout << "Thickness:\t" << _layers[i]._thick << endl;
		cout << "K:\t" << _layers[i]._K << endl;
		cout << "B:\t" << _layers[i]._B << endl;
		cout << "Is perforated:\t" << _layers[i]._is_perforated << endl << endl;
	}
}

void Tissue::tooglePerforation(std::string name)
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
	_layers[idx]._is_perforated = !_layers[idx]._is_perforated;
}

void Tissue::renderLayers(void)
{
	std::vector<int> cube_handlers;
	float depth = _center_pos(2);
	Vector3f curr_cube_pos;
	float sim_curr_cube_pos[3];
	for (int i = 0; i < _N; i++)
	{
		if (i == 0)
			depth += (_layers[i]._thick / 2);
		depth += (_layers[i - 1]._thick / 2) + (_layers[i]._thick / 2);
		curr_cube_pos << _center_pos(0),
			_center_pos(1), depth;
		eigen2SimVec3f(curr_cube_pos, sim_curr_cube_pos);
		float tmp[3] = { _scale[0], _scale[1], _layers[i]._thick };
		cube_handlers.push_back(simCreatePureShape(0, 17, tmp, 1.0, NULL));
		
		simSetObjectPosition(cube_handlers[i], -1, sim_curr_cube_pos);
	}
	simGroupShapes(cube_handlers.data(), _N);
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
