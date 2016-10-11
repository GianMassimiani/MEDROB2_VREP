#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/LU>
#include <math.h>

#include "luaFunctionData.h"
#include "v_repLib.h"
#include "utility.h"

struct Layer
{
	std::string _name;
	float	_thick;
	float	_K;
	float	_B;
	bool	_is_perforated;

	int		_handler;
};

class Tissue
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Tissue();
	~Tissue();

	void init(void);
	void addLayer(std::string name, float t, float k, float b);
	void getLayerParams(std::string name, float& out_t, float& out_k, float& out_b);
	int getLayerHandler(std::string name);
	bool checkPerforation(std::string name);
	void tooglePerforation(std::string name);
	void renderLayers(void);

	inline void setTissueCenter(Eigen::Vector3f c) { _center_pos = c; };
	inline void setScale(float h, float w) { _scale[0] = h; _scale[1] = w; };
	inline float getTotalDepth(void) { return _d; };

	void printTissue(void);

private:
	Eigen::Vector3f _center_pos;
	float _scale[2];
	float _d;

	int _dummy_rederer_handler;

	std::vector<int> _cube_handlers;
	std::vector<Layer> _layers;
	int _N;

};
