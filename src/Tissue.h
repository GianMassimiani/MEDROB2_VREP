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
	bool	_touched;

	int		_handler;
	int		_static_handler;
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
	bool checkPerforation(std::string name);
	void togglePerforation(std::string name);
	bool checkTouched(std::string name);
	void toggleTouched(std::string name);
	void renderLayers(void);
	void resetRendering(void);
	void reloadLayer(std::string name);
	void restoreLayers(int current_layer_idx);
	void removeDynamicLayer(std::string name);
	int getLayerHandler(std::string name, bool type);

	float getDOP(Eigen::Vector3f start_p,
		Eigen::Vector3f final_p,
		Eigen::Vector3f contact_n);
	int getLayerIDXFromDepth(void);

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
	std::vector<int> _static_cube_handlers;
	std::vector<Layer> _layers;
	int _N;

};
