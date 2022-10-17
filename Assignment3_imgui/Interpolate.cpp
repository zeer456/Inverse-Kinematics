#include "Interpolate.h"

namespace Assignment3_imgui
{
	Curve::Curve()
		: _steps(100)
	{

	}

	Curve::~Curve()
	{
	}

	void Curve::clear()
	{
		_nodes.clear();
		_way_points.clear();
		_distances.clear();
	}

	void Curve::add_way_point(const glm::vec3& point)
	{
		_way_points.push_back(point);
		if (_way_points.size() < 4)
		{
			return;
		}

		int new_control_point_index = _way_points.size() - 1;
		int pt = new_control_point_index - 2;
		for (int i = 0; i <= _steps; i++)
		{
			double u = (double)i / (double)_steps;

			add_node(Interpolate::CatmullRom(u, _way_points[pt - 1], _way_points[pt], _way_points[pt + 1], _way_points[pt + 2]));
		}
	}

	void Curve::add_node(const glm::vec3& node)
	{
		_nodes.push_back(node);



		if (_nodes.size() == 1)
		{
			_distances.push_back(0);
		}
		else
		{
			int new_node_index = _nodes.size() - 1;

			double segment_distance = (_nodes[new_node_index] - _nodes[new_node_index - 1]).length();
			_distances.push_back(segment_distance + _distances[new_node_index - 1]);
		}
	}


	Interpolate::Interpolate()
	{
	}


	Interpolate::~Interpolate()
	{
	}

	glm::vec3 Interpolate::CatmullRom(float u, const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& P2, const glm::vec3& P3)
	{
		glm::vec3 point;
		point = u * u* u * ((-1.0f) * P0 + 3.0f * P1 - 3.0f * P2 + P3) / 2.0f;
		point += u * u * (2.0f * P0 - 5.0f * P1 + 4.0f * P2 - P3) / 2.0f;
		point += u * ((-1.0f) * P0 + P2) / 2.0f;
		point += P1;

		return point;
	}

}