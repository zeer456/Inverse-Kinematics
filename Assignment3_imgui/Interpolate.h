#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace Assignment3_imgui
{
	class Curve
	{
	public:
		Curve();
		~Curve();

	protected:
		std::vector<glm::vec3> _way_points;

	public:
		void add_way_point(const glm::vec3& point);
		void clear();

	protected:
		void add_node(const glm::vec3& node);

	protected:
		std::vector<glm::vec3> _nodes;
		std::vector<double> _distances;

	public:
		glm::vec3 node(int i) const { return _nodes[i]; }
		double length_from_starting_point(int i) const { return _distances[i]; }
		bool has_next_node(int i) const { return static_cast<int>(_nodes.size()) > i; }
		int node_count() const { return static_cast<int>(_nodes.size()); }
		bool is_empty() { return _nodes.empty(); }
		double total_length() const
		{
			assert(!_distances.empty());
			return _distances[_distances.size() - 1];
		}

	protected:
		int _steps;

	public:
		void increment_steps(int steps) { _steps += steps; }
		void set_steps(int steps) { _steps = steps; }
	};


	class Interpolate
	{
	public:
		Interpolate();
		~Interpolate();
		static glm::vec3 CatmullRom(float u, const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& P2, const glm::vec3& P3);
	};

}