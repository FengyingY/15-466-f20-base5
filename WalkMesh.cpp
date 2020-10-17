#include "WalkMesh.hpp"

#include "read_write_chunk.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

WalkMesh::WalkMesh(std::vector< glm::vec3 > const &vertices_, std::vector< glm::vec3 > const &normals_, std::vector< glm::uvec3 > const &triangles_)
	: vertices(vertices_), normals(normals_), triangles(triangles_) {

	//construct next_vertex map (maps each edge to the next vertex in the triangle):
	next_vertex.reserve(triangles.size()*3);
	auto do_next = [this](uint32_t a, uint32_t b, uint32_t c) {
		auto ret = next_vertex.insert(std::make_pair(glm::uvec2(a,b), c));
		assert(ret.second);
	};
	for (auto const &tri : triangles) {
		do_next(tri.x, tri.y, tri.z);
		do_next(tri.y, tri.z, tri.x);
		do_next(tri.z, tri.x, tri.y);
	}

	//DEBUG: are vertex normals consistent with geometric normals?
	for (auto const &tri : triangles) {
		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];
		glm::vec3 out = glm::normalize(glm::cross(b-a, c-a));

		float da = glm::dot(out, normals[tri.x]);
		float db = glm::dot(out, normals[tri.y]);
		float dc = glm::dot(out, normals[tri.z]);

		assert(da > 0.1f && db > 0.1f && dc > 0.1f);
	}
}

//project pt to the plane of triangle a,b,c and return the barycentric weights of the projected point:
glm::vec3 barycentric_weights(glm::vec3 const &a, glm::vec3 const &b, glm::vec3 const &c, glm::vec3 const &pt) {
	glm::vec3 v0 = b - a, v1 = c - a, v2 = pt - a;
	
	double d00 = glm::dot(v0, v0);
	double d01 = glm::dot(v0, v1);
	double d11 = glm::dot(v1, v1);
	double d20 = glm::dot(v2, v0);
	double d21 = glm::dot(v2, v1);
	double denom = d00 * d11 - d01 * d01;
	double v = (d11 * d20 - d01 * d21) / denom;
	double w = (d00 * d21 - d01 * d20) / denom;
	double u = 1.0 - v - w;
	
	return glm::vec3(u, v, w);
}

WalkPoint WalkMesh::nearest_walk_point(glm::vec3 const &world_point) const {
	assert(!triangles.empty() && "Cannot start on an empty walkmesh");

	WalkPoint closest;
	float closest_dis2 = std::numeric_limits< float >::infinity();

	for (auto const &tri : triangles) {
		//find closest point on triangle:

		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];

		//get barycentric coordinates of closest point in the plane of (a,b,c):
		glm::vec3 coords = barycentric_weights(a,b,c, world_point);

		//is that point inside the triangle?
		if (coords.x >= 0.0f && coords.y >= 0.0f && coords.z >= 0.0f) {
			//yes, point is inside triangle.
			float dis2 = glm::length2(world_point - to_world_point(WalkPoint(tri, coords)));
			if (dis2 < closest_dis2) {
				closest_dis2 = dis2;
				closest.indices = tri;
				closest.weights = coords;
			}
		} else {
			//check triangle vertices and edges:
			auto check_edge = [&world_point, &closest, &closest_dis2, this](uint32_t ai, uint32_t bi, uint32_t ci) {
				glm::vec3 const &a = vertices[ai];
				glm::vec3 const &b = vertices[bi];

				//find closest point on line segment ab:
				float along = glm::dot(world_point-a, b-a);
				float max = glm::dot(b-a, b-a);
				glm::vec3 pt;
				glm::vec3 coords;
				if (along < 0.0f) {
					pt = a;
					coords = glm::vec3(1.0f, 0.0f, 0.0f);
				} else if (along > max) {
					pt = b;
					coords = glm::vec3(0.0f, 1.0f, 0.0f);
				} else {
					float amt = along / max;
					pt = glm::mix(a, b, amt);
					coords = glm::vec3(1.0f - amt, amt, 0.0f);
				}

				float dis2 = glm::length2(world_point - pt);
				if (dis2 < closest_dis2) {
					closest_dis2 = dis2;
					closest.indices = glm::uvec3(ai, bi, ci);
					closest.weights = coords;
				}
			};
			check_edge(tri.x, tri.y, tri.z);
			check_edge(tri.y, tri.z, tri.x);
			check_edge(tri.z, tri.x, tri.y);
		}
	}
	assert(closest.indices.x < vertices.size());
	assert(closest.indices.y < vertices.size());
	assert(closest.indices.z < vertices.size());
	return closest;
}


void WalkMesh::walk_in_triangle(WalkPoint const &start, glm::vec3 const &step, WalkPoint *end_, float *time_) const {
	assert(end_);
	auto &end = *end_;

	assert(time_);
	auto &time = *time_;

	glm::vec3 step_coords;
	{ //project 'step' into a barycentric-coordinates direction:
		//TODO
		glm::vec3 const &a = vertices[start.indices.x];
		glm::vec3 const &b = vertices[start.indices.y];
		glm::vec3 const &c = vertices[start.indices.z];
		glm::vec3 start_pos = start.weights.x * a + start.weights.y * b + start.weights.z * c;
		step_coords = barycentric_weights(a, b, c, start_pos + step);
	}
	
	//if no edge is crossed, event will just be taking the whole step:
	time = 1.0f;
	end = start;

	glm::vec3 barycentric_v = step_coords - start.weights;
	int zero_index = -1;
	//figure out which edge (if any) is crossed first.
	// set time and end appropriately.

	double time_x = - start.weights.x / barycentric_v.x;
	double time_y = - start.weights.y / barycentric_v.y;
	double time_z = - start.weights.z / barycentric_v.z;
	if (time_x > 0.0 && time_x < time) {
		time = time_x;
		zero_index = 0;
	}
	if (time_y > 0.0 && time_y < time) {
		time = time_y;
		zero_index = 1;
	}
	if (time_z > 0.0 && time_z < time) {
		time = time_z;
		zero_index = 2;
	}
	//TODO
	// update the weights
	end.weights = start.weights + barycentric_v * time;

	//Remember: our convention is that when a WalkPoint is on an edge,
	// then wp.weights.z == 0.0f (so will likely need to re-order the indices)
	int indx = end.indices.x, indy = end.indices.y, indz = end.indices.z;
	if (zero_index == 0) { // y, z, x
		end.weights.x = end.weights.y;
		end.weights.y = end.weights.z;
		end.weights.z = 0.0;
		end.indices.x = indy;
		end.indices.y = indz;
		end.indices.z = indx;
	} else if (zero_index == 1) { // z, x, y
		end.weights.y = end.weights.x;
		end.weights.x = end.weights.z;
		end.weights.z = 0.0;
		end.indices.x = indz;
		end.indices.y = indx;
		end.indices.z = indy;
	} else if (zero_index == 2) {
		end.weights.z = 0.0;
	}
}

bool WalkMesh::cross_edge(WalkPoint const &start, WalkPoint *end_, glm::quat *rotation_) const {
	assert(end_);
	auto &end = *end_;

	assert(rotation_);
	auto &rotation = *rotation_;

	assert(start.weights.z == 0.0f); //*must* be on an edge.
	glm::uvec2 edge = glm::uvec2(start.indices.y, start.indices.x);

	//check if 'edge' is a non-boundary edge:
	auto v_it = WalkMesh::next_vertex.find(edge);
	if (v_it != WalkMesh::next_vertex.end()) {
		//it is!
		glm::vec3 const &sa = vertices[start.indices.x];
		glm::vec3 const &sb = vertices[start.indices.y];
		glm::vec3 const &sc = vertices[start.indices.z];
		glm::vec3 const &a = vertices[start.indices.y];
		glm::vec3 const &b = vertices[start.indices.x];
		glm::vec3 const &c = vertices[v_it->second];

		//make 'end' represent the same (world) point, but on triangle (edge.y, edge.x, [other point]):
		//TODO
		glm::vec3 pt = start.weights.x * sa + start.weights.y * sb + start.weights.z * sc;
		end.weights = barycentric_weights(a, b, c, pt);

		//make 'rotation' the rotation that takes (start.indices)'s normal to (end.indices)'s normal:
		//TODO
		rotation = glm::rotation(
			//glm::normalize(glm::cross(sb-sa, sc-sa)), 
			//glm::normalize(glm::cross(b-a, c-a))
			glm::cross(WalkMesh::normals[start.indices.y] - WalkMesh::normals[start.indices.x], WalkMesh::normals[start.indices.z] - WalkMesh::normals[start.indices.x]),
			glm::cross(WalkMesh::normals[start.indices.x] - WalkMesh::normals[start.indices.y], WalkMesh::normals[v_it->second] - WalkMesh::normals[start.indices.y])
		);

		return true;
	} else {
		end = start;
		rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		return false;
	}
}


WalkMeshes::WalkMeshes(std::string const &filename) {
	std::ifstream file(filename, std::ios::binary);

	std::vector< glm::vec3 > vertices;
	read_chunk(file, "p...", &vertices);

	std::vector< glm::vec3 > normals;
	read_chunk(file, "n...", &normals);

	std::vector< glm::uvec3 > triangles;
	read_chunk(file, "tri0", &triangles);

	std::vector< char > names;
	read_chunk(file, "str0", &names);

	struct IndexEntry {
		uint32_t name_begin, name_end;
		uint32_t vertex_begin, vertex_end;
		uint32_t triangle_begin, triangle_end;
	};

	std::vector< IndexEntry > index;
	read_chunk(file, "idxA", &index);

	if (file.peek() != EOF) {
		std::cerr << "WARNING: trailing data in walkmesh file '" << filename << "'" << std::endl;
	}

	//-----------------

	if (vertices.size() != normals.size()) {
		throw std::runtime_error("Mis-matched position and normal sizes in '" + filename + "'");
	}

	for (auto const &e : index) {
		if (!(e.name_begin <= e.name_end && e.name_end <= names.size())) {
			throw std::runtime_error("Invalid name indices in index of '" + filename + "'");
		}
		if (!(e.vertex_begin <= e.vertex_end && e.vertex_end <= vertices.size())) {
			throw std::runtime_error("Invalid vertex indices in index of '" + filename + "'");
		}
		if (!(e.triangle_begin <= e.triangle_end && e.triangle_end <= triangles.size())) {
			throw std::runtime_error("Invalid triangle indices in index of '" + filename + "'");
		}

		//copy vertices/normals:
		std::vector< glm::vec3 > wm_vertices(vertices.begin() + e.vertex_begin, vertices.begin() + e.vertex_end);
		std::vector< glm::vec3 > wm_normals(normals.begin() + e.vertex_begin, normals.begin() + e.vertex_end);

		//remap triangles:
		std::vector< glm::uvec3 > wm_triangles; wm_triangles.reserve(e.triangle_end - e.triangle_begin);
		for (uint32_t ti = e.triangle_begin; ti != e.triangle_end; ++ti) {
			if (!( (e.vertex_begin <= triangles[ti].x && triangles[ti].x < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].y && triangles[ti].y < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].z && triangles[ti].z < e.vertex_end) )) {
				throw std::runtime_error("Invalid triangle in '" + filename + "'");
			}
			wm_triangles.emplace_back(
				triangles[ti].x - e.vertex_begin,
				triangles[ti].y - e.vertex_begin,
				triangles[ti].z - e.vertex_begin
			);
		}
		
		std::string name(names.begin() + e.name_begin, names.begin() + e.name_end);

		auto ret = meshes.emplace(name, WalkMesh(wm_vertices, wm_normals, wm_triangles));
		if (!ret.second) {
			throw std::runtime_error("WalkMesh with duplicated name '" + name + "' in '" + filename + "'");
		}

	}
}

WalkMesh const &WalkMeshes::lookup(std::string const &name) const {
	auto f = meshes.find(name);
	if (f == meshes.end()) {
		throw std::runtime_error("WalkMesh with name '" + name + "' not found.");
	}
	return f->second;
}
