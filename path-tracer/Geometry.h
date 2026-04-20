#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray {
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray() {}

	Ray(Vec3 _o, Vec3 _d) {
		init(_o, _d);
	}

	void init(Vec3 _o, Vec3 _d) {
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}

	Vec3 at(const float t) const {
		return (o + (dir * t));
	}
};

class Plane {
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d) {
		n = _n;
		d = _d;
	}

	// Add code here
	bool rayIntersect(const Ray& r, float& t) const {
		float nDotOPlusD = -(n.dot(r.o) + d);
		float nDotDir = n.dot(r.dir);
		t = nDotOPlusD / nDotDir;

		return t >= 0;
	}
};

#define EPSILON 0.001f

class Triangle {
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float invArea;
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	Plane trianglePlane;
	Vec3 centroid;

	Triangle() {}

	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex) {
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		invArea = 1.0f / (area * 2.0f);
		d = Dot(n, vertices[0].p);
		trianglePlane.init(n, -d);
		centroid = centre();
	}

	Vec3 centre() const {
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}

	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const {
		float intersection;
		bool planeIntersect = trianglePlane.rayIntersect(r, intersection);

		if (!planeIntersect) {
			return false;
		}

		Vec3 intersectionPoint = r.at(intersection);
		Vec3 q1 = intersectionPoint - vertices[1].p;
		Vec3 c1 = e1.cross(q1);

		u = c1.dot(n) * invArea;

		Vec3 q2 = intersectionPoint - vertices[2].p;
		Vec3 c2 = e2.cross(q2);

		v = c2.dot(n) * invArea;

		t = intersection;

		if (u < 0 || u > 1) return false;
		if (v < 0 || v > 1) return false;

		return ((u + v) <= 1);
	}

	bool rayIntersectMollerTrumbore(const Ray& r, float& t, float& u, float& v) const {
		Vec3 p = r.dir.cross(e2);
		float det = -e1.dot(p);

		if (std::abs(det) < EPSILON) return false;

		Vec3 bigT = r.o - vertices[2].p;
		float beta = bigT.dot(p) / det;

		if (beta < 0 || beta > 1) return false;

		Vec3 q = bigT.cross(-e1);
		float gamma = r.dir.dot(q) / det;

		if (gamma < 0 || gamma > 1 || beta + gamma > 1) return false;

		t = e2.dot(q) / det;

		if (t < 0) return false;

		u = beta;
		v = gamma;

		return true;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const {
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}

	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf) {

		pdf = invArea;
		float e1 = sampler->next();
		float e2 = sampler->next();

		float sqrtE1 = sqrt(e1);
		float alpha = 1 - sqrtE1;
		float beta = e2 * sqrtE1;
		float gamma = 1 - (alpha + beta);

		return vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
	}

	Vec3 gNormal() {
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB {
public:
	Vec3 max;
	Vec3 min;

	AABB() {
		reset();
	}

	void reset() {
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	void extend(const Vec3 p) {
		max = Max(max, p);
		min = Min(min, p);
	}

	// Add code here
	bool rayAABB(const Ray& r, float& t) {
		Vec3 tMin = (min - r.o) * (r.invDir);
		Vec3 tMax = (max - r.o) * (r.invDir);

		Vec3 tEntry = Min(tMin, tMax);
		Vec3 tExit = Max(tMin, tMax);

		float entryVal = std::max({ tEntry.x, tEntry.y, tEntry.z });
		float exitVal = std::min({ tExit.x, tExit.y, tExit.z });

		t = std::min(entryVal, exitVal); // use this in the one that returns t

		return (entryVal <= exitVal && exitVal > 0);
	}

	// Add code here
	bool rayAABB(const Ray& r) {
		Vec3 tMin = (min - r.o) * (r.invDir);
		Vec3 tMax = (max - r.o) * (r.invDir);

		Vec3 tEntry = Min(tMin, tMax);
		Vec3 tExit = Max(tMin, tMax);

		float entryVal = std::max({ tEntry.x, tEntry.y, tEntry.z });
		float exitVal = std::min({ tExit.x, tExit.y, tExit.z });

		return (entryVal <= exitVal && exitVal > 0);
	}

	// Add code here
	float area() {
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere {
public:
	Vec3 centre;
	float radius;

	void init(Vec3& _centre, float _radius) {
		centre = _centre;
		radius = _radius;
	}

	// Add code here
	bool rayIntersect(Ray& r, float& t) {
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define C_ISECT_COST 2.0f
#define BUILD_BINS 100
#define BOUNDS_COST 1.0f

struct Bin {
	float start;
	float end;
	AABB bounds;
	int numTriangles;
};

struct BinSweep {
	AABB bounds;
	int numTriangles;
};

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	bool isLeaf = false;

	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset;
	unsigned int num;

	BVHNode() {
		r = NULL;
		l = NULL;
	}

	int getTriangleBinIdx(Triangle& triangle, std::vector<Bin>& bins, int axis) {
		float triangleCoord = triangle.centroid.coords[axis];
		for (int i = 0; i < BUILD_BINS; i++) {
			if (triangleCoord >= bins[i].start &&
				triangleCoord < bins[i].end) {
				return i;
			}
		}
		return BUILD_BINS - 1;
	}

	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, int start, int count) {
		// This part "shrinks" the BB acourding to the triangles
		for (int i = start; i < (start + count); i++) {
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}

		float lowestSplitCost = FLT_MAX;
		int splitAxis;
		Bin splitBin = {};

		// For each axis (0 = x, 1 = y, 2 = z)
		for (int axis = 0; axis < 3; axis++) {
			// Build the bins
			float minInAxis = bounds.min.coords[axis];
			float maxInAxis = bounds.max.coords[axis];

			float stepSize = (maxInAxis - minInAxis) / float(BUILD_BINS);

			std::vector<Bin> bins;
			bins.resize(BUILD_BINS);
			for (int i = 0; i < BUILD_BINS; i++) {
				float startBin;
				if (i == 0) startBin = minInAxis;
				else startBin = bins[i - 1].end;

				float endBin;
				if (i == BUILD_BINS - 1) endBin = maxInAxis;
				else endBin = startBin + stepSize;

				AABB binBoundBox;
				bins[i] = { startBin, endBin, binBoundBox, 0 };
			}

			// Assign triangles into the bins and extend bins BB accordingly
			for (int i = start; i < (start + count); i++) {
				int binIdx = getTriangleBinIdx(inputTriangles[i], bins, axis);
				bins[binIdx].numTriangles++;
				bins[binIdx].bounds.extend(inputTriangles[i].vertices[0].p);
				bins[binIdx].bounds.extend(inputTriangles[i].vertices[1].p);
				bins[binIdx].bounds.extend(inputTriangles[i].vertices[2].p);
			}

			// Left to right sweep
			int totalNumTriangles = 0;
			AABB boundBoxAcc;
			std::vector<BinSweep> leftSweep;
			leftSweep.resize(BUILD_BINS);
			for (int i = 0; i < BUILD_BINS; i++) {
				totalNumTriangles += bins[i].numTriangles;
				if (bins[i].numTriangles > 0) {
					boundBoxAcc.extend(bins[i].bounds.min);
					boundBoxAcc.extend(bins[i].bounds.max);
				}
				leftSweep[i] = { boundBoxAcc, totalNumTriangles };
			}

			// Right to left sweep
			totalNumTriangles = 0;
			boundBoxAcc.reset();
			std::vector<BinSweep> rightSweep;
			rightSweep.resize(BUILD_BINS);
			for (int i = BUILD_BINS - 1; i >= 0; i--) {
				totalNumTriangles += bins[i].numTriangles;
				if (bins[i].numTriangles > 0) {
					boundBoxAcc.extend(bins[i].bounds.min);
					boundBoxAcc.extend(bins[i].bounds.max);
				}
				rightSweep[i] = { boundBoxAcc, totalNumTriangles };
			}

			// Calculate split costs and find the lowest
			for (int i = 0; i < BUILD_BINS - 1; i++) {
				if (leftSweep[i].numTriangles == 0 || rightSweep[i + 1].numTriangles == 0) continue; // skip empty splits
				float leftCost = (leftSweep[i].bounds.area() / bounds.area()) * leftSweep[i].numTriangles * C_ISECT_COST;
				float rightCost = (rightSweep[i + 1].bounds.area() / bounds.area()) * rightSweep[i + 1].numTriangles * C_ISECT_COST;
				float splitCost = BOUNDS_COST + leftCost + rightCost;
				if (splitCost < lowestSplitCost) {
					lowestSplitCost = splitCost;
					splitAxis = axis;
					splitBin = bins[i];
				}
			}
		}

		if (lowestSplitCost == FLT_MAX) {
			isLeaf = true;
			offset = start;
			num = count;
			l = nullptr;
			r = nullptr;
			return;
		}

		// split triangles into the two spaces
		float splitPos = splitBin.end;

		auto mid = std::partition(
			inputTriangles.begin() + start,
			inputTriangles.begin() + start + count,
			[&splitAxis, &splitPos](Triangle& tri) {
				float triangleCoord = tri.centroid.coords[splitAxis];
				if (triangleCoord < splitPos) return true;
				return false;
			}
		);

		// Create left and right nodes
		int leftCount = mid - (inputTriangles.begin() + start);
		int rightCount = (inputTriangles.begin() + start + count) - mid;

		if (leftCount == 0 || rightCount == 0) {
			isLeaf = true;
			offset = start;
			num = count;
			l = nullptr;
			r = nullptr;
			return;
		}

		BVHNode* leftNode = new BVHNode();
		l = leftNode;
		if (leftCount <= MAXNODE_TRIANGLES) {
			leftNode->isLeaf = true;
			leftNode->offset = start;
			leftNode->num = leftCount;
			for (int i = start; i < (start + leftCount); i++) {
				leftNode->bounds.extend(inputTriangles[i].vertices[0].p);
				leftNode->bounds.extend(inputTriangles[i].vertices[1].p);
				leftNode->bounds.extend(inputTriangles[i].vertices[2].p);
			}
		}
		else {
			leftNode->build(inputTriangles, start, leftCount);
		}

		int rightStart = std::distance(inputTriangles.begin(), mid);

		BVHNode* rightNode = new BVHNode();
		r = rightNode;
		if (rightCount <= MAXNODE_TRIANGLES) {
			rightNode->isLeaf = true;
			rightNode->offset = rightStart;
			rightNode->num = rightCount;
			for (int i = rightStart; i < (rightStart + rightCount); i++) {
				rightNode->bounds.extend(inputTriangles[i].vertices[0].p);
				rightNode->bounds.extend(inputTriangles[i].vertices[1].p);
				rightNode->bounds.extend(inputTriangles[i].vertices[2].p);
			}
		}
		else {
			rightNode->build(inputTriangles, rightStart, rightCount);
		}
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection) {
		float boundT;
		if (!bounds.rayAABB(ray, boundT) || boundT > intersection.t) {
			return;
		}

		if (isLeaf) {
			for (int i = offset; i < (offset + num); i++) {
				float t;
				float u;
				float v;
				if (triangles[i].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = i;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}

		l->traverse(ray, triangles, intersection);
		r->traverse(ray, triangles, intersection);
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles) {
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT) {
		float boundT;
		if (!bounds.rayAABB(ray, boundT) || boundT > maxT) {
			return true;
		}

		if (isLeaf) {
			for (int i = offset; i < (offset + num); i++) {
				float t;
				float u;
				float v;
				if (triangles[i].rayIntersect(ray, t, u, v) && t > 0 && t < maxT) {
					return false;
				}
			}
			return true;
		}

		if (!l->traverseVisible(ray, triangles, maxT)) return false;
		if (!r->traverseVisible(ray, triangles, maxT)) return false;
		return true;
	}

	// bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT) {
	// 	return true;
	// }
};
