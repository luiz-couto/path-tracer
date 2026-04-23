#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <atomic>

#define MAX_DEPTH_PATH_TRACE 5
#define MIN_DEPTH_FOR_RUSSIAN_ROULETTE 3

#define TILE_SIZE 32

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas) {
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}

	void clear() {
		film->clear();
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float pmf;
		Light* sampledLight = scene->sampleLight(sampler, pmf);

		if (sampledLight->isArea())
		{
			Colour emittedColour;
			float pdf;
			Vec3 sampledPoint = sampledLight->sample(shadingData, sampler, emittedColour, pdf);

			Vec3 wi = sampledPoint - shadingData.x;
			float cosTheta = shadingData.sNormal.dot(wi);

			Vec3 normalLine = sampledLight->normal(shadingData, wi);
			float cosThetaLine = -wi.dot(normalLine);

			float denominator = (shadingData.x - sampledPoint).lengthSq();

			float gTerm = (cosTheta * cosThetaLine) / denominator;
			bool isVisible = scene->visible(shadingData.x, sampledPoint);

			float resultGTerm = gTerm * isVisible;

			Colour finalColor = shadingData.bsdf->evaluate(shadingData, wi) * emittedColour * resultGTerm;

			return finalColor / (pdf * pmf); // shall we do this or just divide by the pdf?
		}

		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool isSpecularBounce = false)
	{

		if (depth > MAX_DEPTH_PATH_TRACE) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour aux = Colour(0.0f, 0.0f, 0.0f);

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				if (isSpecularBounce || depth == 0) {
					return shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else {
					return Colour(0, 0, 0);
				}
			}

			// Vec3 sampledDirection = SamplingDistributions::uniformSampleHemisphere(sampler->next(), sampler->next());
			// Vec3 worldDirection = shadingData.frame.toWorld(sampledDirection);
			//float pdf = SamplingDistributions::uniformHemispherePDF(sampledDirection);

			Colour indirect;
			float pdf;
			Vec3 worldDirection = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);

			Colour directLight = computeDirect(shadingData, sampler);
			//Colour output = (directLight) * pathThroughput;

			float cosTheta = std::max(fabsf(worldDirection.dot(shadingData.sNormal)), 0.0f);
			Colour bsdfValue = shadingData.bsdf->evaluate(shadingData, worldDirection);

			Colour newThroughput = pathThroughput * bsdfValue * cosTheta / pdf;

			if (depth >= MIN_DEPTH_FOR_RUSSIAN_ROULETTE) {
				float q = newThroughput.Lum();
				float qClamped = std::min(q, 1.0f);
				float epsilon = sampler->next();
				if (epsilon > qClamped) {
					return (aux + directLight) * pathThroughput;
				}

				newThroughput = newThroughput / qClamped;
			}

			Ray newRay;
			newRay.init(shadingData.x + (worldDirection * EPSILON), worldDirection);
			Colour indirectLight = pathTrace(newRay, newThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());

			return (indirectLight + directLight + aux) * pathThroughput;
		}

		return pathThroughput * scene->background->evaluate(r.dir);
	}

	Colour direct(Ray& r, Sampler* sampler)
	{

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}

			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}

	Colour albedo(Ray& r) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}

	Colour viewNormals(Ray& r) {
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX) {
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void threadProcess(unsigned int tID, std::atomic<unsigned int> &tileID, unsigned int filmWidth, unsigned int filmHeight) {
		unsigned int widthToComplete = filmWidth % TILE_SIZE;
		unsigned int heightToComplete = filmHeight % TILE_SIZE;
		unsigned int fakeWidth = filmWidth + widthToComplete;
		unsigned int fakeHeight = filmHeight + heightToComplete;

		unsigned int tilesPerRow = fakeWidth / TILE_SIZE;
		unsigned int tilesPerColumn = fakeHeight / TILE_SIZE;
		unsigned int lastTileID = (tilesPerRow * tilesPerColumn) - 1;
		
		while (true) {
			unsigned int currTileID = tileID;
			tileID++;

			if (currTileID > lastTileID) return;

			unsigned int xStart = (currTileID % tilesPerRow) * TILE_SIZE;
			unsigned int yStart = (currTileID / tilesPerRow) * TILE_SIZE;

			for (unsigned int y = yStart; y < yStart + TILE_SIZE; y++) {
				if (y >= filmHeight) continue;

				for (unsigned int x = xStart; x < xStart + TILE_SIZE; x++) {
					if (x >= filmWidth) break;
		
					// float px = x + 0.5f;
					// float py = y + 0.5f;

					float px = x + samplers->next();
					float py = y + samplers->next();
					Ray ray = scene->camera.generateRay(px, py);
					// Colour col = viewNormals(ray);
					// Colour col = albedo(ray);

					Colour pathThroughput(1.0f, 1.0f, 1.0f);

					Colour col = pathTrace(ray, pathThroughput, 0, &samplers[tID]);

					//Colour col = direct(ray, &samplers[0]);
					film->splat(px, py, col);
				}
			}

			for (unsigned int y = yStart; y < yStart + TILE_SIZE; y++) {
				if (y >= filmHeight) continue;
				for (unsigned int x = xStart; x < xStart + TILE_SIZE; x++) {
					if (x >= filmWidth) break;
					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}

		}
	}

	void parallelRender() {
		film->incrementSPP();

		std::atomic<unsigned int> tileID(0);
		unsigned int filmWidth = film->width;
		unsigned int filmHeight = film->height;

		for (int i=0; i<numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::threadProcess, this, i, std::ref(tileID), filmWidth, filmHeight);
		}

		for (int i=0; i<numProcs; i++) {
			threads[i]->join();
		}
	}

	void render() {

		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				// float px = x + 0.5f;
				// float py = y + 0.5f;

				float px = x + samplers->next();
				float py = y + samplers->next();
				Ray ray = scene->camera.generateRay(px, py);
				// Colour col = viewNormals(ray);
				// Colour col = albedo(ray);

				Colour pathThroughput(1.0f, 1.0f, 1.0f);

				Colour col = pathTrace(ray, pathThroughput, 0, &samplers[0]);

				//Colour col = direct(ray, &samplers[0]);
				film->splat(px, py, col);
			}
		}

		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	int getSPP() {
		return film->SPP;
	}

	void saveHDR(std::string filename) {
		film->save(filename);
	}

	void savePNG(std::string filename) {
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};
