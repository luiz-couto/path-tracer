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

class VPL {
public:
	ShadingData shadingData;
	Colour le;

	//VPL() {}

	VPL(ShadingData _shadingData, Colour _le): shadingData(_shadingData), le(_le)  {}
};

#define INST_RAD_N 25

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;

	std::vector<VPL> vpls = {};
	unsigned int vplSize = 0;

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

	void instantRadiosityFirstPass(Sampler* sampler) {
		// First pass
		for (int i=0; i<INST_RAD_N; i++) {
			float pmf;
			Light* sampledLight = scene->sampleLightWeighted(sampler, pmf); // Use MIS later!

			float pdf;
			Vec3 p = sampledLight->samplePositionFromLight(sampler, pdf);
			ShadingData shadingData;
			Vec3 lightNormal = sampledLight->normal(shadingData, p);

			// VPL at light source
			shadingData = ShadingData(p, lightNormal);
			float powerWeighted = sampledLight->totalIntegratedPower() / (pdf * pmf);
			Colour rad = Colour(powerWeighted, powerWeighted, powerWeighted);
			vpls.push_back(VPL(shadingData, rad));
			vplSize++;

			int depth = 0;
			// Do some bounces
			while (true) {
				Ray newRay;
				newRay.init(shadingData.x + (lightNormal * EPSILON), lightNormal);
				IntersectionData intersection = scene->traverse(newRay);
				if (intersection.t >= FLT_MAX) break;

				ShadingData shadingDataBounce = scene->calculateShadingData(intersection, newRay);
				
				Colour color;
				float pdf;
				Vec3 worldDirection = shadingDataBounce.bsdf->sample(shadingDataBounce, sampler, color, pdf);
				float cosTheta = std::max(fabsf(worldDirection.dot(shadingDataBounce.sNormal)), 0.0f);
				
				rad = (rad * color * cosTheta) / pdf;
				
				// Store VPL
				if (!shadingDataBounce.bsdf->isPureSpecular()) {
					vpls.push_back(VPL(shadingDataBounce, rad));
					vplSize++;
				}

				// Russian-Roulette to decide when to stop bouncing
				if (depth >= MIN_DEPTH_FOR_RUSSIAN_ROULETTE) {
					float radClamped = std::min(rad.Lum(), 1.0f);
					float epsilon = sampler->next();
					if (epsilon > radClamped) {
						break;
					}
				}

				lightNormal = worldDirection;
				shadingData = shadingDataBounce;
				depth++;
			}
		}
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler) {

		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true) {
			return Colour(0.0f, 0.0f, 0.0f);
		}

		float pmf;
		Light* sampledLight = scene->sampleLightWeighted(sampler, pmf);

		if (sampledLight->isArea())
		{
			Colour emittedColour;
			float pdf;
			Vec3 sampledPoint = sampledLight->sample(shadingData, sampler, emittedColour, pdf);

			Vec3 wi = sampledPoint - shadingData.x;
			wi = wi.normalize();

			float cosTheta = shadingData.sNormal.dot(wi);
			if (cosTheta < 0) cosTheta = 0;

			Vec3 normalLine = sampledLight->normal(shadingData, wi);
			float cosThetaLine = -wi.dot(normalLine);
			if (cosThetaLine < 0) cosThetaLine = 0;

			float denominator = (shadingData.x - sampledPoint).lengthSq();

			float gTerm = (cosTheta * cosThetaLine) / denominator;
			bool isVisible = scene->visible(shadingData.x, sampledPoint);

			float resultGTerm = gTerm * isVisible;

			Colour finalColor = shadingData.bsdf->evaluate(shadingData, wi) * emittedColour * resultGTerm;

			return finalColor / (pdf * pmf); // shall we do this or just divide by the pdf?
		}

		// Env Light
		Colour emittedColour;
		float pdf;
		Vec3 sampledDirection = sampledLight->sample(shadingData, sampler, emittedColour, pdf);

		float cosTheta = shadingData.sNormal.dot(sampledDirection);
		if (cosTheta < 0) cosTheta = 0;

		float gTerm = cosTheta;

		float maxDist = (scene->bounds.max - scene->bounds.min).length();
		Vec3 farPoint = shadingData.x + (sampledDirection * maxDist);

		bool isVisible = scene->visible(shadingData.x, farPoint);

		float resultGTerm = gTerm * isVisible;

		Colour finalColor = shadingData.bsdf->evaluate(shadingData, sampledDirection) * emittedColour * resultGTerm;

		if (pdf <= 0) {
			std::cout << "PDF is zero or negative!" << std::endl;
			return Colour(0.0f, 0.0f, 0.0f);
		}

		return finalColor / (pdf * pmf); // shall we do this or just divide by the pdf?
		
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

			if (cosTheta < 1e-6f || pdf <= 0.0f) {
				return directLight * pathThroughput;
			}

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

			return indirectLight + (directLight * pathThroughput);
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
					Colour normalCol = viewNormals(ray);
					Colour albedoCol = albedo(ray);

					Colour pathThroughput(1.0f, 1.0f, 1.0f);
					Colour col = pathTrace(ray, pathThroughput, 0, &samplers[tID]);

					// Maybe I can remove this after MIS?
					float maxBrightness = 10.0f;
					col.r = std::min(col.r, maxBrightness);
					col.g = std::min(col.g, maxBrightness);
					col.b = std::min(col.b, maxBrightness);

					//Colour col = direct(ray, &samplers[0]);
					if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) {
						continue;
					}

					film->splat(px, py, col);

					if (!film->normalSet) {
						film->setNormal(px, py, normalCol);
						film->setAlbedo(px, py, albedoCol);
					}

					//lightTrace(&this->samplers[tID]);
				}
			}

			// for (unsigned int y = yStart; y < yStart + TILE_SIZE; y++) {
			// 	if (y >= filmHeight) continue;
			// 	for (unsigned int x = xStart; x < xStart + TILE_SIZE; x++) {
			// 		if (x >= filmWidth) break;
			// 		unsigned char r, g, b;
			// 		film->tonemap(x, y, r, g, b);
			// 		//canvas->draw(x, y, r, g, b);
			// 	}
			// }
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

		film->runDenoiserAndSetOutput();

		for (unsigned int y = 0; y < filmHeight; y++) {
			for (unsigned int x = 0; x < filmWidth; x++) {
				unsigned char r, g, b;
				film->filmicTonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}

		film->normalSet = true;
		film->albedoSet = true;
	}

	void threadProcessInstantRadiosity(unsigned int tID, std::atomic<unsigned int> &tileID, unsigned int filmWidth, unsigned int filmHeight) {
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

					float px = x + samplers->next();
					float py = y + samplers->next();
					
					Ray ray = scene->camera.generateRay(px, py);
					IntersectionData intersection = scene->traverse(ray);
					ShadingData shadingData = scene->calculateShadingData(intersection, ray);

					if (shadingData.t >= FLT_MAX) {
						Colour bgColor = scene->background->evaluate(ray.dir);
						film->splat(px, py, bgColor);
						continue;
					}

					Colour specThrouput = Colour(1.0f, 1.0f, 1.0f);
					
					if (shadingData.bsdf->isPureSpecular()) {
						int specDepth = 0;

						while(shadingData.bsdf->isPureSpecular()) {
							if (specDepth >= MAX_DEPTH_PATH_TRACE) break;

							Colour color;
							float pdf;
							Vec3 wi = shadingData.bsdf->sample(shadingData, &this->samplers[tID], color, pdf);

							float cosTheta = std::max(fabsf(wi.dot(shadingData.sNormal)), 0.0f);
							Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
							specThrouput = specThrouput * bsdf * cosTheta / pdf;

							Ray specRay;
							specRay.init(shadingData.x + (wi * EPSILON), wi);

							IntersectionData specIntersection = scene->traverse(specRay);
							if (specIntersection.t >= FLT_MAX) {
								Colour bgColor = scene->background->evaluate(specRay.dir);
								film->splat(px, py, bgColor * specThrouput);
								specThrouput = Colour(0.0f, 0.0f, 0.0f);
								break;
							}

							shadingData = scene->calculateShadingData(specIntersection, specRay);
							specDepth++;
						}
					};

					if (specThrouput.Lum() <= 0.0f || shadingData.bsdf->isPureSpecular()) continue;

					if (shadingData.bsdf->isLight()) {
						Colour emitted = shadingData.bsdf->emit(shadingData, shadingData.wo);
						film->splat(px, py, emitted * specThrouput);
						continue;
					}
					
					Colour col = computeDirect(shadingData, &this->samplers[tID]);

					for (int i=0; i<vplSize; i++) {
						const VPL& vpl = vpls[i];
						bool isVisible = scene->visible(shadingData.x, vpl.shadingData.x);

						if (!isVisible) continue;

						Vec3 wi = vpl.shadingData.x - shadingData.x;
						wi = wi.normalize();

						float cosTheta = shadingData.sNormal.dot(wi);
						if (cosTheta <= 0) continue;

						float cosThetaVPL = vpl.shadingData.sNormal.dot(-wi);
						if (cosThetaVPL < 0) cosThetaVPL = 0;
						
						float distSqr = (shadingData.x - vpl.shadingData.x).lengthSq();

						// This + 0.01f is a bias, to avoid singularity
						float gTerm = (cosTheta * cosThetaVPL) / distSqr;
						if (gTerm > 10.0f) gTerm = 10.0f;

						//distSqr = std::max(distSqr, 0.01f);
						//float gTerm = (cosTheta * cosThetaVPL) / distSqr;


						Colour brdf = shadingData.bsdf->evaluate(shadingData, wi);

						col = col + (vpl.le * brdf * gTerm) / INST_RAD_N; // NOT SURE ABOUT THIS
					}

					col = col * specThrouput;

					//Colour col = direct(ray, &samplers[0]);
					if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) {
						continue;
					}
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

	void parallelRenderInstantRadiosity() {
		film->incrementSPP();
		
		vplSize = 0;
		vpls.clear();
		vpls.reserve(INST_RAD_N * MAX_DEPTH_PATH_TRACE * 3);

		this->instantRadiosityFirstPass(&this->samplers[0]);

		std::atomic<unsigned int> tileID(0);
		unsigned int filmWidth = film->width;
		unsigned int filmHeight = film->height;

		for (int i=0; i<numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::threadProcessInstantRadiosity, this, i, std::ref(tileID), filmWidth, filmHeight);
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

	void renderLightTraceSequential() {
		film->incrementSPP();

		// for (int i=0; i<1000; i++) {
		// 	lightTrace(&this->samplers[0]);
		// }
		
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				lightTrace(&this->samplers[0]);
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

	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		float x,y;
		bool isPOnCamera = scene->camera.projectOntoCamera(p, x, y);

		if (!isPOnCamera) return;

		Vec3 cameraNormal = scene->camera.viewDirection;
		Vec3 cameraOrigin = scene->camera.origin;

		Vec3 cameraToPoint = (p - cameraOrigin).normalize();
		Vec3 pointToCamera = -cameraToPoint;
		
		float cosTheta = cameraNormal.dot(cameraToPoint);
		if (cosTheta < 0) return;

		float pointCosTheta = pointToCamera.dot(n);
		//if (pointCosTheta < 0) return;

		float gTerm = cosTheta * pointCosTheta;

		bool isVisible = scene->visible(p, cameraOrigin);
		if (!isVisible) return;

		float cosThetaSqr = cosTheta * cosTheta;

		float We = 1 / (scene->camera.Afilm * cosThetaSqr * cosThetaSqr);
		col = We * col;

		film->splat(x, y, col);
	}

	void lightTrace(Sampler *sampler) {
		float pmf;
		Light* sampledLight = scene->sampleLightWeighted(sampler, pmf);

		if (sampledLight->isArea()) {
			float pdfPosition;
			Vec3 p = sampledLight->samplePositionFromLight(sampler, pdfPosition);

			float pdfDirection;
			Vec3 wi = sampledLight->sampleDirectionFromLight(sampler, pdfDirection);

			Colour col = sampledLight->evaluate(-wi) / pdfPosition;
			
			ShadingData shadingData;
			Vec3 normal = sampledLight->normal(shadingData, -wi);
			
			connectToCamera(p, normal, col);

			Ray ray;
			ray.init(p + (normal * EPSILON), wi);

			Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
			lightTracePath(ray, pathThroughput, col, sampler, 0);
			return;
		}

		// For env light, we can just sample a direction and trace a ray in that direction, and if it hits a light source we connect it to the camera
		float pdfDirection;
		Vec3 wi = sampledLight->sampleDirectionFromLight(sampler, pdfDirection);
		Colour col = sampledLight->evaluate(-wi) / pdfDirection;

		Ray ray;
		ray.init(scene->camera.origin, wi);

		Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
		lightTracePath(ray, pathThroughput, col, sampler, 0);
	}

	void lightTracePath(Ray &r , Colour pathThroughput, Colour Le, Sampler *sampler, int depth) {
		if (depth > MAX_DEPTH_PATH_TRACE) return;
		
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				Colour col = shadingData.bsdf->emit(shadingData, shadingData.wo);
				connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput * col * Le);
				return;
			}

			Vec3 wi = scene->camera.origin - shadingData.x;
			wi = wi.normalize();

			float cosTheta = wi.dot(shadingData.sNormal);
			if (cosTheta < 0) cosTheta = 0.0f;

			bool isVisible = scene->visible(shadingData.x, scene->camera.origin);

			Colour bsdfValue = shadingData.bsdf->evaluate(shadingData, wi);
			Colour newThroughput = pathThroughput * bsdfValue * cosTheta * Le;

			if (depth >= MIN_DEPTH_FOR_RUSSIAN_ROULETTE) {
				float q = newThroughput.Lum();
				float qClamped = std::min(q, 1.0f);
				float epsilon = sampler->next();
				if (epsilon > qClamped) {
					//connectToCamera(shadingData.x, shadingData.sNormal, newThroughput);
					return;
				}

				newThroughput = newThroughput / qClamped;
			}

			if (!shadingData.bsdf->isPureSpecular() && isVisible) {
            	connectToCamera(shadingData.x, shadingData.sNormal, newThroughput);
        	}

			Colour bounceColour;
			float bouncePdf;
			Vec3 bounceDir = shadingData.bsdf->sample(shadingData, sampler, bounceColour, bouncePdf);

			Colour bounceBsdf = shadingData.bsdf->evaluate(shadingData, bounceDir);
			float cosBounce = bounceDir.dot(shadingData.sNormal);
			if (cosBounce < 0) cosBounce = 0.0f;

			Colour bounceThroughput = pathThroughput * Le * bounceBsdf * cosBounce / bouncePdf;

			Ray newRay;
			newRay.init(shadingData.x + (bounceDir * EPSILON), bounceDir);

			return lightTracePath(newRay, bounceThroughput, Colour(1.0f, 1.0f, 1.0f), sampler, depth + 1); // Or should I use Le here as well?
		}

		// Should I do this?
		Colour col = pathThroughput * scene->background->evaluate(r.dir);
		connectToCamera(r.o, Vec3(0, 1, 0), col);
		return;
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
