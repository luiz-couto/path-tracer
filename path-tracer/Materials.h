#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	
	static float getCosThetaT(float cosTheta, float n) {
		float term = 1 - ((1 - (cosTheta * cosTheta)) / (n * n));
		if (term < 0) {
			std::cout << "Total internal reflection!" << std::endl;
		}
		return std::sqrtf(term);
	}

	static float fresnelDielectric(float cosTheta, float n)
	{
		float cosThetaT = getCosThetaT(cosTheta, n);
		float fParallel = (cosTheta - (n * cosThetaT)) / (cosTheta + (n * cosThetaT));
		float fPerp = ((n * cosTheta) - cosThetaT) / ((n * cosTheta) + cosThetaT);

		float fresnel = ((fParallel * fParallel) + (fPerp * fPerp)) / 2;
		return fresnel;
	}

	static float fresnelConductorParallelSqr(float cosTheta, float n, float k) {
		float cosThetaSqr = (cosTheta * cosTheta);
		float sinThetaSqr = 1 - cosThetaSqr;
		float term1 = (((n * n) + (k * k)) * cosThetaSqr) - ((2 * n * cosTheta) + sinThetaSqr);
		float term2 = (((n * n) + (k * k)) * cosThetaSqr) + ((2 * n * cosTheta) + sinThetaSqr);
		return term1 / term2;
	}

	static float fresnelConductorPerpSqr(float cosTheta, float n, float k) {
		float cosThetaSqr = (cosTheta * cosTheta);
		float term1 = ((n * n) + (k * k)) - (2 * n * cosTheta) + cosThetaSqr;
		float term2 = ((n * n) + (k * k)) + (2 * n * cosTheta) + cosThetaSqr;
		return term1 / term2;
	}

	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k) {
		float r = (fresnelConductorParallelSqr(cosTheta, ior.r, k.r) + fresnelConductorPerpSqr(cosTheta, ior.r, k.r)) / 2;
		float g = (fresnelConductorParallelSqr(cosTheta, ior.g, k.g) + fresnelConductorPerpSqr(cosTheta, ior.g, k.g)) / 2;
		float b = (fresnelConductorParallelSqr(cosTheta, ior.b, k.b) + fresnelConductorPerpSqr(cosTheta, ior.b, k.b)) / 2;
		return Colour(r, g, b);
	}

	static float lambdaGGX(Vec3 wi, float alpha) {
		float cosTheta = wi.z;
		float cosThetaSqr = cosTheta * cosTheta;
		float tanThetaSqr = (1 - cosThetaSqr) / cosThetaSqr;

		float term = 1 + (alpha * alpha * tanThetaSqr);
		float lambda = (std::sqrtf(term) - 1) / 2;

		return lambda;
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha) {
		float go = 1 / (1 + lambdaGGX(wo, alpha));
		float gi = 1 / (1 + lambdaGGX(wi, alpha));

		return go * gi;
	}

	static float Dggx(Vec3 h, float alpha) {
		float cosThetaM = h.z;
		float alphaSqr = alpha * alpha;
		float term = (cosThetaM * cosThetaM) * (alphaSqr - 1) + 1;

		float D = alphaSqr / (M_PI * (term * term));
		return D;
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = PDF(shadingData, wi);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		//Vec3 localWI = shadingData.frame.toLocal(wi);
		float cosTheta = SamplingDistributions::cosineHemispherePDF(wi);
		return cosTheta;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo) {
		albedo = _albedo;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Replace this with Mirror sampling code
		// Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		// pdf = wi.z / M_PI;
		// reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		// wi = shadingData.frame.toWorld(wi);
		// return wi;

		pdf = 1;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wi = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		//reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / fabsf(wi.z);
		wi = shadingData.frame.toWorld(wi);
		return wi;

	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 localWi = shadingData.frame.toLocal(wi);
		float cosTheta = fabsf(localWi.z);

		Colour fresnel = ShadingHelper::fresnelConductor(cosTheta, Colour(1.4f, 1.4f, 1.4f), Colour(7.6f, 6.3f, 5.4f));
		return fresnel * albedo->sample(shadingData.tu, shadingData.tv) / cosTheta;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		return 0;
	}

	bool isPureSpecular() {
		return true;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness) {
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		float s1 = sampler->next();
		float s2 = sampler->next();

		float thetaM = (1 - s1) / ((s1 * ((alpha * alpha) - 1)) + 1);
		thetaM = acosf(sqrtf(thetaM));
		float phiM = 2 * M_PI * s2;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);


		Vec3 wm = Vec3(sinf(thetaM) * cosf(phiM), sinf(thetaM) * sinf(phiM), cosf(thetaM));
		Vec3 wi = -woLocal +  (wm * (2 * wm.dot(woLocal)));
		pdf = (ShadingHelper::Dggx(wm, alpha) * cosf(thetaM)) / (4 * wm.dot(woLocal));
		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 localWi = shadingData.frame.toLocal(wi);
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		Vec3 h = (localWi + localWo).normalize();
		float gWoWi = ShadingHelper::Gggx(localWi, localWo, alpha);
		float dWm = ShadingHelper::Dggx(h, alpha);
		Colour fresnel = ShadingHelper::fresnelConductor(localWo.z, eta, k);

		Colour term1 = fresnel * gWoWi * dWm;
		float term2 = 4 * localWo.z * localWi.z;
		Colour brdf = term1 / term2;

		return brdf * albedo->sample(shadingData.tu, shadingData.tv);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 localWi = shadingData.frame.toLocal(wi);
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		Vec3 h = (localWi + localWo).normalize();
		float dWm = ShadingHelper::Dggx(h, alpha);
		float cosThetaM = h.z;

		float pdf = (dWm * cosThetaM) / (4 * localWo.dot(h));
		return pdf;
	}

	bool isPureSpecular() {
		return false;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR) {
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		pdf = 1;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = woLocal.z;

		float n;
		bool isEntering = cosTheta > 0;
		float sign = 1;
		if (isEntering) {
			n = extIOR / intIOR;
		} else {
			sign = -1;
			n = intIOR / extIOR;
		}

		float term = 1 - ((1 - (cosTheta * cosTheta)) / (n * n));
		if (term < 0.0f) {
			Vec3 wi = Vec3(-woLocal.x, -woLocal.y, cosTheta);
			return shadingData.frame.toWorld(wi);
		}

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(cosTheta), n);
		if (sampler->next() < fresnel) {
			// reflect, act as mirror
			Vec3 wi = Vec3(-woLocal.x, -woLocal.y, cosTheta);
			return shadingData.frame.toWorld(wi);
		}

		// refract
		Vec3 wi = Vec3(-woLocal.x, -woLocal.y, cosTheta);

		float cosThetaT = ShadingHelper::getCosThetaT(cosTheta, n);
		Vec3 wt = Vec3(-n*woLocal.x, -n*woLocal.y, -cosThetaT * sign);
		return shadingData.frame.toWorld(wt);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		float cosTheta = woLocal.z;
		
		float n;
		bool isEntering = cosTheta > 0;
		if (isEntering) {
			n = extIOR / intIOR;
		} else {
			n = intIOR / extIOR;
		}

		float term = 1 - ((1 - (cosTheta * cosTheta)) / (n * n));
		if (term < 0.0f) {
			return albedo->sample(shadingData.tu, shadingData.tv) / fabsf(cosTheta);
		}

		Vec3 wr = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		float cosThetaT = ShadingHelper::getCosThetaT(cosTheta, n);

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(cosTheta), n);
		Colour fresnel1 = Colour(fresnel, fresnel, fresnel);
		Colour fresnel2 = Colour(1 - fresnel, 1 - fresnel, 1 - fresnel);

		Colour sample = albedo->sample(shadingData.tu, shadingData.tv);

		Colour term1 = (fresnel1 * sample) / cosTheta;
		float auxN = n * n;
		Colour nColor = Colour(auxN, auxN, auxN);
		Colour term2 = nColor * fresnel2 * (sample / cosThetaT);

		return term1 + term2;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {		
		return 0;
	}

	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness) {
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}

	float alphaToPhongExponent() {
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Replace this with Plastic sampling code
		// Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		// pdf = wi.z / M_PI;
		// reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		// wi = shadingData.frame.toWorld(wi);
		// return wi;

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = woLocal.z;

		float n;
		bool isEntering = cosTheta > 0;
		float sign = 1;
		if (isEntering) {
			n = extIOR / intIOR;
		} else {
			sign = -1;
			n = intIOR / extIOR;
		}

		if (alpha < 0.001f) {  // epsilon threshold
			// Treat as mirror
			pdf = 1.0f;
			Vec3 wi = Vec3(-woLocal.x, -woLocal.y, woLocal.z * sign);
			return shadingData.frame.toWorld(wi);
		}

		float s1 = sampler->next();
		float s2 = sampler->next();

		//std::cout << "Sampler values: " << s1 << ", " << s2 << std::endl;

		float thetaM = (1 - s1) / ((s1 * ((alpha * alpha) - 1)) + 1);
		//std::cout << "ThetaM before acos: " << thetaM << std::endl;
		thetaM = acosf(sqrtf(thetaM));
		float phiM = 2 * M_PI * s2;


		Vec3 wm = Vec3(sinf(thetaM) * cosf(phiM), sinf(thetaM) * sinf(phiM), cosf(thetaM));
		Vec3 wi = -woLocal +  (wm * (2 * wm.dot(woLocal)));
		pdf = PDF(shadingData, shadingData.frame.toWorld(wi));
		return shadingData.frame.toWorld(wi);
	}
	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Plastic evaluation code
		// return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;

		Vec3 localWi = shadingData.frame.toLocal(wi);
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = localWo.z;

		if (cosTheta < 0) {
			return albedo->sample(shadingData.tu, shadingData.tv) / fabsf(cosTheta);
		}

		Vec3 h = (localWi + localWo).normalize();
		float gWoWi = ShadingHelper::Gggx(localWi, localWo, alpha);
		float dWm = ShadingHelper::Dggx(h, alpha);

		float n;
		bool isEntering = cosTheta > 0;
		float sign = 1;
		if (isEntering) {
			n = extIOR / intIOR;
		} else {
			sign = -1;
			n = intIOR / extIOR;
		}

    	if (alpha < 0.001f) {
        	Colour fresnel = ShadingHelper::fresnelConductor(cosTheta, Colour(n, n, n), Colour(10.0, 10.0, 10.0));
        	return fresnel * albedo->sample(shadingData.tu, shadingData.tv) / fabsf(localWo.z);
    	}

		float term = 1 - ((1 - (cosTheta * cosTheta)) / (n * n));
		if (term < 0.0f) {
			return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		}

		float fresnel = ShadingHelper::fresnelDielectric(fabsf(localWo.z), n);

		float term1 = fresnel * gWoWi * dWm;
		float term2 = 4 * localWo.z * localWi.z;
		float brdf = term1 / term2;
		Colour brdfColour = Colour(brdf, brdf, brdf);

		return brdfColour + ((1 - fresnel) * albedo->sample(shadingData.tu, shadingData.tv) / M_PI) ;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Plastic PDF
		//Vec3 wiLocal = shadingData.frame.toLocal(wi);
		//return SamplingDistributions::cosineHemispherePDF(wiLocal);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		Vec3 localWo = shadingData.frame.toLocal(shadingData.wo);

		
		Vec3 h = (localWi + localWo).normalize();
		float dWm = ShadingHelper::Dggx(h, alpha);
		float cosThetaM = h.z;
		
		//std::cout << "Local Wo dot h: " << localWo.dot(h) << std::endl;
		float pdf = (dWm * cosThetaM) / (4 * localWo.dot(h));
		return pdf;
	}

	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};
