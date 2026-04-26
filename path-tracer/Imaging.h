#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"
#include <algorithm>
#include <OpenImageDenoise/oidn.hpp>

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const {
		// if (fabsf(x) < 0.5f && fabs(y) < 0.5f)
		// {
		// 	return 1.0f;
		// }
		// return 0;
		return 1.0f;
	}
	int size() const
	{
		return 1;
	}
};

#define GAUSSIAN_RADIUS 2.0f

class GaussianFilter : public ImageFilter {
public:
	float filter(float x, float y) const {
		float gaussianX = gaussian(x, GAUSSIAN_RADIUS);
		float gaussianY = gaussian(y, GAUSSIAN_RADIUS);
		return gaussianX * gaussianY;
	}

	float gaussian(float d, float radius) const {
		float alpha = 0.35f;
		return std::expf(-alpha * (d * d)) - std::expf(-alpha * (radius * radius));
	}

	// change this later
	int size() const {
		return int(GAUSSIAN_RADIUS + 0.5f);
	}
};

class Film
{
public:
	Colour* film;
	Colour* filmNormals;
	Colour* filmAlbedos;
	Colour* output;

	bool normalSet = false;
	bool albedoSet = false;

	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;

	void splat(const float x, const float y, const Colour& L) {
		//film[(int(y) * width) + int(x)] = L;
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}

	void setNormal(const float x, const float y, const Colour& L) {
		filmNormals[(int(y) * width) + int(x)] = L;
	}

	void setAlbedo(const float x, const float y, const Colour& L) {
		filmAlbedos[(int(y) * width) + int(x)] = L;
	}

	void runDenoiserAndSetOutput() {
		Colour* denoiserInput = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++) {
			denoiserInput[i] = film[i] / (float)SPP;
		}

		oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();
		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None) {
			std::cout << "OIDN device error: " << errorMessage << std::endl;
			delete[] denoiserInput;
			return;
		}

		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", denoiserInput, oidn::Format::Float3, width, height);
		//filter.setImage("albedo", filmAlbedos, oidn::Format::Float3, width, height);
		//filter.setImage("normal", filmNormals, oidn::Format::Float3, width, height);
		filter.setImage("output", output, oidn::Format::Float3, width, height);
		filter.set("hdr", true);

		filter.commit();
		if (device.getError(errorMessage) != oidn::Error::None) {
			std::cout << "OIDN filter commit error: " << errorMessage << std::endl;
			delete[] denoiserInput;
			return;
		}

		filter.execute();
		if (device.getError(errorMessage) != oidn::Error::None) {
			std::cout << "OIDN execute error: " << errorMessage << std::endl;
			delete[] denoiserInput;
			return;
		}

		std::cout << "OIDN OK - output[0] = " << output[0].r << " " << output[0].g << " " << output[0].b << std::endl;

		delete[] denoiserInput;

	}

	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f) {
		// Return a tonemapped pixel at coordinates x, y
		return filmicTonemap(x, y, r, g, b, exposure);
	}

	float filmicCFunc(float value) {
		float A = 0.15;
		float B = 0.50;
		float C = 0.10;
		float D = 0.20;
		float E = 0.02;
		float F = 0.30;

		float v1 = (value * (A * value + C * B) + D * E);
		float v2 = (value * (A * value + B) + D * F);
		float v3 = E / F;

		return (v1 / v2) - v3;
	}

	void filmicTonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f) {
		// Use average color per pixel (accumulated / SPP)
		Colour curr = output[(y * width) + x];
		//if (SPP > 0) curr = curr / (float)SPP;

		float expFac = 1 / 2.2;
		float W = 11.2;
		float CW = filmicCFunc(W);
		float CR = filmicCFunc(curr.r);
		float rOut = powf((CR / CW), expFac);

		float CG = filmicCFunc(curr.g);
		float gOut = powf((CG / CW), expFac);

		float CB = filmicCFunc(curr.b);
		float bOut = powf((CB / CW), expFac);

		r = std::clamp(rOut, 0.f, 1.f) * 255;
		g = std::clamp(gOut, 0.f, 1.f) * 255;
		b = std::clamp(bOut, 0.f, 1.f) * 255;

		//std::cout << "R: " << rOut << ", G: " << gOut << ", B: " << bOut << std::endl;

		//film[(y * width) + x] = Colour(rOut, gOut, bOut);
	}

	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter) {
		width = _width;
		height = _height;
		film = new Colour[width * height];
		filmNormals = new Colour[width * height];
		filmAlbedos = new Colour[width * height];
		output = new Colour[width * height];

		clear();
		filter = _filter;
	}

	void clear() {
		memset(film, 0, width * height * sizeof(Colour));
		memset(filmNormals, 0, width * height * sizeof(Colour));
		memset(filmAlbedos, 0, width * height * sizeof(Colour));
		memset(output, 0, width * height * sizeof(Colour));

		normalSet = false;
		albedoSet = false;

		SPP = 0;
	}

	void incrementSPP() {
		SPP++;
	}

	void save(std::string filename) {
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};
