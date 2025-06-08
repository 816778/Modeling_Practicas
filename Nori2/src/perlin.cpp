#include <nori/texture.h>
#include <nori/vector.h>
#include <cmath>
#include <vector>     
#include <numeric>     // std::iota
#include <random>      // std::mt19937, std::shuffle
#include <algorithm>   // std::shuffle
#include <iostream>  

NORI_NAMESPACE_BEGIN

template <typename T>
T clamp(T val, T minVal, T maxVal) {
    return (val < minVal) ? minVal : ((val > maxVal) ? maxVal : val);
}

class PerlinTexture : public Texture {
public:
    PerlinTexture(const PropertyList &props) {
        m_scale = props.getFloat("scale", 1.0f);
        m_octaves = props.getInteger("octaves", 4);
        m_color = props.getColor("color", Color3f(1.0f));

        m_height = props.getInteger("height", 256);
        m_width = props.getInteger("width", 256);
        fScalingBias = props.getFloat("scalingBias", 2.0f);

        if (fScalingBias < 0.2f){
            fScalingBias = 0.2f;
        }
        if (m_octaves >= 9){
            m_octaves = 8; 
        }

        std::mt19937 rng(42); 
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        m_noiseSeed.resize(m_width * m_height);
        for (int i = 0; i < m_noiseSeed.size(); ++i) {
            m_noiseSeed[i] = dist(rng);
        }

    }

    Color3f eval(const Point2f &uv) const override {
        float t = PerlinNoise2D(uv);
        // std::cout << "Perlin noise value at (" << uv.x() << ", " << uv.y() << ") = " << t << std::endl;
        Color3f result = m_color * t; 
        
        return result;
    }

    std::string toString() const override {
        return tfm::format("PerlinTexture[scale=%f, octaves=%d]", m_scale, m_octaves);
    }
    

private:
    float m_scale;
    int m_octaves;
    Color3f m_color;
    std::vector<int> perm; 

    int m_height, m_width;
    std::vector<float> m_noiseSeed;
    float fScalingBias;

    float PerlinNoise2D(const Point2f &uv) const
	{
		// float x = uv.x();
        // float y = uv.y();
        float x = uv.x() * m_width * m_scale;
        float y = uv.y() * m_height * m_scale;
        		
        float fNoise = 0.0f;
        float fScaleAcc = 0.0f;
        float fScale = 1.0f;

        for (int o = 0; o < m_octaves; o++)
        {
            int nPitch = m_width >> o;
            //int nPitch = std::max(1, m_width >> o);
            int nSampleX1 = (x / nPitch) * nPitch;
            int nSampleY1 = (y / nPitch) * nPitch;

            int nSampleX2 = (nSampleX1 + nPitch) % m_width;					
            int nSampleY2 = (nSampleY1 + nPitch) % m_width;

            float fBlendX = (float)(x - nSampleX1) / (float)nPitch;
            float fBlendY = (float)(y - nSampleY1) / (float)nPitch;

            float fSampleT = (1.0f - fBlendX) * m_noiseSeed[nSampleY1 * m_width + nSampleX1] + fBlendX * m_noiseSeed[nSampleY1 * m_width + nSampleX2];
            float fSampleB = (1.0f - fBlendX) * m_noiseSeed[nSampleY2 * m_width + nSampleX1] + fBlendX * m_noiseSeed[nSampleY2 * m_width + nSampleX2];
        
            fScaleAcc += fScale;
            fNoise += (fBlendY * (fSampleB - fSampleT) + fSampleT) * fScale;
            fScale = fScale / fScalingBias;
        }

        return fNoise / fScaleAcc; 
	
	}

  
    float perlinNoiseExample(const Point3f &p) const {
         return std::sin(p.x() * 12.9898f + p.y() * 78.233f + p.z() * 37.719f);
    }

    
};

NORI_REGISTER_CLASS(PerlinTexture, "perlin")

NORI_NAMESPACE_END
