#pragma once

#include "texture.h"
#include "texturesystem.h"
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>
#include <stdexcept>
#include "fastmath.h"


namespace OpenImageIO = OIIO;

template <int nChannels>
class BitmapTexture : public Texture<nChannels> {
    public:
    BitmapTexture(const std::string &filename,
                  const Vector2 stScaler = Vector2(Float(1.0), Float(1.0)));
    BitmapTexture(const OpenImageIO::ustring &filename,
                  const Vector2 stScaler = Vector2(Float(1.0), Float(1.0)));
    TVector<Float, nChannels> Eval(const Vector2 st) const override;
    TVector<Float, nChannels> Avg() const override {
        return avg;
    }
    std::string Name() const override {
        return filename.string();
    }
    std::shared_ptr<const Texture<1>> ToTexture1D() const override {
        if (nChannels == 1) {
            return std::dynamic_pointer_cast<const Texture<1>>(this->shared_from_this());
        } else {
            return std::make_shared<const BitmapTexture<1>>(filename, stScaler);
        }
    }
    std::shared_ptr<const Texture<3>> ToTexture3D() const override {
        if (nChannels == 1) {
            return std::make_shared<const BitmapTexture<3>>(filename, stScaler);
        } else {
            return std::dynamic_pointer_cast<const Texture<3>>(this->shared_from_this());
        }
    }

    private:
    TVector<Float, nChannels> ComputeAvg() const;
    Float GetGamma() const;

    const OpenImageIO::ustring filename;
    const Vector2 stScaler;
    const TVector<Float, nChannels> avg;
    const Float gamma;
    // weird non-constant declaration in oiio...
    mutable OpenImageIO::TextureOpt options;
};

template <int nChannels>
BitmapTexture<nChannels>::BitmapTexture(const std::string &filename, const Vector2 stScaler)
    : filename(OpenImageIO::ustring(filename)),
      stScaler(stScaler),
      avg(ComputeAvg()),
      gamma(GetGamma()) {
    options.swrap = OpenImageIO::TextureOpt::Wrap::WrapPeriodic;
    options.twrap = OpenImageIO::TextureOpt::Wrap::WrapPeriodic;
}

template <int nChannels>
BitmapTexture<nChannels>::BitmapTexture(const OpenImageIO::ustring &filename,
                                        const Vector2 stScaler)
    : filename(filename), stScaler(stScaler), avg(ComputeAvg()), gamma(GetGamma()) {
    options.swrap = OpenImageIO::TextureOpt::Wrap::WrapPeriodic;
    options.twrap = OpenImageIO::TextureOpt::Wrap::WrapPeriodic;
}

template <int nChannels>
TVector<Float, nChannels> BitmapTexture<nChannels>::Eval(const Vector2 st) const {
    float scaledS = stScaler[0] * st[0];
    float scaledT = stScaler[1] * st[1];
    std::array<float, nChannels> result;

    if (!TextureSystem::s_TextureSystem->texture(filename,
                                                 options,
                                                 scaledS,
                                                 scaledT,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 nChannels,
                                                 result.data())) {
        std::cerr << "Filename:" << filename << std::endl;
        std::cerr << "Error:" << OpenImageIO::geterror() << std::endl;
        Error("Texture lookup error");
    }
    TVector<Float, nChannels> fresult;
    for (int i = 0; i < nChannels; i++) {
        fresult[i] = fastpow(std::max(Float(result[i]), Float(0.0)), gamma);
    }
    return fresult;
}

template <int nChannels>
TVector<Float, nChannels> BitmapTexture<nChannels>::ComputeAvg() const {
    OpenImageIO::ImageBuf img(filename);
    OpenImageIO::ImageSpec spec = img.nativespec();
    Float gamma = Float(1.0);
    if (spec.format == OpenImageIO::TypeDesc::UINT8) {
        gamma = Float(2.2);
    }
    std::cout << "computeAvg(): fn = " << filename << " gamma = " << gamma << std::endl;
    
    OpenImageIO::ImageBufAlgo::pow(img, img, gamma);
    OpenImageIO::ImageBufAlgo::PixelStats stats;
    OpenImageIO::ImageBufAlgo::computePixelStats(stats, img);
    TVector<Float, nChannels> avg;
    if (img.nchannels() != nChannels) {
        if (nChannels == 1) {
            avg[0] = Float(0.0);
            for (int c = 0; c < img.nchannels(); c++) {
                avg[0] += stats.avg[c];
            }
        } else if (img.nchannels() == 1) {
            for (int c = 0; c < nChannels; c++) {
                avg[c] = stats.avg[0];
            }
        } else {
            std::cerr << "filename: " << filename << std::endl;
            Error("Texture dimension mismatch");
        }
    } else {
        for (int c = 0; c < img.nchannels(); c++) {
            avg[c] = stats.avg[c];
        }
    }
    return avg;
}

template <int nChannels>
Float BitmapTexture<nChannels>::GetGamma() const {
    OpenImageIO::ImageBuf img(filename);
    OpenImageIO::ImageSpec spec = img.nativespec();
    Float gamma = Float(1.0);
    if (spec.format == OpenImageIO::TypeDesc::UINT8) {
        gamma = Float(2.2);
    }
    return gamma;
}

typedef BitmapTexture<3> BitmapTextureRGB;
