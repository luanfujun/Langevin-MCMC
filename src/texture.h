#pragma once

#include "commondef.h"
#include <array>
#include <memory>

template <int nChannels>
class Texture : public std::enable_shared_from_this<const Texture<nChannels>> {
    public:
    virtual TVector<Float, nChannels> Eval(const Vector2 st) const = 0;
    virtual TVector<Float, nChannels> Avg() const = 0;
    virtual std::string Name() const = 0;
    virtual std::shared_ptr<const Texture<1>> ToTexture1D() const = 0;
    virtual std::shared_ptr<const Texture<3>> ToTexture3D() const = 0;
};

using Texture1D = Texture<1>;
using Texture3D = Texture<3>;
using TextureRGB = Texture<3>;
