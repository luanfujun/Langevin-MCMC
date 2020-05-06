#pragma once

#include "texture.h"
#include "utils.h"

template <int nChannels>
class ConstantTexture : public Texture<nChannels> {
    public:
    ConstantTexture(const TVector<Float, nChannels> &val) : val(val) {
    }
    TVector<Float, nChannels> Eval(const Vector2 /*st*/) const override {
        return val;
    }
    TVector<Float, nChannels> Avg() const override {
        return val;
    }
    std::string Name() const override {
        return "Constant";
    }
    std::shared_ptr<const Texture<1>> ToTexture1D() const override {
        if (nChannels == 1) {
            return std::dynamic_pointer_cast<const Texture<1>>(this->shared_from_this());
        } else if (nChannels == 3) {
            return std::make_shared<ConstantTexture<1>>(Vector1(::Avg(val)));
        } else {
            Error("unsupported texture dimension");
        }
    }
    std::shared_ptr<const Texture<3>> ToTexture3D() const override {
        if (nChannels == 1) {
            return std::make_shared<ConstantTexture<3>>(Vector3(val[0], val[0], val[0]));
        } else if (nChannels == 3) {
            return std::dynamic_pointer_cast<const Texture<3>>(this->shared_from_this());
        } else {
            Error("unsupported texture dimension");
        }
    }

    private:
    TVector<Float, nChannels> val;
};

using ConstantTexture1D = ConstantTexture<1>;
using ConstantTexture3D = ConstantTexture<3>;
using ConstantTextureRGB = ConstantTexture<3>;
