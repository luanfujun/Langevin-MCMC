#include "parsescene.h"
#include "pugixml.hpp"
#include "animatedtransform.h"
#include "transform.h"
#include "image.h"
#include "camera.h"
#include "trianglemesh.h"
#include "parseobj.h"
#include "loadserialized.h"
#include "pointlight.h"
#include "arealight.h"
#include "envlight.h"
#include "lambertian.h"
#include "phong.h"
#include "roughdielectric.h"
#include "constanttexture.h"
#include "bitmaptexture.h"

#include <iostream>
#include <regex>
#include <map>

using BSDFMap = std::map<std::string, std::shared_ptr<const BSDF>>;
using TextureMap = std::map<std::string, std::shared_ptr<const TextureRGB>>;

Vector3 ParseVector3(const std::string &value);
Matrix4x4 ParseMatrix4x4(const std::string &value);
Matrix4x4 ParseTransform(pugi::xml_node node);
AnimatedTransform ParseAnimatedTransform(pugi::xml_node node);
std::unique_ptr<Scene> ParseScene(pugi::xml_node node);
std::shared_ptr<const Camera> ParseSensor(pugi::xml_node node, std::string &filename);
std::shared_ptr<Image3> ParseFilm(pugi::xml_node node, std::string &filename);
std::shared_ptr<const Shape> ParseShape(pugi::xml_node node,
                                        const BSDFMap &bsdfMap,
                                        const TextureMap &textureMap,
                                        std::shared_ptr<const Light> &areaLight);
std::shared_ptr<const BSDF> ParseBSDF(pugi::xml_node node,
                                      const TextureMap &textureMap,
                                      bool twoSided = false);
std::shared_ptr<const Light> ParseEmitter(pugi::xml_node node,
                                          std::shared_ptr<const EnvLight> &envLight);
std::shared_ptr<const TextureRGB> ParseTexture(pugi::xml_node node);
std::shared_ptr<const Texture1D> Parse1DMap(pugi::xml_node node, const TextureMap &textureMap);
std::shared_ptr<const TextureRGB> Parse3DMap(pugi::xml_node node, const TextureMap &textureMap);

Vector3 ParseVector3(const std::string &value) {
    std::regex rgx("(,| )+");
    std::sregex_token_iterator first{begin(value), end(value), rgx, -1}, last;
    std::vector<std::string> list{first, last};
    Vector3 v;
    if (list.size() == 1) {
        v[0] = stof(list[0]);
        v[1] = stof(list[0]);
        v[2] = stof(list[0]);
    } else if (list.size() == 3) {
        v[0] = stof(list[0]);
        v[1] = stof(list[1]);
        v[2] = stof(list[2]);
    } else {
        Error("ParseVector3 failed");
    }
    return v;
}

Matrix4x4 ParseMatrix4x4(const std::string &value) {
    std::regex rgx("(,| )+");
    std::sregex_token_iterator first{begin(value), end(value), rgx, -1}, last;
    std::vector<std::string> list{first, last};
    if (list.size() != 16) {
        Error("ParseMatrix4x4 failed");
    }

    Float m[4][4];
    int k = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            m[i][j] = std::stof(list[k++]);
        }
    }

    Matrix4x4 mat;
    mat << m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1],
        m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3];

    return mat;
}

Matrix4x4 ParseTransform(pugi::xml_node node) {
    Matrix4x4 tform = Matrix4x4::Identity();
    for (auto child : node.children()) {
        std::string name = child.name();
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        if (name == "scale") {
            if (!child.attribute("value").empty()) {
                Float s = std::stof(child.attribute("value").value());
                tform = Scale(Vector3(s, s, s)) * tform;
            } else {
                Float x = 1.0;
                Float y = 1.0;
                Float z = 1.0;
                if (!child.attribute("x").empty())
                    x = std::stof(child.attribute("x").value());
                if (!child.attribute("y").empty())
                    y = std::stof(child.attribute("y").value());
                if (!child.attribute("z").empty())
                    z = std::stof(child.attribute("z").value());
                tform = Scale(Vector3(x, y, z)) * tform;
            }
        } else if (name == "translate") {
            Float x = 0.0;
            Float y = 0.0;
            Float z = 0.0;
            if (!child.attribute("x").empty())
                x = std::stof(child.attribute("x").value());
            if (!child.attribute("y").empty())
                y = std::stof(child.attribute("y").value());
            if (!child.attribute("z").empty())
                z = std::stof(child.attribute("z").value());
            tform = Translate(Vector3(x, y, z)) * tform;
        } else if (name == "rotate") {
            Float x = 0.0;
            Float y = 0.0;
            Float z = 0.0;
            if (!child.attribute("x").empty())
                x = std::stof(child.attribute("x").value());
            if (!child.attribute("y").empty())
                y = std::stof(child.attribute("y").value());
            if (!child.attribute("z").empty())
                z = std::stof(child.attribute("z").value());
            Float angle = 0.0;
            if (!child.attribute("angle").empty())
                angle = std::stof(child.attribute("angle").value());
            tform = Rotate(angle, Vector3(x, y, z)) * tform;
        } else if (name == "lookat") {
            Vector3 pos = ParseVector3(child.attribute("origin").value());
            Vector3 target = ParseVector3(child.attribute("target").value());
            Vector3 up = ParseVector3(child.attribute("up").value());
            tform = LookAt(pos, target, up) * tform;
        } else if (name == "matrix") {
            Matrix4x4 trans = ParseMatrix4x4(std::string(child.attribute("value").value()));
            tform = trans * tform;
        }
    }
    return tform;
}

AnimatedTransform ParseAnimatedTransform(pugi::xml_node node) {
    int transformCount = 0;
    Matrix4x4 m[2];
    for (auto child : node.children()) {
        if (std::string(child.name()) == "transform") {
            m[transformCount++] = ParseTransform(child);
            if (transformCount >= 2)
                break;
        }
    }
    return MakeAnimatedTransform(m[0], m[1]);
}

std::shared_ptr<Image3> ParseFilm(pugi::xml_node node, std::string &filename) {
    int width = 512;
    int height = 512;
    for (auto child : node.children()) {
        std::string name = child.attribute("name").value();
        if (name == "width") {
            width = atoi(child.attribute("value").value());
        } else if (name == "height") {
            height = atoi(child.attribute("value").value());
        } else if (name == "filename") {
            filename = std::string(child.attribute("value").value());
        }
    }
    return std::make_shared<Image3>(width, height);
}

std::shared_ptr<const Camera> ParseSensor(pugi::xml_node node, std::string &filename) {
    AnimatedTransform toWorld = MakeAnimatedTransform(Matrix4x4::Identity(), Matrix4x4::Identity());
    Float nearClip = 1e-2;
    Float farClip = 1000.0;
    Float fov = 45.0;
    std::shared_ptr<Image3> film;

    for (auto child : node.children()) {
        std::string name = child.attribute("name").value();
        if (name == "nearClip") {
            nearClip = std::stof(child.attribute("value").value());
        } else if (name == "farClip") {
            farClip = std::stof(child.attribute("value").value());
        } else if (name == "fov") {
            fov = std::stof(child.attribute("value").value());
        } else if (name == "toWorld") {
            if (std::string(child.name()) == "transform") {
                Matrix4x4 m = ParseTransform(child);
                toWorld = MakeAnimatedTransform(m, m);
            } else if (std::string(child.name()) == "animation") {
                toWorld = ParseAnimatedTransform(child);
            }
        } else if (std::string(child.name()) == "film") {
            film = ParseFilm(child, filename);
        }
    }
    if (film.get() == nullptr) {
        film = std::make_shared<Image3>(512, 512);
    }

    // Eigen alignment issue
    return std::allocate_shared<Camera>(
        Eigen::aligned_allocator<Camera>(), toWorld, fov, film, nearClip, farClip);
}

std::shared_ptr<const Shape> ParseShape(pugi::xml_node node,
                                        const BSDFMap &bsdfMap,
                                        const TextureMap &textureMap,
                                        std::shared_ptr<const Light> &areaLight) {
    std::shared_ptr<const BSDF> bsdf;
    for (auto child : node.children()) {
        std::string name = child.name();
        if (name == "bsdf") {
            bsdf = ParseBSDF(child, textureMap);
            break;
        } else if (name == "ref") {
            pugi::xml_attribute idAttr = child.attribute("id");
            if (!idAttr.empty()) {
                auto bsdfIt = bsdfMap.find(idAttr.value());
                if (bsdfIt == bsdfMap.end()) {
                    printf("ref: %s, %s, %s\n", idAttr.name(), idAttr.value(), idAttr.as_string());
                    Error("ref not found");
                }
                bsdf = bsdfIt->second;
                break;
            } else {
                printf("ref: %s, %s, %s\n", idAttr.name(), idAttr.value(), idAttr.as_string());
                Error("ref not found");
            }
        }
    }
    std::shared_ptr<Shape> shape;
    std::string type = node.attribute("type").value();
    if (type == "serialized") {
        std::string filename;
        int shapeIndex = 0;
        Matrix4x4 toWorld[2];
        toWorld[0] = toWorld[1] = Matrix4x4::Identity();
        bool isMoving = false;
        bool flipNormals = false;
        bool faceNormals = false;

        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "filename") {
                filename = child.attribute("value").value();
            } else if (name == "shapeIndex") {
                shapeIndex = atoi(child.attribute("value").value());
            } else if (name == "flipNormals") {
                flipNormals = child.attribute("value").value();
            } else if (name == "faceNormals") {
                faceNormals = child.attribute("value").value();
            } else if (name == "toWorld") {
                if (std::string(child.name()) == "transform") {
                    toWorld[0] = toWorld[1] = ParseTransform(child);
                } else if (std::string(child.name()) == "animation") {
                    int transformCount = 0;
                    for (auto grandChild : child.children()) {
                        if (std::string(grandChild.name()) == "transform") {
                            toWorld[transformCount++] = ParseTransform(grandChild);
                            if (transformCount >= 2)
                                break;
                        }
                    }
                    if (transformCount != 2) {
                        Error("Invalid animation");
                    }
                    isMoving = true;
                }
            }
        }
        // printf("load serialized fn: %s\n", filename.c_str());
        shape = std::make_shared<TriangleMesh>(
            bsdf, LoadSerialized(filename, shapeIndex, toWorld[0], toWorld[1], isMoving, flipNormals, faceNormals));
    } else if (type == "obj") {
        std::string filename;
        Matrix4x4 toWorld[2];
        toWorld[0] = toWorld[1] = Matrix4x4::Identity();
        bool isMoving = false;
        bool flipNormals = false;
        bool faceNormals = false;
        
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "filename") {
                filename = child.attribute("value").value();
            } else if (name == "flipNormals") {
                flipNormals = child.attribute("value").value();
            } else if (name == "faceNormals") {
                faceNormals = child.attribute("value").value();
            } else if (name == "toWorld") {
                if (std::string(child.name()) == "transform") {
                    toWorld[0] = toWorld[1] = ParseTransform(child);
                } else if (std::string(child.name()) == "animation") {
                    int transformCount = 0;
                    for (auto grandChild : child.children()) {
                        if (std::string(grandChild.name()) == "transform") {
                            toWorld[transformCount++] = ParseTransform(grandChild);
                            if (transformCount >= 2)
                                break;
                        }
                    }
                    if (transformCount != 2) {
                        Error("Invalid animation");
                    }
                    isMoving = true;
                }
            }
        }
        shape = std::make_shared<TriangleMesh>(
            bsdf, ParseObj(filename, toWorld[0], toWorld[1], isMoving, flipNormals, faceNormals));
    } else {
        printf("shape type: %s not found.\n", type.c_str());
    }

    if (shape.get() == nullptr) {
        Error("Invalid shape");
    }

    for (auto child : node.children()) {
        std::string name = child.name();
        if (name == "emitter") {
            Vector3 radiance(Float(1.0), Float(1.0), Float(1.0));
            for (auto grandChild : child.children()) {
                std::string name = grandChild.attribute("name").value();
                if (name == "radiance")
                    radiance = ParseVector3(grandChild.attribute("value").value());
            }
            areaLight = std::make_shared<const AreaLight>(Float(1.0), shape.get(), radiance);
        }
    }

    return shape;
}

std::shared_ptr<const BSDF> ParseBSDF(pugi::xml_node node,
                                      const TextureMap &textureMap,
                                      bool twoSided) {
    std::string type = node.attribute("type").value();
    if (type == "diffuse") {
        std::shared_ptr<const TextureRGB> reflectance =
            std::make_shared<const ConstantTextureRGB>(Vector3(0.5, 0.5, 0.5));
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "reflectance") {
                reflectance = Parse3DMap(child, textureMap);
            }
        }
        return std::make_shared<Lambertian>(twoSided, reflectance);
    } else if (type == "phong") {
        std::shared_ptr<const TextureRGB> diffuseReflectance =
            std::make_shared<const ConstantTextureRGB>(Vector3(0.5, 0.5, 0.5));
        std::shared_ptr<const TextureRGB> specularReflectance =
            std::make_shared<const ConstantTextureRGB>(Vector3(0.2, 0.2, 0.2));
        std::shared_ptr<const Texture1D> exponent =
            std::make_shared<const ConstantTexture1D>(Vector1(30.0));
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "diffuseReflectance") {
                diffuseReflectance = Parse3DMap(child, textureMap);
            } else if (name == "specularReflectance") {
                std::string value = child.attribute("value").value();
                specularReflectance = Parse3DMap(child, textureMap);
            } else if (name == "exponent") {
                exponent = Parse1DMap(child, textureMap);
            }
        }
        return std::make_shared<Phong>(twoSided, diffuseReflectance, specularReflectance, exponent);
    } else if (type == "roughdielectric") {
        std::shared_ptr<const TextureRGB> specularReflectance =
            std::make_shared<const ConstantTextureRGB>(Vector3(1.0, 1.0, 1.0));
        std::shared_ptr<const TextureRGB> specularTransmittance =
            std::make_shared<const ConstantTextureRGB>(Vector3(1.0, 1.0, 1.0));
        Float intIOR = 1.5046;
        Float extIOR = 1.000277;
        Vector1 v(0.1);
        std::shared_ptr<const Texture1D> alpha = std::make_shared<const ConstantTexture1D>(v);
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "intIOR") {
                intIOR = std::stof(child.attribute("value").value());
            } else if (name == "extIOR") {
                extIOR = std::stof(child.attribute("value").value());
            } else if (name == "alpha") {
                alpha = Parse1DMap(child, textureMap);
            } else if (name == "specularReflectance") {
                std::string value = child.attribute("value").value();
                specularReflectance = Parse3DMap(child, textureMap);
            } else if (name == "specularTransmittance") {
                std::string value = child.attribute("value").value();
                specularTransmittance = Parse3DMap(child, textureMap);
            }
        }
        return std::make_shared<RoughDielectric>(
            twoSided, specularReflectance, specularTransmittance, intIOR, extIOR, alpha);
    } else if (type == "twosided") {
        for (auto child : node.children()) {
            if (std::string(child.name()) == "bsdf") {
                return ParseBSDF(child, textureMap, true);
            }
        }
    } else {
        printf("BSDF: %s not found.\n", type.c_str());
    }
    Error("Unknown BSDF");
    return nullptr;
}

std::shared_ptr<const TextureRGB> ParseTexture(pugi::xml_node node) {
    std::string type = node.attribute("type").value();
    if (type == "bitmap") {
        std::string filename = "";
        Float sScale = 1.0;
        Float tScale = 1.0;
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "filename") {
                filename = child.attribute("value").value();
            } else if (name == "uvscale") {
                sScale = tScale = std::stof(child.attribute("value").value());
            }
        }
        return std::make_shared<BitmapTextureRGB>(filename, Vector2(sScale, tScale));
    }
    Error("Unknown texture type");
    return nullptr;
}

template <int N>
std::shared_ptr<const Texture<N>> ParseNDMap(pugi::xml_node node, const TextureMap &textureMap) {
    std::shared_ptr<const TextureRGB> reflectance;
    std::string nodeType = node.name();
    if (nodeType == "texture") {
        reflectance = ParseTexture(node);
    } else if (nodeType == "ref") {
        pugi::xml_attribute idAttr = node.attribute("id");
        if (!idAttr.empty()) {
            auto texIt = textureMap.find(idAttr.value());
            if (texIt == textureMap.end()) {
                printf("ref: %s, %s, %s\n", idAttr.name(), idAttr.value(), idAttr.as_string());
                Error("ref not found");
            }
            reflectance = texIt->second;
        } else {
            printf("ref: %s, %s, %s\n", idAttr.name(), idAttr.value(), idAttr.as_string());
            Error("ref not found");
        }
    } else {
        std::string value = node.attribute("value").value();
        TVector<Float, N> v;
        if (N == 1) {
            v[0] = std::stof(value);
        } else if (N == 3) {
            Vector3 v3 = ParseVector3(value);
            v[0] = v3[0];
            v[1] = v3[1];
            v[2] = v3[2];
        } else {
            Error("unsupported texture dimension");
        }
        return std::make_shared<const ConstantTexture<N>>(v);
    }

    if (N == 1) {
        return std::dynamic_pointer_cast<const Texture<N>>(reflectance->ToTexture1D());
    } else if (N == 3) {
        return std::dynamic_pointer_cast<const Texture<N>>(reflectance->ToTexture3D());
    }
    Error("unsupported texture dimension");
    return nullptr;
}

std::shared_ptr<const TextureRGB> Parse3DMap(pugi::xml_node node, const TextureMap &textureMap) {
    return ParseNDMap<3>(node, textureMap);
}

std::shared_ptr<const Texture1D> Parse1DMap(pugi::xml_node node, const TextureMap &textureMap) {
    return ParseNDMap<1>(node, textureMap);
}

std::shared_ptr<const Light> ParseEmitter(pugi::xml_node node,
                                          std::shared_ptr<const EnvLight> &envLight) {
    std::string type = node.attribute("type").value();
    if (type == "point") {
        Vector3 pos(Float(0.0), Float(0.0), Float(0.0));
        Vector3 intensity(Float(1.0), Float(1.0), Float(1.0));
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "position") {
                Float x = 0.0;
                Float y = 0.0;
                Float z = 0.0;
                if (!child.attribute("x").empty())
                    x = std::stof(child.attribute("x").value());
                if (!child.attribute("y").empty())
                    y = std::stof(child.attribute("y").value());
                if (!child.attribute("z").empty())
                    z = std::stof(child.attribute("z").value());
                pos = Vector3(x, y, z);
            } else if (name == "intensity") {
                intensity = ParseVector3(child.attribute("value").value());
            }
        }
        return std::make_shared<PointLight>(Float(1.0), pos, intensity);
    } else if (type == "envmap") {
        std::string filename = "";
        AnimatedTransform toWorld =
            MakeAnimatedTransform(Matrix4x4::Identity(), Matrix4x4::Identity());
        for (auto child : node.children()) {
            std::string name = child.attribute("name").value();
            if (name == "filename") {
                filename = child.attribute("value").value();
            } else if (name == "toWorld") {
                if (std::string(child.name()) == "transform") {
                    Matrix4x4 m = ParseTransform(child);
                    toWorld = MakeAnimatedTransform(m, m);
                } else if (std::string(child.name()) == "animation") {
                    toWorld = ParseAnimatedTransform(child);
                }
            }
        }
        envLight = std::make_shared<EnvLight>(Float(1.0), toWorld, filename);
        return envLight;
    }

    Error("Unsupported emitter");
    return nullptr;
}

std::shared_ptr<DptOptions> ParseDptOptions(pugi::xml_node node) {
    std::shared_ptr<DptOptions> dptOptions = std::make_shared<DptOptions>();
    for (auto child : node.children()) {
        std::string name = child.attribute("name").value();
        if (name == "integrator") {
            dptOptions->integrator = child.attribute("value").value();
        } else if (name == "spp") {
            dptOptions->spp = std::stoi(child.attribute("value").value());
        } else if (name == "bidirectional") {
            dptOptions->bidirectional = child.attribute("value").value() == std::string("true");
        } else if (name == "numinitsamples") {
            dptOptions->numInitSamples = std::stoi(child.attribute("value").value());
        } else if (name == "largestepprob") {
            dptOptions->largeStepProbability = std::stof(child.attribute("value").value());
        } else if (name == "largestepscale") {
            dptOptions->largeStepProbScale = std::stof(child.attribute("value").value());
        } else if (name == "mindepth") {
            dptOptions->minDepth = std::stoi(child.attribute("value").value());
        } else if (name == "maxdepth") {
            dptOptions->maxDepth = std::stoi(child.attribute("value").value());
        } else if (name == "directspp") {
            dptOptions->directSpp = std::stoi(child.attribute("value").value());
        } else if (name == "perturbstddev") {
            dptOptions->perturbStdDev = std::stof(child.attribute("value").value());
        } else if (name == "roughnessthreshold") {
            dptOptions->roughnessThreshold = std::stof(child.attribute("value").value());
        } else if (name == "uniformmixprob") {
            dptOptions->uniformMixingProbability = std::stof(child.attribute("value").value());  
        } else if (name == "numchains") {
            dptOptions->numChains = std::stoi(child.attribute("value").value());
        } else if (name == "seedoffset") {
            dptOptions->seedOffset = std::stoi(child.attribute("value").value());
        } else if (name == "reportintervalspp") {
            dptOptions->reportIntervalSpp = std::stoi(child.attribute("value").value());
        } else if (name == "uselightcoordinatesampling") {
            dptOptions->useLightCoordinateSampling =
                child.attribute("value").value() == std::string("true");
        } else if (name == "largestepmultiplexed") {
            dptOptions->largeStepMultiplexed =
                child.attribute("value").value() == std::string("true");
        } else if (name == "h2mc") {
            dptOptions->h2mc = child.attribute("value").value() == std::string("true");
        } else if (name == "mala") {
            dptOptions->mala = child.attribute("value").value() == std::string("true");
        } else if (name == "mala-stepsize") {
            dptOptions->malaStepsize = std::stof(child.attribute("value").value());
        } else if (name == "mala-gn") {
            dptOptions->malaGN = std::stof(child.attribute("value").value());
        } else if (name == "samplecache") {
            dptOptions->sampleFromGlobalCache = child.attribute("value").value() == std::string("true");
        } else {
            std::cerr << "Unknown dpt option:" << name << std::endl;
        }
    }
    return dptOptions;
}

std::unique_ptr<Scene> ParseScene(pugi::xml_node node) {
    std::shared_ptr<DptOptions> options = std::make_shared<DptOptions>();
    std::shared_ptr<const Camera> camera;
    std::vector<std::shared_ptr<const Shape>> objs;
    std::vector<std::shared_ptr<const Light>> lights;
    std::shared_ptr<const EnvLight> envLight;
    std::map<std::string, std::shared_ptr<const BSDF>> bsdfMap;
    std::map<std::string, std::shared_ptr<const TextureRGB>> textureMap;
    std::string outputName = "image.exr";
    for (auto child : node.children()) {
        std::string name = child.name();
        if (name == "sensor") {
            camera = ParseSensor(child, outputName);
        } else if (name == "shape") {
            std::shared_ptr<const Light> areaLight;
            objs.push_back(ParseShape(child, bsdfMap, textureMap, areaLight));
            if (areaLight.get() != nullptr) {
                lights.push_back(areaLight);
            }
        } else if (name == "bsdf") {
            std::string id = child.attribute("id").value();
            bsdfMap[id] = ParseBSDF(child, textureMap);
        } else if (name == "emitter") {
            lights.push_back(ParseEmitter(child, envLight));
        } else if (name == "texture") {
            std::string id = child.attribute("id").value();
            textureMap[id] = ParseTexture(child);
        } else if (name == "dpt") {
            options = ParseDptOptions(child);
        }
    }
    return std::unique_ptr<Scene>(
        new Scene(options, camera, objs, lights, envLight, outputName));
}

std::unique_ptr<Scene> ParseScene(const std::string &filename) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    std::unique_ptr<Scene> scene;
    if (result) {
        scene = ParseScene(doc.child("scene"));
    } else {
        std::cerr << "Error description: " << result.description() << std::endl;
        std::cerr << "Error offset: " << result.offset << std::endl;
        Error("Parse error");
    }
    return scene;
}
