#include "envlight.h"

#include "image.h"
#include "distribution.h"
#include "transform.h"
#include "sampling.h"

int GetEnvLightSerializedSize() {
    return 1 +      // type
           16 +     // toWorld
           16 +     // toLight
           1 +      // uCDF0
           1 +      // uCDF1
           1 +      // vCDF0
           1 +      // vCDF1
           1 +      // col
           1 +      // row
           2 +      // pixelSize
           4 * 3 +  // img00~img11
           2 +      // rowWeight0 & rowWeight1
           1;       // normalization
}

std::unique_ptr<const EnvmapSampleInfo> CreateEnvmapSampleInfo(const Image3 *image) {
    int height = image->pixelHeight;
    int width = image->pixelWidth;
    size_t nEntries = (size_t)(width + 1) * (size_t)height;

    std::vector<Float> cdfCols(nEntries);
    std::vector<Float> cdfRows(height + 1);
    std::vector<Float> rowWeights(height);

    size_t colPos = 0, rowPos = 0;
    Float rowSum = Float(0.0);
    cdfRows[rowPos++] = Float(0.0);

    for (int y = 0; y < height; y++) {
        Float colSum = Float(0.0);
        cdfCols[colPos++] = Float(0.0);
        for (int x = 0; x < width; x++) {
            Vector3 value = image->At(x, y);
            colSum += Luminance(value);
            cdfCols[colPos++] = colSum;
        }

        Float normalization = inverse(colSum);
        for (int x = 1; x < width; x++) {
            cdfCols[colPos - x - 1] *= normalization;
        }
        cdfCols[colPos - 1] = Float(1.0);
        Float weight = sin((y + Float(0.5)) * c_PI / (Float)height);
        rowWeights[y] = weight;
        rowSum += colSum * weight;
        cdfRows[rowPos++] = rowSum;
    }
    Float normalization = inverse(rowSum);
    for (int y = 1; y < height; y++) {
        cdfRows[rowPos - y - 1] *= normalization;
    }
    cdfRows[rowPos - 1] = Float(1.0);

    if (rowSum == 0 || !std::isfinite(rowSum)) {
        Error("Invalid environment map");
    }
    normalization = inverse(rowSum * (c_TWOPI / width) * (c_PI / height));

    Vector2 pixelSize = Vector2(c_TWOPI / width, M_PI / height);

    return std::unique_ptr<const EnvmapSampleInfo>(
        new EnvmapSampleInfo{cdfRows, cdfCols, rowWeights, normalization, pixelSize});
}

EnvLight::EnvLight(const Float &samplingWeight,
                   const AnimatedTransform &toWorld,
                   const std::string &filename)
    : Light(samplingWeight),
      toWorld(toWorld),
      toLight(Invert(toWorld)),
      image(new Image3(filename)),
      sampleInfo(CreateEnvmapSampleInfo(image.get())) {
}

void EnvLight::Serialize(const LightPrimID &lPrimID, Float *buffer) const {
    buffer = ::Serialize((Float)LightType::EnvLight, buffer);
    buffer = ::Serialize(toWorld, buffer);
    buffer = ::Serialize(toLight, buffer);

    size_t col = lPrimID % image->pixelWidth;
    size_t row = lPrimID / image->pixelWidth;
    const Float *cdfCol = &sampleInfo->cdfCols[0] + row * (image->pixelWidth + 1);
    Float cdfCol0 = cdfCol[col];
    Float cdfCol1 = cdfCol[col + 1];
    Float cdfRow0 = sampleInfo->cdfRows[row];
    Float cdfRow1 = sampleInfo->cdfRows[row + 1];
    Vector2 pixelSize = sampleInfo->pixelSize;
    Vector3 img00 = image->RepAt(col, row);
    Vector3 img10 = image->RepAt(col + 1, row);
    Vector3 img01 = image->RepAt(col, row + 1);
    Vector3 img11 = image->RepAt(col + 1, row + 1);
    Float rowWeight0 =
        sampleInfo->rowWeights[Clamp(row, (size_t)0, (size_t)image->pixelHeight - 1)];
    Float rowWeight1 =
        sampleInfo->rowWeights[Clamp(row + 1, (size_t)0, (size_t)image->pixelHeight - 1)];

    buffer = ::Serialize(cdfCol0, buffer);
    buffer = ::Serialize(cdfCol1, buffer);
    buffer = ::Serialize(cdfRow0, buffer);
    buffer = ::Serialize(cdfRow1, buffer);
    buffer = ::Serialize((Float)col, buffer);
    buffer = ::Serialize((Float)row, buffer);
    buffer = ::Serialize(pixelSize, buffer);
    buffer = ::Serialize(img00, buffer);
    buffer = ::Serialize(img10, buffer);
    buffer = ::Serialize(img01, buffer);
    buffer = ::Serialize(img11, buffer);
    buffer = ::Serialize(rowWeight0, buffer);
    buffer = ::Serialize(rowWeight1, buffer);
    ::Serialize(sampleInfo->normalization, buffer);
}

void SampleDirection(const EnvLight *light,
                     const Vector2 rndParam,
                     const Float time,
                     LightPrimID &lPrimID,
                     Vector3 &dirToLight,
                     Vector3 &value,
                     Float &pdf) {
    Matrix4x4 transform = Interpolate(light->toWorld, time);
    auto uToIndex = [](const Float *cdf, const size_t size, Float &u) {
        const Float *entry = std::lower_bound(cdf, cdf + size + 1, u);
        size_t index = std::min(std::max((ptrdiff_t)0, entry - cdf - 1), (ptrdiff_t)size - 1);
        u = (u - (Float)cdf[index]) / (Float)(cdf[index + 1] - cdf[index]);
        return index;
    };

    const EnvmapSampleInfo *sampleInfo = light->sampleInfo.get();
    const Image3 *image = light->image.get();

    int width = image->pixelWidth;
    int height = image->pixelHeight;

    Float u0 = rndParam[0];
    Float u1 = rndParam[1];
    int row = uToIndex(&sampleInfo->cdfRows[0], height, u1);
    int col = uToIndex(&sampleInfo->cdfCols[0] + row * (width + 1), width, u0);
    lPrimID = row * width + col;

    Vector2 tent = Vector2(Tent(u0), Tent(u1));
    Vector2 pl = Vector2((Float)col, (Float)row) + tent;

    Float phi = (pl[0] + Float(0.5)) * sampleInfo->pixelSize[0];
    Float theta = (pl[1] + Float(0.5)) * sampleInfo->pixelSize[1];
    Float sinPhi = sin(phi);
    Float cosPhi = cos(phi);
    Float sinTheta = sin(theta);
    Float cosTheta = cos(theta);

    dirToLight = XformVector(transform, Vector3(sinPhi * sinTheta, cosTheta, -cosPhi * sinTheta));

    Float dx1 = tent[0], dx2 = Float(1.0) - tent[0], dy1 = tent[1], dy2 = Float(1.0) - tent[1];

    Vector3 value1 = image->At(col, row) * dx2 * dy2 + image->At(col + 1, row) * dx1 * dy2;
    Vector3 value2 = image->At(col, row + 1) * dx2 * dy1 + image->At(col + 1, row + 1) * dx1 * dy1;

    value = (value1 + value2);
    Float rowWeight0 = sampleInfo->rowWeights[Clamp(row, 0, image->pixelHeight - 1)];
    Float rowWeight1 = sampleInfo->rowWeights[Clamp(row + 1, 0, image->pixelHeight - 1)];
    pdf = (Luminance(value1) * rowWeight0 + Luminance(value2) * rowWeight1) *
          sampleInfo->normalization / fmax(fabs(sinTheta), Float(1e-7));
}

bool EnvLight::SampleDirect(const BSphere &sceneSphere,
                            const Vector3 & /*pos*/,
                            const Vector3 & /*normal*/,
                            const Vector2 rndParam,
                            const Float time,
                            LightPrimID &lPrimID,
                            Vector3 &dirToLight,
                            Float &dist,
                            Vector3 &contrib,
                            Float &cosAtLight,
                            Float &directPdf,
                            Float &emissionPdf) const {
    Vector3 value;
    SampleDirection(this, rndParam, time, lPrimID, dirToLight, value, directPdf);
    dist = std::numeric_limits<Float>::infinity();
    contrib = value * inverse(directPdf);
    cosAtLight = Float(1.0);
    Float positionPdf = c_INVPI / square(sceneSphere.radius);
    emissionPdf = directPdf * positionPdf;

    return true;
}

void EnvLight::Emission(const BSphere &sceneSphere,
                        const Vector3 &dirToLight,
                        const Vector3 & /*normalOnLight*/,
                        const Float time,
                        LightPrimID &lPrimID,
                        Vector3 &emission,
                        Float &directPdf,
                        Float &emissionPdf) const {
    Matrix4x4 transform = Interpolate(toLight, time);
    Vector3 d = XformVector(transform, dirToLight);
    Vector2 uv(atan2(d[0], -d[2]) * c_INVTWOPI * (Float)image->pixelWidth - Float(0.5),
               acos(d[1]) * c_INVPI * (Float)image->pixelHeight - Float(0.5));

    int col = int(floor(uv[0]));
    int row = int(floor(uv[1]));

    lPrimID = Modulo(row, image->pixelHeight) * image->pixelWidth + Modulo(col, image->pixelWidth);

    Float dx1 = uv[0] - col, dx2 = Float(1.0) - dx1, dy1 = uv[1] - row, dy2 = Float(1.0) - dy1;

    Vector3 value1 = image->RepAt(col, row) * dx2 * dy2 + image->RepAt(col + 1, row) * dx1 * dy2;
    Vector3 value2 =
        image->RepAt(col, row + 1) * dx2 * dy1 + image->RepAt(col + 1, row + 1) * dx1 * dy1;
    emission = value1 + value2;
    Float sinTheta = sqrt(Float(1.0) - square(d[1]));
    Float rowWeight0 = sampleInfo->rowWeights[Clamp(row, 0, image->pixelHeight - 1)];
    Float rowWeight1 = sampleInfo->rowWeights[Clamp(row + 1, 0, image->pixelHeight - 1)];
    directPdf = (Luminance(value1) * rowWeight0 + Luminance(value2) * rowWeight1) *
                sampleInfo->normalization / fmax(fabs(sinTheta), Float(1e-7));
    Float positionPdf = c_INVPI / square(sceneSphere.radius); 
    emissionPdf = directPdf * positionPdf;
}

void EnvLight::Emit(const BSphere &sceneSphere,
                    const Vector2 rndParamPos,
                    const Vector2 rndParamDir,
                    const Float time,
                    LightPrimID &lPrimID,
                    Ray &ray,
                    Vector3 &emission,
                    Float &cosAtLight,
                    Float &emissionPdf,
                    Float &directPdf) const {
    SampleDirection(this, rndParamDir, time, lPrimID, ray.dir, emission, directPdf);
    ray.dir = -ray.dir;
    Vector2 offset = SampleConcentricDisc(rndParamPos);
    Vector3 b0, b1;
    CoordinateSystem(ray.dir, b0, b1);
    Vector3 perpOffset = offset[0] * b0 + offset[1] * b1;
    ray.org = sceneSphere.center + (perpOffset - ray.dir) * sceneSphere.radius;
    cosAtLight = Float(1.0);
    Float positionPdf = c_INVPI / square(sceneSphere.radius);
    emissionPdf = directPdf * positionPdf;
}

struct ADEnvLight {
    ADAnimatedTransform toWorld;
    ADAnimatedTransform toLight;
    ADFloat cdfCol0;
    ADFloat cdfCol1;
    ADFloat cdfRow0;
    ADFloat cdfRow1;
    ADFloat col;
    ADFloat row;
    ADVector2 pixelSize;
    ADVector3 img00;
    ADVector3 img10;
    ADVector3 img01;
    ADVector3 img11;
    ADFloat rowWeight0;
    ADFloat rowWeight1;
    ADFloat normalization;
};

const ADFloat *Deserialize(const ADFloat *buffer, ADEnvLight &envLight) {
    buffer = Deserialize(buffer, envLight.toWorld);
    buffer = Deserialize(buffer, envLight.toLight);
    buffer = Deserialize(buffer, envLight.cdfCol0);
    buffer = Deserialize(buffer, envLight.cdfCol1);
    buffer = Deserialize(buffer, envLight.cdfRow0);
    buffer = Deserialize(buffer, envLight.cdfRow1);
    buffer = Deserialize(buffer, envLight.col);
    buffer = Deserialize(buffer, envLight.row);
    buffer = Deserialize(buffer, envLight.pixelSize);
    buffer = Deserialize(buffer, envLight.img00);
    buffer = Deserialize(buffer, envLight.img10);
    buffer = Deserialize(buffer, envLight.img01);
    buffer = Deserialize(buffer, envLight.img11);
    buffer = Deserialize(buffer, envLight.rowWeight0);
    buffer = Deserialize(buffer, envLight.rowWeight1);
    buffer = Deserialize(buffer, envLight.normalization);
    return buffer;
}

void SampleDirection(const ADEnvLight &envLight,
                     const ADVector2 rndParam,
                     const ADFloat time,
                     ADVector3 &dirToLight,
                     ADVector3 &value,
                     ADFloat &pdf) {
    ADMatrix4x4 transform = Interpolate(envLight.toWorld, time);

    ADFloat u0 = (rndParam[0] - envLight.cdfCol0) / (envLight.cdfCol1 - envLight.cdfCol0);
    ADFloat u1 = (rndParam[1] - envLight.cdfRow0) / (envLight.cdfRow1 - envLight.cdfRow0);
    ADVector2 tent = ADVector2(Tent(u0), Tent(u1));
    ADVector2 pl = ADVector2(envLight.col, envLight.row) + tent;

    ADFloat phi = (pl[0] + Float(0.5)) * envLight.pixelSize[0];
    ADFloat theta = (pl[1] + Float(0.5)) * envLight.pixelSize[1];
    ADFloat sinPhi = sin(phi);
    ADFloat cosPhi = cos(phi);
    ADFloat sinTheta = sin(theta);
    ADFloat cosTheta = cos(theta);

    dirToLight = XformVector(transform, ADVector3(sinPhi * sinTheta, cosTheta, -cosPhi * sinTheta));

    ADFloat dx1 = tent[0], dx2 = Float(1.0) - tent[0], dy1 = tent[1], dy2 = Float(1.0) - tent[1];

    ADVector3 value1 = envLight.img00 * dx2 * dy2 + envLight.img10 * dx1 * dy2;
    ADVector3 value2 = envLight.img01 * dx2 * dy1 + envLight.img11 * dx1 * dy1;

    value = (value1 + value2);
    pdf = (Luminance(value1) * envLight.rowWeight0 + Luminance(value2) * envLight.rowWeight1) *
          envLight.normalization / fmax(fabs(sinTheta), Float(1e-7));
}

void SampleDirectEnvLight(const ADFloat *buffer,
                          const ADBSphere &sceneSphere,
                          const ADVector3 &pos,
                          const ADVector3 &normal,
                          const ADVector2 rndParam,
                          const ADFloat time,
                          const bool isStatic,
                          ADVector3 &dirToLight,
                          ADVector3 &lightContrib,
                          ADFloat &cosAtLight,
                          ADFloat &directPdf,
                          ADFloat &emissionPdf) {
    ADEnvLight envLight;
    Deserialize(buffer, envLight);

    ADVector3 value;
    SampleDirection(envLight, rndParam, time, dirToLight, value, directPdf);
    lightContrib = value * inverse(directPdf);
    cosAtLight = Const<ADFloat>(1.0);
    ADFloat positionPdf = c_INVPI / square(sceneSphere.radius);
    emissionPdf = directPdf * positionPdf;
}

void EmissionEnvLight(const ADFloat *buffer,
                      const ADBSphere &sceneSphere,
                      const ADVector3 &dirToLight,
                      const ADVector3 &normalOnLight,
                      const ADFloat time,
                      ADVector3 &emission,
                      ADFloat &directPdf,
                      ADFloat &emissionPdf) {
    ADEnvLight envLight;
    Deserialize(buffer, envLight);

    ADMatrix4x4 transform = Interpolate(envLight.toLight, time);
    ADVector3 d = XformVector(transform, dirToLight);
    ADVector2 uv(atan2(d[0], -d[2]) / envLight.pixelSize[0] - Float(0.5),
                 acos(d[1]) / envLight.pixelSize[1] - Float(0.5));

    ADFloat dx1 = uv[0] - envLight.col, dx2 = Float(1.0) - dx1, dy1 = uv[1] - envLight.row,
            dy2 = Float(1.0) - dy1;

    ADVector3 value1 = envLight.img00 * dx2 * dy2 + envLight.img10 * dx1 * dy2;
    ADVector3 value2 = envLight.img01 * dx2 * dy1 + envLight.img11 * dx1 * dy1;
    emission = value1 + value2;
    // Ugly hack to avoid undefined derivatives
    ADFloat sinTheta = sqrt(fmax(Float(1.0) - square(d[1]), Float(1e-6)));
    directPdf =
        (Luminance(value1) * envLight.rowWeight0 + Luminance(value2) * envLight.rowWeight1) *
        envLight.normalization / fmax(fabs(sinTheta), Float(1e-7));
    ADFloat positionPdf = c_INVPI / square(sceneSphere.radius);
    emissionPdf = directPdf * positionPdf;
}

void EmitEnvLight(const ADFloat *buffer,
                  const ADBSphere &sceneSphere,
                  const ADVector2 rndParamPos,
                  const ADVector2 rndParamDir,
                  const ADFloat time,
                  const bool isStatic,
                  ADRay &ray,
                  ADVector3 &emission,
                  ADFloat &cosAtLight,
                  ADFloat &emissionPdf,
                  ADFloat &directPdf) {
    ADEnvLight envLight;
    Deserialize(buffer, envLight);

    SampleDirection(envLight, rndParamDir, time, ray.dir, emission, directPdf);
    ray.dir = -ray.dir;
    ADVector2 offset = SampleConcentricDisc(rndParamPos);
    ADVector3 b0, b1;
    CoordinateSystem(ray.dir, b0, b1);
    ADVector3 perpOffset = offset[0] * b0 + offset[1] * b1;
    ray.org = sceneSphere.center + (perpOffset - ray.dir) * sceneSphere.radius;
    cosAtLight = Const<ADFloat>(1.0);
    ADFloat positionPdf = c_INVPI / square(sceneSphere.radius);
    emissionPdf = directPdf * positionPdf;
}
