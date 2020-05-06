#include "phong.h"
#include "utils.h"
#include "sampling.h"
#include "fastmath.h"

int GetPhongSerializedSize() {
    return 1 +  // type
           3 +  // Kd
           3 +  // Ks
           1 +  // Exponent
           1;   // KsWeight
}

void Phong::Serialize(const Vector2 st, Float *buffer) const {
    buffer = ::Serialize((Float)BSDFType::Phong, buffer);
    buffer = ::Serialize(Kd->Eval(st), buffer);
    buffer = ::Serialize(Ks->Eval(st), buffer);
    buffer = ::Serialize(exponent->Eval(st)[0], buffer);
    ::Serialize(KsWeight, buffer);
}

void Phong::Evaluate(const Vector3 &wi,
                     const Vector3 &normal,
                     const Vector3 &wo,
                     const Vector2 st,
                     Vector3 &contrib,
                     Float &cosWo,
                     Float &pdf,
                     Float &revPdf) const {
    contrib.setZero();
    pdf = Float(0.0);
    revPdf = Float(0.0);
    Float cosWi = Dot(normal, wi);
    Vector3 normal_ = normal;
    if (twoSided && cosWi < Float(0.0)) {
        cosWi = -cosWi;
        normal_ = -normal_;
    }
    cosWo = Dot(normal_, wo);
    if (cosWi <= c_CosEpsilon || cosWo <= c_CosEpsilon) {
        return;
    }
    if (KsWeight > Float(0.0)) {
        const Float alpha = fmax(Dot(Reflect(wi, normal_), wo), Float(0.0));
        const Float expo = exponent->Eval(st)[0];
        const Float weight = std::pow(alpha, expo) * c_INVTWOPI;
        const Float expoConst1 = (expo + Float(1.0));
        const Float expoConst2 = (expo + Float(2.0));
        if (weight > Float(1e-10)) {
            contrib = Ks->Eval(st) * (expoConst2 * weight);
            pdf = KsWeight * expoConst1 * weight;
            revPdf = pdf;
        }
    }
    if (KsWeight < Float(1.0)) {
        pdf += (Float(1.0) - KsWeight) * cosWo * c_INVPI;
        revPdf += (Float(1.0) - KsWeight) * cosWi * c_INVPI;
        contrib += Kd->Eval(st) * c_INVPI;
    }
    contrib *= cosWo;
    // Just for numerical stability
    if (contrib.maxCoeff() < Float(1e-10)) {
        contrib.setZero();
    }
}

bool Phong::Sample(const Vector3 &wi,
                   const Vector3 &normal,
                   const Vector2 st,
                   const Vector2 rndParam,
                   const Float /*uDiscrete*/,
                   Vector3 &wo,
                   Vector3 &contrib,
                   Float &cosWo,
                   Float &pdf,
                   Float &revPdf) const {
    Float cosWi = Dot(wi, normal);
    if (fabs(cosWi) < c_CosEpsilon) {
        return false;
    }

    Vector3 normal_ = normal;
    if (cosWi < Float(0.0)) {
        if (twoSided) {
            cosWi = -cosWi;
            normal_ = -normal_;
        } else {
            return false;
        }
    }

    const Float expo = exponent->Eval(st)[0];
    const Vector3 R = Reflect(wi, normal_);
    Float g;
    Vector3 n;

    const Float uDiscrete = rndParam[0];
    Float rndParam0;
    if (uDiscrete > KsWeight) {
        g = Float(1.0);
        n = normal_;
        rndParam0 = (uDiscrete - KsWeight) / (Float(1.0) - KsWeight + Float(1e-10));
    } else {
        g = expo;
        n = R;
        rndParam0 = uDiscrete / (KsWeight + Float(1e-10));
    }
    const Float power = Float(1.0) / (g + Float(1.0));
    const Float cosAlpha = std::pow(rndParam[1], power);
    // assert(cosAlpha >= Float(0.0) && cosAlpha <= Float(1.0));
    const Float sinAlpha = sqrt(Float(1.0) - square(cosAlpha));
    const Float phi = c_TWOPI * rndParam0 /*rndParam[0]*/;
    const Vector3 localDir = Vector3(sinAlpha * cos(phi), sinAlpha * sin(phi), cosAlpha);
    Vector3 b0;
    Vector3 b1;
    CoordinateSystem(n, b0, b1);
    wo = localDir[0] * b0 + localDir[1] * b1 + localDir[2] * n;

    cosWo = Dot(normal_, wo);
    if (cosWo < c_CosEpsilon) {
        return false;
    }
    contrib = Vector3::Zero();
    pdf = Float(0.0);
    if (KsWeight > Float(0.0)) {
        const Float alpha = fmax(Dot(R, wo), Float(0.0));
        const Float weight = std::pow(alpha, expo) * c_INVTWOPI;
        const Float expoConst1 = (expo + Float(1.0));
        const Float expoConst2 = (expo + Float(2.0));
        if (weight > Float(1e-10)) {
            contrib = Ks->Eval(st) * (expoConst2 * weight);
            pdf = KsWeight * expoConst1 * weight;
        }
        // Phong lobe sampling is symmetric
        revPdf = pdf;
    }
    if (KsWeight < Float(1.0)) {
        contrib += Kd->Eval(st) * c_INVPI;
        pdf += (Float(1.0) - KsWeight) * cosWo * c_INVPI;
        // reverse cosine hemisphere sampling is not symmetric
        revPdf += (Float(1.0) - KsWeight) * cosWi * c_INVPI;
    }
    contrib *= cosWo;

    if (pdf < Float(1e-10)) {
        return false;
    }

    // assert(pdf > Float(0.0));
    contrib *= inverse(pdf);

    return true;
}

Float Phong::Roughness(const Vector2 /*st*/, const Float /*uDiscrete*/) const {
    return Float(1.0);
}

Float Phong::GetKsWeight() const {
    Float ksAvg = Luminance(Ks->Avg());
    Float kdAvg = Luminance(Kd->Avg());
    Float sum = ksAvg + kdAvg;
    Float ret(0.0);
    if (sum > Float(0.0)) {
        ret = ksAvg / sum;
    }
    std::cout << "GetKsWeight(): " << ret << " ksAvg: " << ksAvg << " kdAvg: " << kdAvg << std::endl;
    return ret; 
}

void EvaluatePhong(const bool adjoint,
                   const ADFloat *buffer,
                   const ADVector3 &wi,
                   const ADVector3 &normal,
                   const ADVector3 &wo,
                   const ADVector2 st,
                   ADVector3 &contrib,
                   ADFloat &cosWo,
                   ADFloat &pdf,
                   ADFloat &revPdf) {
    ADVector3 Kd;
    ADVector3 Ks;
    ADFloat exponent;
    ADFloat KsWeight;
    buffer = Deserialize(buffer, Kd);
    buffer = Deserialize(buffer, Ks);
    buffer = Deserialize(buffer, exponent);
    buffer = Deserialize(buffer, KsWeight);

    ADFloat cosWi = Dot(normal, wi);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(4);
    BeginIf(Gt(cosWi, Float(0.0)), ret);
    { SetCondOutput({normal[0], normal[1], normal[2], cosWi}); }
    BeginElse();
    { SetCondOutput({-normal[0], -normal[1], -normal[2], -cosWi}); }
    EndIf();
    ADVector3 normal_(ret[0], ret[1], ret[2]);
    cosWi = ret[3];
    cosWo = Dot(normal_, wo);
    ret = CreateCondExprVec(4);
    BeginIf(Gt(KsWeight, Float(0.0)), ret);
    {
        ADFloat alpha = Dot(Reflect(wi, normal_), wo);
        ADFloat weight = pow(alpha, exponent) * c_INVTWOPI;
        std::vector<CondExprCPtr> ret = CreateCondExprVec(4);
        BeginIf(Gt(weight, Float(1e-10)), ret);
        {
            ADFloat expoConst1 = (exponent + Float(1.0));
            ADFloat expoConst2 = (exponent + Float(2.0));
            ADVector3 specContrib = Ks * (expoConst2 * weight);
            ADFloat specPdf = KsWeight * expoConst1 * weight;
            SetCondOutput({specContrib[0], specContrib[1], specContrib[2], specPdf});
        }
        BeginElse();
        {
            SetCondOutput({Const<ADFloat>(0.0),
                           Const<ADFloat>(0.0),
                           Const<ADFloat>(0.0),
                           Const<ADFloat>(0.0)});
        }
        EndIf();
        ADVector3 specContrib = ADVector3(ret[0], ret[1], ret[2]);
        ADFloat specPdf = ret[3];
        SetCondOutput({specContrib[0], specContrib[1], specContrib[2], specPdf});
    }
    BeginElse();
    {
        SetCondOutput(
            {Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0)});
    }
    EndIf();
    contrib[0] = ret[0];
    contrib[1] = ret[1];
    contrib[2] = ret[2];
    pdf = ret[3];
    revPdf = ret[3];

    ret = CreateCondExprVec(5);
    BeginIf(Lt(KsWeight, Float(1.0)), ret);
    {
        ADVector3 diffContrib = Kd * Const<ADFloat>(c_INVPI);
        ADFloat tmp = (Float(1.0) - KsWeight) * c_INVPI;
        ADFloat diffPdf = tmp * cosWo;
        ADFloat revDiffPdf = tmp * cosWi;
        SetCondOutput({diffContrib[0], diffContrib[1], diffContrib[2], diffPdf, revDiffPdf});
    }
    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    contrib[0] += ret[0];
    contrib[1] += ret[1];
    contrib[2] += ret[2];
    pdf += ret[3];
    revPdf += ret[4];
    contrib *= cosWo;
}

void SamplePhong(const bool adjoint,
                 const ADFloat *buffer,
                 const ADVector3 &wi,
                 const ADVector3 &normal,
                 const ADVector2 st,
                 const ADVector2 rndParam,
                 const ADFloat uDiscrete,
                 const bool fixDiscrete,
                 ADVector3 &wo,
                 ADVector3 &contrib,
                 ADFloat &cosWo,
                 ADFloat &pdf,
                 ADFloat &revPdf) {
    ADVector3 Kd;
    ADVector3 Ks;
    ADFloat exponent;
    ADFloat KsWeight;
    buffer = Deserialize(buffer, Kd);
    buffer = Deserialize(buffer, Ks);
    buffer = Deserialize(buffer, exponent);
    buffer = Deserialize(buffer, KsWeight);

    ADFloat cosWi = Dot(normal, wi);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(4);
    BeginIf(Gt(cosWi, Float(0.0)), ret);
    { SetCondOutput({normal[0], normal[1], normal[2], cosWi}); }
    BeginElse();
    { SetCondOutput({-normal[0], -normal[1], -normal[2], -cosWi}); }
    EndIf();
    ADVector3 normal_(ret[0], ret[1], ret[2]);
    cosWi = ret[3];
    ADVector3 R = Reflect(wi, normal_);
    ret = CreateCondExprVec(4);
    BeginIf(Gt(uDiscrete, KsWeight), ret);
    {
        ADVector3 localDir = SampleCosHemisphere(rndParam);
        ADVector3 b0;
        ADVector3 b1;
        CoordinateSystem(normal_, b0, b1);
        ADVector3 wo = localDir[0] * b0 + localDir[1] * b1 + localDir[2] * normal_;
        ADFloat factor = (Float(1.0) - KsWeight);
        SetCondOutput({wo[0], wo[1], wo[2], factor});
    }
    BeginElse();
    {
        ADFloat power = Float(1.0) / (exponent + Float(1.0));
        ADFloat cosAlpha = pow(rndParam[1], power);
        // Ugly hack to avoid sqrt(0) which has undefined derivatives...
        ADFloat sinAlpha = sqrt(fmax(Float(1.0) - square(cosAlpha), Float(1e-6)));
        ADFloat phi = c_TWOPI * rndParam[0];
        ADVector3 localDir = ADVector3(sinAlpha * cos(phi), sinAlpha * sin(phi), cosAlpha);
        ADVector3 b0;
        ADVector3 b1;
        CoordinateSystem(R, b0, b1);
        ADVector3 wo = localDir[0] * b0 + localDir[1] * b1 + localDir[2] * R;
        ADFloat factor = KsWeight;
        SetCondOutput({wo[0], wo[1], wo[2], factor});
    }
    EndIf();
    wo = ADVector3(ret[0], ret[1], ret[2]);
    ADFloat factor = ret[3];
    cosWo = Dot(normal_, wo);

    ret = CreateCondExprVec(4);
    BeginIf(Gt(KsWeight, Float(0.0)), ret);
    {
        ADFloat alpha = Dot(R, wo);
        ADFloat weight = pow(alpha, exponent) * c_INVTWOPI;
        std::vector<CondExprCPtr> ret = CreateCondExprVec(4);
        BeginIf(Gt(weight, Float(1e-10)), ret);
        {
            ADFloat expoConst1 = (exponent + Float(1.0));
            ADFloat expoConst2 = (exponent + Float(2.0));
            ADVector3 specContrib = Ks * (expoConst2 * weight);
            ADFloat specPdf = KsWeight * expoConst1 * weight;
            SetCondOutput({specContrib[0], specContrib[1], specContrib[2], specPdf});
        }
        BeginElse();
        {
            SetCondOutput({Const<ADFloat>(0.0),
                           Const<ADFloat>(0.0),
                           Const<ADFloat>(0.0),
                           Const<ADFloat>(0.0)});
        }
        EndIf();
        ADVector3 specContrib = ADVector3(ret[0], ret[1], ret[2]);
        ADFloat specPdf = ret[3];
        SetCondOutput({specContrib[0], specContrib[1], specContrib[2], specPdf});
    }
    BeginElse();
    {
        SetCondOutput(
            {Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0)});
    }
    EndIf();
    contrib[0] = ret[0];
    contrib[1] = ret[1];
    contrib[2] = ret[2];
    pdf = ret[3];
    revPdf = ret[3];

    ret = CreateCondExprVec(5);
    BeginIf(Lt(KsWeight, Float(1.0)), ret);
    {
        ADVector3 diffContrib = Kd * Const<ADFloat>(c_INVPI);
        ADFloat tmp = (Float(1.0) - KsWeight) * c_INVPI;
        ADFloat diffPdf = tmp * cosWo;
        ADFloat revDiffPdf = tmp * cosWi;
        SetCondOutput({diffContrib[0], diffContrib[1], diffContrib[2], diffPdf, revDiffPdf});
    }
    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    contrib[0] += ret[0];
    contrib[1] += ret[1];
    contrib[2] += ret[2];
    pdf += ret[3];
    revPdf += ret[4];

    contrib *= cosWo;
    contrib *= inverse(pdf);
    if (fixDiscrete) {
        contrib *= factor;
    }
}