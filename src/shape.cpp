#include "shape.h"
#include "trianglemesh.h"
#include "utils.h"

int GetMaxShapeSerializedSize() {
    return GetTriangleMeshSerializedSize();
}

void Shape::SetAreaLight(const AreaLight *areaLight) {
    this->areaLight = areaLight;
}

const ADFloat *Intersect(const ADFloat *buffer,
                         const ADRay &ray,
                         const ADFloat time,
                         const bool isStatic,
                         ADIntersection &isect,
                         ADVector2 &st) {
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(11);
    BeginIf(Eq(type, (Float)ShapeType::TriangleMesh), ret);
    {
        ADIntersection isect;
        IntersectTriangleMesh(buffer, ray, time, isStatic, isect, st);
        SetCondOutput({isect.position[0],
                       isect.position[1],
                       isect.position[2],
                       isect.geomNormal[0],
                       isect.geomNormal[1],
                       isect.geomNormal[2],
                       isect.shadingNormal[0],
                       isect.shadingNormal[1],
                       isect.shadingNormal[2],
                       st[0],
                       st[1]});
    }
    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    isect.position[0] = ret[0];
    isect.position[1] = ret[1];
    isect.position[2] = ret[2];
    isect.geomNormal[0] = ret[3];
    isect.geomNormal[1] = ret[4];
    isect.geomNormal[2] = ret[5];
    isect.shadingNormal[0] = ret[6];
    isect.shadingNormal[1] = ret[7];
    isect.shadingNormal[2] = ret[8];
    st[0] = ret[9];
    st[1] = ret[10];
    buffer += (GetMaxShapeSerializedSize() - 1);
    return buffer;
}

void SampleShape(const ADFloat *buffer,
                 const ADVector2 rndParam,
                 const ADFloat time,
                 const bool isStatic,
                 ADVector3 &position,
                 ADVector3 &normal,
                 ADFloat &pdf) {
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(7);
    BeginIf(Eq(type, (Float)ShapeType::TriangleMesh), ret);
    {
        ADVector3 position;
        ADVector3 normal;
        ADFloat pdf;
        SampleTriangleMesh(buffer, rndParam, time, isStatic, position, normal, pdf);
        SetCondOutput(
            {position[0], position[1], position[2], normal[0], normal[1], normal[2], pdf});
    }
    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    position[0] = ret[0];
    position[1] = ret[1];
    position[2] = ret[2];
    normal[0] = ret[3];
    normal[1] = ret[4];
    normal[2] = ret[5];
    pdf = ret[6];
}

const ADFloat *SampleShapePdf(const ADFloat *buffer, ADFloat &shapePdf) {
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(1);
    BeginIf(Eq(type, (Float)ShapeType::TriangleMesh), ret);
    {
        ADFloat shapePdf = SampleTriangleMeshPdf(buffer);
        SetCondOutput({shapePdf});
    }
    BeginElse();
    { SetCondOutput({Const<ADFloat>(0.0)}); }
    EndIf();
    shapePdf = ret[0];
    buffer += (GetMaxShapeSerializedSize() - 1);
    return buffer;
}
