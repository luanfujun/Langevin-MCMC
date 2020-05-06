#pragma once

#include "commondef.h"
#include "trianglemesh.h"
#include <string>

std::shared_ptr<TriMeshData> LoadSerialized(const std::string &filename,
                                            int shapeIndex,
                                            const Matrix4x4 &toWorld0,
                                            const Matrix4x4 &toWorld1,
                                            bool isMoving,
                                            bool flipNormals,
                                            bool faceNormals);
