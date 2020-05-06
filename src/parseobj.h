#pragma once

#include "commondef.h"
#include "trianglemesh.h"

std::shared_ptr<TriMeshData> ParseObj(const std::string &filename,
                                      const Matrix4x4 &toWorld0,
                                      const Matrix4x4 &toWorld1,
                                      bool isMoving,
                                      bool flipNormals,
                                      bool faceNormals);
