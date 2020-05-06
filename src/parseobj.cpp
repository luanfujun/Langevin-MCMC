#include "parseobj.h"
#include "transform.h"
#include "utils.h"

#include <map>
#include <fstream>
#include <regex>
#include <string>

// trim from start
static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(),
            std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
    s.erase(
        std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),
        s.end());
    return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

static std::vector<int> splitFaceStr(const std::string &s) {
    std::regex rgx("/");
    std::sregex_token_iterator first{begin(s), end(s), rgx, -1}, last;
    std::vector<std::string> list{first, last};
    std::vector<int> result;
    for (auto &i : list) {
        if (i != "")
            result.push_back(std::stoi(i));
        else
            result.push_back(0);
    }
    while (result.size() < 3)
        result.push_back(0);

    return result;
}


// Numerical robust computation of angle between unit vectors
template <typename VectorType>
inline Float UnitAngle(const VectorType &u, const VectorType &v) {
    if (Dot(u, v) < 0)
        return (c_PI - Float(2.0)) * asin(Float(0.5) * Length(Vector3(v + u)));
    else
        return Float(2.0) * asin(Float(0.5) * Length(Vector3(v - u)));
}

inline void ComputeNormal(const std::vector<Vector3> &vertices,
                          const std::vector<TriIndex> &triangles,
                          std::vector<Vector3> &normals) {
    normals.resize(vertices.size(), Vector3::Zero());

    // Nelson Max, "Computing Vertex Vector3ds from Facet Vector3ds", 1999
    for (auto &tri : triangles) {
        Vector3 n = Vector3::Zero();
        for (int i = 0; i < 3; ++i) {
            const Vector3 &v0 = vertices[tri.index[i]];
            const Vector3 &v1 = vertices[tri.index[(i + 1) % 3]];
            const Vector3 &v2 = vertices[tri.index[(i + 2) % 3]];
            Vector3 sideA(v1 - v0), sideB(v2 - v0);
            if (i == 0) {
                n = Cross(sideA, sideB);
                Float length = Length(n);
                if (length == 0)
                    break;
                n = n / length;
            }
            Float angle = UnitAngle(Normalize(sideA), Normalize(sideB));
            normals[tri.index[i]] = normals[tri.index[i]] + n * angle;
        }
    }

    for (auto &n : normals) {
        Float length = Length(n);
        if (length != 0) {
            n = n / length;
        } else {
            /* Choose some bogus value */
            n = Vector3::Zero();
        }
    }
}

struct ObjVertex {
    ObjVertex(const std::vector<int> &id) : v(id[0] - 1), vt(id[1] - 1), vn(id[2] - 1) {
    }

    bool operator<(const ObjVertex &vertex) const {
        if (v != vertex.v)
            return v < vertex.v;
        if (vt != vertex.vt)
            return vt < vertex.vt;
        if (vn != vertex.vn)
            return vn < vertex.vn;
        return false;
    }

    int v, vt, vn;
};

size_t GetVertexId(const ObjVertex &vertex,
                   const std::vector<Vector3> &posPool,
                   const std::vector<Vector2> &stPool,
                   const std::vector<Vector3> &norPool,
                   const Matrix4x4 &toWorld0,
                   const Matrix4x4 &toWorld1,
                   bool isMoving,
                   std::vector<Vector3> &pos0,
                   std::vector<Vector3> &pos1,
                   std::vector<Vector2> &st,
                   std::vector<Vector3> &nor0,
                   std::vector<Vector3> &nor1,
                   std::map<ObjVertex, size_t> &vertexMap) {
    auto it = vertexMap.find(vertex);
    if (it != vertexMap.end()) {
        return it->second;
    }
    size_t id = pos0.size();
    pos0.push_back(XformPoint(toWorld0, posPool[vertex.v]));
    if (isMoving) {
        pos1.push_back(XformPoint(toWorld1, posPool[vertex.v]));
    } else {
        pos1.push_back(pos0.back());
    }
    if (vertex.vt != -1)
        st.push_back(stPool[vertex.vt]);
    if (vertex.vn != -1) {
        nor0.push_back(XformNormal(Matrix4x4(toWorld0.inverse()), norPool[vertex.vn]));
        if (isMoving) {
            nor1.push_back(XformNormal(Matrix4x4(toWorld1.inverse()), norPool[vertex.vn]));
        } else {
            nor1.push_back(nor0.back());
        }
    }
    vertexMap[vertex] = id;
    return id;
}

std::shared_ptr<TriMeshData> ParseObj(const std::string &filename,
                                      const Matrix4x4 &toWorld0,
                                      const Matrix4x4 &toWorld1,
                                      bool isMoving,
                                      bool flipNormals, 
                                      bool faceNormals) {
    std::vector<Vector3> posPool;
    std::vector<Vector3> norPool;
    std::vector<Vector2> stPool;
    std::map<ObjVertex, size_t> vertexMap;
    std::shared_ptr<TriMeshData> data = std::make_shared<TriMeshData>();
    data->isMoving = isMoving;

    std::ifstream ifs(filename.c_str(), std::ifstream::in);

    if (!ifs.is_open())
        throw std::runtime_error("Unable to open the obj file");
    while (ifs.good()) {
        std::string line;
        std::getline(ifs, line);
        line = trim(line);
        if (line.size() == 0 || line[0] == '#')  // comment
            continue;

        std::stringstream ss(line);
        std::string token;
        ss >> token;
        if (token == "v") {  // vertices
            Float x, y, z, w = 1.f;
            ss >> x >> y >> z >> w;
            Float invW = 1.f / w;
            posPool.push_back(Vector3(x * invW, y * invW, z * invW));
        } else if (token == "vt") {
            Float s, t, w;
            ss >> s >> t >> w;
            stPool.push_back(Vector2(s, Float(1.0) - t));
        } else if (token == "vn") {
            Float x, y, z;
            ss >> x >> y >> z;
            norPool.push_back(Normalize(Vector3(x, y, z)));
        } else if (token == "f") {
            std::string i0, i1, i2;
            ss >> i0 >> i1 >> i2;
            std::vector<int> i0f = splitFaceStr(i0);
            std::vector<int> i1f = splitFaceStr(i1);
            std::vector<int> i2f = splitFaceStr(i2);

            ObjVertex v0(i0f), v1(i1f), v2(i2f);
            size_t v0id = GetVertexId(v0,
                                      posPool,
                                      stPool,
                                      norPool,
                                      toWorld0,
                                      toWorld1,
                                      isMoving,
                                      data->position0,
                                      data->position1,
                                      data->st,
                                      data->normal0,
                                      data->normal1,
                                      vertexMap);
            size_t v1id = GetVertexId(v1,
                                      posPool,
                                      stPool,
                                      norPool,
                                      toWorld0,
                                      toWorld1,
                                      isMoving,
                                      data->position0,
                                      data->position1,
                                      data->st,
                                      data->normal0,
                                      data->normal1,
                                      vertexMap);
            size_t v2id = GetVertexId(v2,
                                      posPool,
                                      stPool,
                                      norPool,
                                      toWorld0,
                                      toWorld1,
                                      isMoving,
                                      data->position0,
                                      data->position1,
                                      data->st,
                                      data->normal0,
                                      data->normal1,
                                      vertexMap);
            data->indices.push_back(TriIndex(v0id, v1id, v2id));

            std::string i3;
            if (ss >> i3) {
                std::vector<int> i3f = splitFaceStr(i3);
                ObjVertex v3(i3f);
                size_t v3id = GetVertexId(v3,
                                          posPool,
                                          stPool,
                                          norPool,
                                          toWorld0,
                                          toWorld1,
                                          isMoving,
                                          data->position0,
                                          data->position1,
                                          data->st,
                                          data->normal0,
                                          data->normal1,
                                          vertexMap);
                data->indices.push_back(TriIndex(v0id, v2id, v3id));
            }
            std::string i4;
            if (ss >> i4) {
                Error("The object file contains n-gon (n>4) that we do not support.");
            }
        }  // Currently ignore other tokens
    }
    if (data->normal0.size() == 0 || faceNormals) {
        ComputeNormal(data->position0, data->indices, data->normal0);
        ComputeNormal(data->position1, data->indices, data->normal1);
    }

    if (flipNormals) {
        for (int i = 0; i < data->normal0.size(); i++)
            data->normal0[i] = -data->normal0[i];
        for (int i = 0; i < data->normal1.size(); i++) 
            data->normal1[i] = -data->normal1[i];
    }


    return data;
}