#include "loadserialized.h"

#include "transform.h"

#include <fstream>
#include <iostream>
#include <zlib.h>

#define MTS_FILEFORMAT_VERSION_V3 0x0003
#define MTS_FILEFORMAT_VERSION_V4 0x0004

/// Buffer size used to communicate with zlib. The larger, the better.
#define ZSTREAM_BUFSIZE 32768

enum ETriMeshFlags {
    EHasNormals = 0x0001,
    EHasTexcoords = 0x0002,
    EHasTangents = 0x0004,  // unused
    EHasColors = 0x0008,
    EFaceNormals = 0x0010,
    ESinglePrecision = 0x1000,
    EDoublePrecision = 0x2000
};

class ZStream {
    public:
    /// Create a new compression stream
    ZStream(std::fstream &fs);
    void read(void *ptr, size_t size);
    virtual ~ZStream();

    private:
    std::fstream &fs;
    size_t fsize;
    z_stream m_inflateStream;
    uint8_t m_inflateBuffer[ZSTREAM_BUFSIZE];
};

ZStream::ZStream(std::fstream &fs) : fs(fs) {
    std::streampos pos = fs.tellg();
    fs.seekg(0, fs.end);
    fsize = fs.tellg();
    fs.seekg(pos, fs.beg);

    int windowBits = 15;
    m_inflateStream.zalloc = Z_NULL;
    m_inflateStream.zfree = Z_NULL;
    m_inflateStream.opaque = Z_NULL;
    m_inflateStream.avail_in = 0;
    m_inflateStream.next_in = Z_NULL;

    int retval = inflateInit2(&m_inflateStream, windowBits);
    if (retval != Z_OK) {
        Error("Could not initialize ZLIB");
    }
}

void ZStream::read(void *ptr, size_t size) {
    uint8_t *targetPtr = (uint8_t *)ptr;
    while (size > 0) {
        if (m_inflateStream.avail_in == 0) {
            size_t remaining = fsize - fs.tellg();
            m_inflateStream.next_in = m_inflateBuffer;
            m_inflateStream.avail_in = (uInt)std::min(remaining, sizeof(m_inflateBuffer));
            if (m_inflateStream.avail_in == 0) {
                Error("Read less data than expected");
            }

            fs.read((char *)m_inflateBuffer, m_inflateStream.avail_in);
        }

        m_inflateStream.avail_out = (uInt)size;
        m_inflateStream.next_out = targetPtr;

        int retval = inflate(&m_inflateStream, Z_NO_FLUSH);
        switch (retval) {
            case Z_STREAM_ERROR: {
                Error("inflate(): stream error!");
            }
            case Z_NEED_DICT: {
                Error("inflate(): need dictionary!");
            }
            case Z_DATA_ERROR: {
                Error("inflate(): data error!");
            }
            case Z_MEM_ERROR: {
                Error("inflate(): memory error!");
            }
        };

        size_t outputSize = size - (size_t)m_inflateStream.avail_out;
        targetPtr += outputSize;
        size -= outputSize;

        if (size > 0 && retval == Z_STREAM_END) {
            Error("inflate(): attempting to read past the end of the stream!");
        }
    }
}

ZStream::~ZStream() {
    inflateEnd(&m_inflateStream);
}

template <typename VectorType>
inline Float UnitAngle(const VectorType &u, const VectorType &v) {
    if (Dot(u, v) < 0) {
        return (c_PI - Float(2.0)) * asin(Float(0.5) * Length(Vector3(v + u)));
    } else {
        return Float(2.0) * asin(Float(0.5) * Length(Vector3(v - u)));
    }
}

inline void ComputeNormal(const std::vector<Vector3> &vertices,
                          const std::vector<TriIndex> &triangles,
                          std::vector<Vector3> &normals,
                          bool flipNormals) {
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
            if (flipNormals)
                normals[tri.index[i]] = -normals[tri.index[i]];
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

void SkipToIdx(std::fstream &fs, const short version, const size_t idx) {
    // Go to the end of the file to see how many components are there
    fs.seekg(-sizeof(uint32_t), fs.end);
    uint32_t count = 0;
    fs.read((char *)&count, sizeof(uint32_t));
    size_t offset = 0;
    if (version == MTS_FILEFORMAT_VERSION_V4) {
        fs.seekg(-sizeof(uint64_t) * (count - idx) - sizeof(uint32_t), fs.end);
        fs.read((char *)&offset, sizeof(size_t));
    } else {  // V3
        fs.seekg(-sizeof(uint32_t) * (count - idx + 1), fs.end);
        uint32_t upos = 0;
        fs.read((char *)&upos, sizeof(unsigned int));
        offset = upos;
    }
    fs.seekg(offset, fs.beg);
    // Skip the header
    fs.ignore(sizeof(short) * 2);
}

template <typename Precision>
void LoadPosition(ZStream &zs,
                  const std::shared_ptr<TriMeshData> &data,
                  const Matrix4x4 &toWorld0,
                  const Matrix4x4 &toWorld1,
                  const bool isMoving) {
    for (size_t i = 0; i < data->position0.size(); i++) {
        Precision x, y, z;
        zs.read(&x, sizeof(Precision));
        zs.read(&y, sizeof(Precision));
        zs.read(&z, sizeof(Precision));
        data->position0[i] = XformPoint(toWorld0, Vector3(x, y, z));
        if (isMoving) {
            data->position1[i] = XformPoint(toWorld1, Vector3(x, y, z));
        } else {
            data->position1[i] = data->position0[i];
        }
    }
}

template <typename Precision>
void LoadNormal(ZStream &zs,
                const std::shared_ptr<TriMeshData> &data,
                const Matrix4x4 &invToWorld0,
                const Matrix4x4 &invToWorld1,
                const bool isMoving,
                const bool flipNormals) {
    for (size_t i = 0; i < data->normal0.size(); i++) {
        Precision x, y, z;
        zs.read(&x, sizeof(Precision));
        zs.read(&y, sizeof(Precision));
        zs.read(&z, sizeof(Precision));
        data->normal0[i] = XformNormal(invToWorld0, Vector3(x, y, z));
        if (isMoving) {
            data->normal1[i] = XformNormal(invToWorld1, Vector3(x, y, z));
        } else {
            data->normal1[i] = data->normal0[i];
        }
        if (flipNormals) {
            data->normal0[i] = -data->normal0[i];
            data->normal1[i] = -data->normal1[i];
        }
    }
}

template <typename Precision>
void LoadUV(ZStream &zs, const std::shared_ptr<TriMeshData> &data) {
    for (size_t i = 0; i < data->st.size(); i++) {
        Precision u, v;
        zs.read(&u, sizeof(Precision));
        zs.read(&v, sizeof(Precision));
        data->st[i] = Vector2(Float(u), Float(v));
    }
}

template <typename Precision>
void LoadColor(ZStream &zs, const std::shared_ptr<TriMeshData> &data) {
    for (size_t i = 0; i < data->colors.size(); i++) {
        double r, g, b;
        zs.read(&r, sizeof(double));
        zs.read(&g, sizeof(double));
        zs.read(&b, sizeof(double));
        data->colors[i] = Vector3(r, g, b);
    }
}

std::shared_ptr<TriMeshData> LoadSerialized(const std::string &filename,
                                            int idx,
                                            const Matrix4x4 &toWorld0,
                                            const Matrix4x4 &toWorld1,
                                            bool isMoving,
                                            bool flipNormals,
                                            bool faceNormals) {
    Matrix4x4 invToWorld0 = toWorld0.inverse();
    Matrix4x4 invToWorld1 = toWorld1.inverse();

    std::fstream fs(filename.c_str(), std::fstream::in | std::fstream::binary);
    // Format magic number, ignore it
    fs.ignore(sizeof(short));
    // Version number
    short version = 0;
    fs.read((char *)&version, sizeof(short));
    if (idx > 0) {
        SkipToIdx(fs, version, idx);
    }
    ZStream zs(fs);
    uint32_t flags;
    zs.read((char *)&flags, sizeof(uint32_t));
    std::string name;
    if (version == MTS_FILEFORMAT_VERSION_V4) {
        char c;
        while (true) {
            zs.read((char *)&c, sizeof(char));
            if (c == '\0')
                break;
            name.push_back(c);
        }
    }
    size_t vertexCount = 0;
    zs.read((char *)&vertexCount, sizeof(size_t));
    size_t triangleCount = 0;
    zs.read((char *)&triangleCount, sizeof(size_t));

    bool fileDoublePrecision = flags & EDoublePrecision;
    faceNormals = (flags & EFaceNormals) || faceNormals;

    std::shared_ptr<TriMeshData> data = std::make_shared<TriMeshData>();
    data->isMoving = isMoving;
    data->position0 = std::vector<Vector3>(vertexCount);
    data->position1 = std::vector<Vector3>(vertexCount);
    if (fileDoublePrecision) {
        LoadPosition<double>(zs, data, toWorld0, toWorld1, isMoving);
    } else {
        LoadPosition<float>(zs, data, toWorld0, toWorld1, isMoving);
    }

    if (flags & EHasNormals) {
        data->normal0 = std::vector<Vector3>(vertexCount);
        data->normal1 = std::vector<Vector3>(vertexCount);
        if (fileDoublePrecision) {
            LoadNormal<double>(zs, data, invToWorld0, invToWorld1, isMoving, flipNormals);
        } else {
            LoadNormal<float>(zs, data, invToWorld0, invToWorld1, isMoving, flipNormals);
        }
    }

    if (flags & EHasTexcoords) {
        data->st = std::vector<Vector2>(vertexCount);
        if (fileDoublePrecision) {
            LoadUV<double>(zs, data);
        } else {
            LoadUV<float>(zs, data);
        }
    }

    if (flags & EHasColors) {
        data->colors = std::vector<Vector3>(vertexCount);
        if (fileDoublePrecision) {
            LoadColor<double>(zs, data);
        } else {
            LoadColor<float>(zs, data);
        }
    }

    data->indices = std::vector<TriIndex>(triangleCount);
    zs.read(&data->indices[0], triangleCount * sizeof(TriIndex));
    if (data->normal0.size() == 0 || faceNormals) {
        ComputeNormal(data->position0, data->indices, data->normal0, flipNormals);
        ComputeNormal(data->position1, data->indices, data->normal1, flipNormals);
    }

    return data;
}
