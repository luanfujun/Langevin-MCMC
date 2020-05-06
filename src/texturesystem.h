#pragma once

#include <OpenImageIO/texture.h>
namespace OpenImageIO = OIIO;
class TextureSystem {
    public:
    static void Init() {
        s_TextureSystem = OpenImageIO::TextureSystem::create();
        // s_TextureSystem->attribute("autotile", 64);
        // s_TextureSystem->attribute("automip", 1);
        // s_TextureSystem->attribute("forcefloat", 1);

        // s_TextureSystem->attribute("autotile", 64);
        s_TextureSystem->attribute("autotile", 1024);
        s_TextureSystem->attribute("max_memory_MB", 10240);
        s_TextureSystem->attribute("automip", 1);
        s_TextureSystem->attribute("forcefloat", 1);
    }
    static void Destroy() {
        OpenImageIO::TextureSystem::destroy(s_TextureSystem);
    }

    private:
    static OpenImageIO::TextureSystem *s_TextureSystem;
    template <int nChannels>
    friend class BitmapTexture;
};
