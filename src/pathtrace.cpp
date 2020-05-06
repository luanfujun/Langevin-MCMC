#include "pathtrace.h"
#include "camera.h"
#include "image.h"
#include "path.h"
#include "progressreporter.h"
#include "parallel.h"
#include "timer.h"
#include "bsdf.h"

#include <algorithm>
#include <vector>
#include <unordered_map>

void PathTrace(const Scene *scene, const std::shared_ptr<const PathFuncLib> pathFuncLib) {
    SerializedSubpath ssubPath;

    int maxDervDepth = pathFuncLib->maxDepth;
    std::vector<Float> sceneParams(GetSceneSerializedSize());
    Serialize(scene, &sceneParams[0]);
    ssubPath.primary.resize(GetPrimaryParamSize(maxDervDepth, maxDervDepth));
    ssubPath.vertParams.resize(GetVertParamSize(maxDervDepth, maxDervDepth));

    const int spp = scene->options->spp;
    std::shared_ptr<const Camera> camera = scene->camera;
    std::shared_ptr<Image3> film = camera->film;
    film->Clear();
    const int pixelHeight = GetPixelHeight(camera.get());
    const int pixelWidth = GetPixelWidth(camera.get());
    const int tileSize = 16;
    const int nXTiles = (pixelWidth + tileSize - 1) / tileSize;
    const int nYTiles = (pixelHeight + tileSize - 1) / tileSize;
    ProgressReporter reporter(nXTiles * nYTiles);
    SampleBuffer buffer(pixelWidth, pixelHeight);
    auto pathFunc = scene->options->bidirectional ? GeneratePathBidir : GeneratePath;
    Timer timer;
    Tick(timer);

    ParallelFor([&](const Vector2i tile) {
        const int seed = tile[1] * nXTiles + tile[0];
        RNG rng(seed);
        const int x0 = tile[0] * tileSize;
        const int x1 = std::min(x0 + tileSize, pixelWidth);
        const int y0 = tile[1] * tileSize;
        const int y1 = std::min(y0 + tileSize, pixelHeight);
        Path path;
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                for (int s = 0; s < spp; s++) {
                    Clear(path);
                    std::vector<SubpathContrib> spContribs;
                    
                    pathFunc(scene,
                             Vector2i(x, y),
                             scene->options->minDepth,
                             scene->options->maxDepth,
                             path,
                             spContribs,
                             rng);

                    for (const auto &spContrib : spContribs) {
                        if (Luminance(spContrib.contrib) <= Float(1e-10)) {
                            continue;
                        }
                        Vector3 contrib = spContrib.contrib / Float(spp);
                        Splat(buffer, spContrib.screenPos, contrib);
                    }
                }
            }
        }
        reporter.Update(1);
    }, Vector2i(nXTiles, nYTiles));
    TerminateWorkerThreads();
    reporter.Done();
    Float elapsed = Tick(timer);
    std::cout << "Elapsed time:" << elapsed << std::endl;

    BufferToFilm(buffer, film.get());
}
