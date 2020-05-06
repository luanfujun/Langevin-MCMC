#include "direct.h"


void DirectLighting(const Scene *scene, SampleBuffer &buffer) 
{
    if (scene->options->minDepth > 2 || scene->options->maxDepth < 1) {
        return;
    }

    std::cout << "Compute direct lighting" << std::endl;
    const Camera *camera = scene->camera.get();
    const int pixelHeight = GetPixelHeight(camera);
    const int pixelWidth = GetPixelWidth(camera);
    const int tileSize = 16;
    const int nXTiles = (pixelWidth + tileSize - 1) / tileSize;
    const int nYTiles = (pixelHeight + tileSize - 1) / tileSize;
    ProgressReporter reporter(nXTiles * nYTiles);

    Timer timer;
    Tick(timer);
    ParallelFor([&](const Vector2i tile) {
        const int seed = tile[1] * nXTiles + tile[0] + scene->options->seedOffset;
        RNG rng(seed);
        const int x0 = tile[0] * tileSize;
        const int x1 = std::min(x0 + tileSize, pixelWidth);
        const int y0 = tile[1] * tileSize;
        const int y1 = std::min(y0 + tileSize, pixelHeight);
        Path path;
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                for (int s = 0; s < scene->options->directSpp; s++) {
                    std::vector<SubpathContrib> spContribs;
                    Clear(path);
                    GeneratePath(scene,
                                 Vector2i(x, y),
                                 std::min(scene->options->minDepth, 2),
                                 std::min(scene->options->maxDepth, 2),
                                 path,
                                 spContribs,
                                 rng);
                    for (const auto &spContrib : spContribs) {
                        Vector3 contrib = spContrib.contrib;
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
}