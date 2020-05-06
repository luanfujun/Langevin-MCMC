#pragma once 

#include "scene.h"
#include "image.h"
#include "camera.h"
#include "progressreporter.h"
#include "timer.h"
#include "path.h"
#include "camera.h"


void DirectLighting(const Scene *scene, SampleBuffer &buffer);