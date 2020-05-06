#pragma once 

#include "mlt.h"
#include "mala.h"
#include "fastmath.h"
#include <nanoflann.hpp>

#define PSS_MIN_LENGTH 2
#define PSS_MAX_LENGTH 12
#define PSS_MAX_SIZE   3000 // 10000
#define PSS_QUERY_DIST Float(0.01)
#define PSS_REUSE_DIST Float(0.10)
#define CACHE_SIG  Float(0.15)   
#define CACHE_PROB Float(0.50)

using namespace nanoflann;

template <int dim>
struct point_cloud_t {
    std::vector<std::vector<Float>> data_pss, data_v1, data_v2;
    std::vector<Path> data_path; 
    std::vector<Float> data_pathWeight;
    std::vector<SubpathContrib> data_spContrib;
    inline size_t kdtree_get_point_count() const {
        return data_pss.size();
    }
    inline Float kdtree_get_pt(const size_t idx, const size_t offset) const {
        return data_pss[idx][offset];
    }
    template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }
};

template <int dim>
struct global_cache_t {
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<Float, point_cloud_t<dim>>, 
        point_cloud_t<dim>, 
        dim
    > KDTree;
    
    int64_t data_idx;
    bool is_ready;
    std::mutex mutex;
    double score_sum;
    KDTree *data_tree;
    point_cloud_t<dim> data_pts;
    std::shared_ptr<PiecewiseConstant1D> data_distrib;
    Float inv_sigma_sq, factor;

    global_cache_t() : score_sum(0), data_idx(0), is_ready(false) {
        data_pts.data_pss.resize(PSS_MAX_SIZE);
        data_pts.data_v1.resize(PSS_MAX_SIZE);
        data_pts.data_v2.resize(PSS_MAX_SIZE);
        data_pts.data_path.resize(PSS_MAX_SIZE);
        data_pts.data_spContrib.resize(PSS_MAX_SIZE);
        data_pts.data_pathWeight.resize(PSS_MAX_SIZE);
        inv_sigma_sq = inverse(CACHE_SIG * CACHE_SIG);
        factor = std::exp(dim * (Float(0.5) * std::log(inv_sigma_sq) - Float(0.9189385332046727)));
    }  

    inline std::mutex &getMutex() {
        return mutex; 
    }

    inline bool isReady() const {
        return is_ready; 
    }

    inline bool push(const std::vector<Float> &pss, 
                     const std::vector<Float> &v1, 
                     const std::vector<Float> &v2,
                     const Path &path, 
                     const SubpathContrib &spContrib,
                     const Float pathWeight) {
        if (is_ready)    return false; 
        data_pts.data_pss[data_idx] = pss;
        data_pts.data_v1[data_idx] = v1; 
        data_pts.data_v2[data_idx] = v2; 
        data_pts.data_path[data_idx] = path; 
        data_pts.data_spContrib[data_idx] = spContrib;
        data_pts.data_pathWeight[data_idx] = pathWeight; 
        score_sum += pathWeight; 
        data_idx += 1;
        if (data_idx >= PSS_MAX_SIZE) {
            data_tree = new KDTree(dim, data_pts, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
            data_tree->buildIndex();
            data_distrib = std::make_shared<PiecewiseConstant1D>(
                &data_pts.data_pathWeight[0], data_pts.data_pathWeight.size());
            is_ready = true; 
            // std::cout << " Cache[" << dim << "] built." << std::endl;
        }
        return true; 
    }
    
    inline bool query(const std::vector<Float> &pss,
                      std::vector<Float> &v1,
                      std::vector<Float> &v2) {
        if (!is_ready)    return false; 
        const int knn = 5; 
        const Float radius = dim * (PSS_QUERY_DIST * PSS_QUERY_DIST); // nanoflann uses squared L2 distance
        std::vector<std::pair<size_t, Float>> ret_matches;
        SearchParams params; 
        const size_t nMatches = data_tree->radiusSearch(&pss[0], radius, ret_matches, params, knn);
        if (!nMatches)    return false;
        double sum_w = 0;
        std::fill(v1.begin(), v1.end(), Float(0.0));
        std::fill(v2.begin(), v2.end(), Float(0.0));
        for (int k = 0; k < nMatches; k++) {
            int index = static_cast<int>(ret_matches[k].first);
            Float dist = ret_matches[k].second; 
            Float w = inverse(dist * dist + Float(1e-6));
            for (int i = 0; i < dim; i++) {
                v1[i] += data_pts.data_v1[index][i] * w; 
                v2[i] += data_pts.data_v2[index][i] * w;
            }
            sum_w += w; 
        }
        for (int i = 0; i < dim; i++) {
            v1[i] /= sum_w; 
            v2[i] /= sum_w;
        }
        return true;         
    }
     
    void sampleCache(Path &path, std::vector<Float> &pss, 
        SubpathContrib &spContrib, Float &pathWeight, RNG &rng) {
        assert(is_ready);
        std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
        int idx = data_distrib->SampleDiscrete(uniDist(rng), nullptr);
        assert(idx >= 0 && idx < PSS_MAX_SIZE);
        path = data_pts.data_path[idx];
        pss = data_pts.data_pss[idx];
        spContrib = data_pts.data_spContrib[idx];
        pathWeight = data_pts.data_pathWeight[idx];
        return ;
    }

    Float evalPdfCache(const std::vector<Float> &pss_query, const Path &path) {
        assert(is_ready);
        assert(pss_query.size() == dim); 
#if 1
        Float ret(0.0);
        for (int i = 0; i < PSS_MAX_SIZE; i++) {
            const std::vector<Float> &pss = data_pts.data_pss[i];
            const SubpathContrib &spContrib = data_pts.data_spContrib[i];
            if (spContrib.camDepth != path.camDepth || spContrib.lightDepth != path.lgtDepth)
                continue;    
            Float sumDistSqr = 0;
            for (int j = 0; j < dim; j++) {
                Float c = pss[j], q = pss_query[j];
                Float d1 = fabs(q - c);
                Float d2 = Float(1.0) - d1;
                Float d = std::min(d1, d2);
                sumDistSqr += d * d;
            }
            Float expo = -Float(0.5) * sumDistSqr * inv_sigma_sq;
            Float scale = factor * data_pts.data_pathWeight[i] / score_sum;
            ret += std::exp(expo) * scale; 
        }
#endif 
        return ret;
    }
};

struct GlobalCache {
    global_cache_t<2> global_cache_2;
    global_cache_t<3> global_cache_3;
    global_cache_t<4> global_cache_4;
    global_cache_t<5> global_cache_5;
    global_cache_t<6> global_cache_6;
    global_cache_t<7> global_cache_7;
    global_cache_t<8> global_cache_8;
    global_cache_t<9> global_cache_9;
    global_cache_t<10> global_cache_10;
    global_cache_t<11> global_cache_11;
    global_cache_t<12> global_cache_12;
    global_cache_t<13> global_cache_13;
    global_cache_t<14> global_cache_14;
    global_cache_t<15> global_cache_15;
    global_cache_t<16> global_cache_16;

    bool isReady(const int dim) const {
        switch(dim) {
            case 2: {
                return global_cache_2.isReady();
            }
            case 3: {
                return global_cache_3.isReady();
            }
            case 4: {
                return global_cache_4.isReady();
            }
            case 5: {
                return global_cache_5.isReady();
            }
            case 6: {
                return global_cache_6.isReady();
            }
            case 7: {
                return global_cache_7.isReady();
            }
            case 8: {
                return global_cache_8.isReady();
            }
            case 9: {
                return global_cache_9.isReady();
            }
            case 10: {
                return global_cache_10.isReady();
            }
            case 11: {
                return global_cache_11.isReady();
            }
            case 12: {
                return global_cache_12.isReady();
            }
            case 13: {
                return global_cache_13.isReady();
            }
            case 14: {
                return global_cache_14.isReady();
            }
            case 15: {
                return global_cache_15.isReady();
            }
            case 16: {
                return global_cache_16.isReady();
            }
            default: {
                std::cerr << "isReady() dim: " << dim << " is not supported for global caching!" << std::endl;
            }
        }
        return false;
    }

    std::mutex &getMutex(const int dim) {
        switch(dim) {
            case 2: {
                return global_cache_2.getMutex();
            }
            case 3: {
                return global_cache_3.getMutex();
            }
            case 4: {
                return global_cache_4.getMutex();
            }
            case 5: {
                return global_cache_5.getMutex();
            }
            case 6: {
                return global_cache_6.getMutex();
            }
            case 7: {
                return global_cache_7.getMutex();
            }
            case 8: {
                return global_cache_8.getMutex();
            }
            case 9: {
                return global_cache_9.getMutex();
            }
            case 10: {
                return global_cache_10.getMutex();
            }
            case 11: {
                return global_cache_11.getMutex();
            }
            case 12: {
                return global_cache_12.getMutex();
            }
            case 13: {
                return global_cache_13.getMutex();
            }
            case 14: {
                return global_cache_14.getMutex();
            }
            case 15: {
                return global_cache_15.getMutex();
            }
            case 16: {
                return global_cache_16.getMutex();
            }
            default: {
                std::cerr << "getMutex() dim: " << dim << " is not supported for global caching!" << std::endl;
            }
        }
        // should not reach here
        assert(false);
        return global_cache_2.getMutex();
    }
    
    bool push(
        const int dim, 
        const std::vector<Float> &pss, 
        const std::vector<Float> &v1, 
        const std::vector<Float> &v2,
        const Path &path, 
        const SubpathContrib &spContrib,
        const Float pathWeight
    ) {
        // const size_t dim = pss.size();
        bool ret = false; 
        switch(dim) {
            case 2: {
                ret = global_cache_2.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 3: {
                ret = global_cache_3.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 4: {
                ret = global_cache_4.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 5: {
                ret = global_cache_5.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 6: {
                ret = global_cache_6.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 7: {
                ret = global_cache_7.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 8: {
                ret = global_cache_8.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 9: {
                ret = global_cache_9.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 10: {
                ret = global_cache_10.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 11: {
                ret = global_cache_11.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 12: {
                ret = global_cache_12.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 13: {
                ret = global_cache_13.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 14: {
                ret = global_cache_14.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 15: {
                ret = global_cache_15.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            case 16: {
                ret = global_cache_16.push(pss, v1, v2, path, spContrib, pathWeight);
                break;  
            }
            default: {
                std::cerr << "push() dim: " << dim << " not supported!" << std::endl;
            }
        }
        return ret; 
    }

    bool query(
        const int dim,
        const std::vector<Float> &pss, 
              std::vector<Float> &v1,
              std::vector<Float> &v2
    ) {
        // const size_t dim = pss.size(); 
        bool ret = false;
        switch(dim) {
            case 2: {
                ret = global_cache_2.query(pss, v1, v2);
                break;  
            }
            case 3: {
                ret = global_cache_3.query(pss, v1, v2);
                break;  
            }
            case 4: {
                ret = global_cache_4.query(pss, v1, v2);
                break;  
            }
            case 5: {
                ret = global_cache_5.query(pss, v1, v2);
                break;  
            }
            case 6: {
                ret = global_cache_6.query(pss, v1, v2);
                break;  
            }
            case 7: {
                ret = global_cache_7.query(pss, v1, v2);
                break;  
            }
            case 8: {
                ret = global_cache_8.query(pss, v1, v2);
                break;  
            }
            case 9: {
                ret = global_cache_9.query(pss, v1, v2);
                break;  
            }
            case 10: {
                ret = global_cache_10.query(pss, v1, v2);
                break;  
            }
            case 11: {
                ret = global_cache_11.query(pss, v1, v2);
                break;  
            }
            case 12: {
                ret = global_cache_12.query(pss, v1, v2);
                break;  
            }
            case 13: {
                ret = global_cache_13.query(pss, v1, v2);
                break;  
            }
            case 14: {
                ret = global_cache_14.query(pss, v1, v2);
                break;  
            }
            case 15: {
                ret = global_cache_15.query(pss, v1, v2);
                break;  
            }
            case 16: {
                ret = global_cache_16.query(pss, v1, v2);
                break;  
            }
            default: {
                std::cerr << "query() dim: " << dim << " not supported!" << std::endl;
            }
        }
        return ret; 
    }

    void sampleCache(const int dim, Path &path, std::vector<Float> &pss, 
                     SubpathContrib &spContrib, Float &pathWeight, RNG &rng) {
        switch (dim) {
            case 2: 
                global_cache_2.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 3: 
                global_cache_3.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 4: 
                global_cache_4.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 5: 
                global_cache_5.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 6: 
                global_cache_6.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 7: 
                global_cache_7.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 8: 
                global_cache_8.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 9: 
                global_cache_9.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 10: 
                global_cache_10.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 11: 
                global_cache_11.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 12: 
                global_cache_12.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 13: 
                global_cache_13.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 14: 
                global_cache_14.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 15: 
                global_cache_15.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            case 16: 
                global_cache_16.sampleCache(path, pss, spContrib, pathWeight, rng);
                break; 
            default: 
                std::cerr << "sampleCache() dim: " << dim << " not supported!" << std::endl;
        }  
        return ;   
    }
 
    Float evalPdfCache(const int dim, const std::vector<Float> pss, 
                       const Path &path) {
        assert(dim == pss.size());
        Float ret = Float(0.0);
        switch (dim) {
            case 2: 
                ret = global_cache_2.evalPdfCache(pss, path);
                break;
            case 3: 
                ret = global_cache_3.evalPdfCache(pss, path);
                break; 
            case 4: 
                ret = global_cache_4.evalPdfCache(pss, path);
                break; 
            case 5: 
                ret = global_cache_5.evalPdfCache(pss, path);
                break; 
            case 6: 
                ret = global_cache_6.evalPdfCache(pss, path);
                break; 
            case 7: 
                ret = global_cache_7.evalPdfCache(pss, path);
                break; 
            case 8: 
                ret = global_cache_8.evalPdfCache(pss, path);
                break; 
            case 9: 
                ret = global_cache_9.evalPdfCache(pss, path);
                break; 
            case 10: 
                ret = global_cache_10.evalPdfCache(pss, path);
                break; 
            case 11: 
                ret = global_cache_11.evalPdfCache(pss, path);
                break; 
            case 12: 
                ret = global_cache_12.evalPdfCache(pss, path);
                break; 
            case 13: 
                ret = global_cache_13.evalPdfCache(pss, path);
                break; 
            case 14: 
                ret = global_cache_14.evalPdfCache(pss, path);
                break; 
            case 15: 
                ret = global_cache_15.evalPdfCache(pss, path);
                break; 
            case 16: 
                ret = global_cache_16.evalPdfCache(pss, path);
                break; 
            default: 
                std::cerr << "evalPdfCache() dim: " << dim << " not supported!" << std::endl;
                break;             
        }
        return ret;
    }
    
};