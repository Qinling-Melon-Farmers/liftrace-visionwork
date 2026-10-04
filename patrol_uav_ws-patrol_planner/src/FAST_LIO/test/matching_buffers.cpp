// Compile and exercise the production matcher, including its real ikd-tree and
// OpenMP loop. The ROS node entry point is never called; no master is required.
#define main fast_lio_node_main
#include "../src/laserMapping.cpp"
#undef main

#include <stdexcept>
#include <type_traits>

static_assert(std::is_same<decltype(point_selected_surf)::value_type, uint8_t>::value,
              "Parallel selection must use independently writable bytes");

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void set_scan(size_t count, bool reject_all = false)
{
    feats_down_size = static_cast<int>(count);
    feats_down_body->resize(count);
    feats_down_world->resize(count);
    normvec->resize(count);
    Nearest_Points.resize(count);
    for (size_t i = 0; i < count; ++i)
    {
        PointType point{};
        point.x = 0.01f * static_cast<float>(i % 7);
        point.y = 0.02f * static_cast<float>(i % 5);
        point.z = (reject_all || i % 2) ? 10.0f : 1.01f;
        point.intensity = static_cast<float>(i);  // Verify stable compaction order.
        feats_down_body->points[i] = point;
    }
}

void check_match(state_ikfom &state, bool search, size_t expected)
{
    esekfom::dyn_share_datastruct<double> data;
    data.valid = true;
    data.converge = search;
    h_share_model(state, data);
    require(point_selected_surf.size() == static_cast<size_t>(feats_down_size),
            "Selection buffer must track the current scan");
    require(res_last.size() == static_cast<size_t>(feats_down_size),
            "Residual buffer must track the current scan");
    require(laserCloudOri->size() == expected && corr_normvect->size() == expected,
            "Compacted point clouds must have their actual logical size");
    require(effct_feat_num == static_cast<int>(expected), "Unexpected inlier count");
    if (expected == 0)
    {
        require(!data.valid, "An empty match must invalidate the measurement");
        return;
    }
    require(data.valid && data.h_x.rows() == static_cast<int>(expected) &&
            data.h_x.cols() == 12 && data.h.size() == static_cast<int>(expected),
            "Production Jacobian and residual dimensions differ from inliers");
    require(data.h_x.allFinite() && data.h.allFinite(), "Non-finite match output");
    for (size_t j = 0; j < expected; ++j)
    {
        require(laserCloudOri->points[j].intensity == static_cast<float>(2 * j),
                "Rejected points leaked into compaction or order changed");
        require(std::abs(data.h(j) - 0.01) < 1e-5,
                "Residual must equal the known offset from the z=1 plane");
        require(std::abs(data.h_x(j, 2) + 1.0) < 1e-5,
                "Jacobian must retain the known plane normal");
    }
}

int main()
{
    try
    {
        omp_set_dynamic(0);
        PointVector plane;
        for (const auto &xy : std::vector<std::pair<float, float>>{
                 {-1, -1}, {-1, 1}, {1, -1}, {1, 1}, {0, 0}})
        {
            PointType p{};
            p.x = xy.first; p.y = xy.second; p.z = 1;
            plane.push_back(p);
        }
        ikdtree.Build(plane);
        state_ikfom state;
        state.pos.setZero();
        state.rot = M3D::Identity();
        state.offset_T_L_I.setZero();
        state.offset_R_L_I = M3D::Identity();
        extrinsic_est_en = false;

        set_scan(0); check_match(state, true, 0);
        set_scan(9); check_match(state, true, 5);
        check_match(state, false, 5);  // Reuse real nearest neighbors and mask.
        set_scan(100017); check_match(state, true, 50009);
        check_match(state, false, 50009);
        set_scan(100017, true); check_match(state, true, 0);
        set_scan(3); check_match(state, true, 2);
        set_scan(0); check_match(state, true, 0);
        set_scan(100019); check_match(state, true, 50010);
        extrinsic_est_en = true;
        check_match(state, false, 50010);

        int workers = 1;
#ifdef MP_EN
        #pragma omp parallel reduction(max:workers)
        workers = omp_get_num_threads();
        require(workers == MP_PROC_NUM, "Configured OpenMP team was not exercised");
#endif
        std::cout << "PASS: 10 production matching cases; workers=" << workers
                  << "; maximum scan=100019; checked vector indexing\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
