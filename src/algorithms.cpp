//
// Created by himalaya on 12/7/20.
//

#include "algorithms.h"
#include "display.h"
#include <sstream>

using namespace std;

void testAll() {
    int num_images = 120;
    shared_ptr<DepthImage> p_ref, p_cur;
    string file_name_prefix = "../res/Synthetic_Bunny_Circle/depth_";
    string file_name_suffix = ".exr";
    stringstream ss;
    ss << setw(6) << setfill('0') << 0;
    string file_name = file_name_prefix + ss.str() + file_name_suffix;
    p_ref = make_shared<DepthImage>();
    p_ref->loadImage(file_name);

    Mat3 K;
    K << 570.3999633789062,                   0,   320.0,
        0,   570.3999633789062,   240.0,
        0,                   0,       1;
    constexpr dtype resolution = .002;
    constexpr dtype delta = .002;
    constexpr dtype eta = .002;
    constexpr int padding_size = 3;

    vector<Vec6> twists;
    twists.reserve(num_images);
    twists.emplace_back(Eigen::Matrix<dtype, 6, 1>::Zero());

    auto combined_tsdf = make_shared<TSDF>();
    combined_tsdf->setIntrinsic(K);
    combined_tsdf->setDelta(delta);
    combined_tsdf->setEta(eta);

    // used for parallelly iterating the volume
    int num_slices_each_thread, remaining_slices;
    int num_threads = 8;
    vector<thread> threads;
    threads.resize(num_threads);
    VarsVolumeParallel client_data;

    for (int i = 1; i < num_images; i++) {
        p_cur = make_shared<DepthImage>();
        ss.str("");
        ss << setw(6) << setfill('0') << i;
        file_name = file_name_prefix + ss.str() + file_name_suffix;
        p_cur->loadImage(file_name);
        // for synthetic images only
        cv::Mat ref_mask = p_ref->getImage() != INF;
        cv::Mat cur_mask = p_cur->getImage() != INF;

        auto tsdf = make_shared<TSDF>();
        tsdf->setIntrinsic(K);
        tsdf->setDelta(delta);
        tsdf->setEta(eta);
        tsdf->setPaddingSize(padding_size);
//        tsdf->setRender();
        auto start = std::chrono::high_resolution_clock::now();
        twists.emplace_back(tsdf->estimateTwist(p_ref->getImage(), p_cur->getImage(),
                                                ref_mask, cur_mask,
                                                resolution,
                                                10,
                                                .002,
                                                true, -1));
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        cout << "duration: " << duration.count() << "ms" << endl;
        if (i == 1) {
            // initialize the combined_volume
            combined_tsdf->setHeight(tsdf->getHeight());
            combined_tsdf->setWidth(tsdf->getWidth());
            combined_tsdf->setDepth(.15 / resolution);   // the model thickness is 15cm
            combined_tsdf->setMinCoord(tsdf->getMinCoord());
            client_data.cur_depth_image = p_ref->getImage();
            client_data.cur_mask = ref_mask;
            client_data.T_mat = SE3::exp(twists[0]).matrix();
            client_data.delta = tsdf->getDelta();
            client_data.eta = tsdf->getEta();
            client_data.volume_height = combined_tsdf->getHeight();
            client_data.volume_width = combined_tsdf->getWidth();
            client_data.fx = K(0, 0);
            client_data.fy = K(1, 1);
            client_data.cx = K(0, 2);
            client_data.cy = K(1, 2);
            combined_tsdf->parallelInit(resolution);
            num_slices_each_thread = combined_tsdf->getDepth() / num_threads;
            remaining_slices = combined_tsdf->getDepth() % num_threads;
            int remaining = remaining_slices;
            int i_start = 0, i_end;
            for (int count = 0; count < num_threads; count++) {
                int offset = 0;
                if (remaining-- > 0) offset = 1;
                i_end = i_start + num_slices_each_thread + offset;
                threads[count] = thread(&TSDF::parallelComputePhi, combined_tsdf.get(), client_data, i_start, i_end,
                                            std::ref(combined_tsdf->phi), std::ref(combined_tsdf->m_weight));
                i_start = i_end;
            }
            for (int count = 0; count < num_threads; count++) {
                threads[count].join();
            }
        }
        // compute SDF of current frame
        int remaining = remaining_slices;
        int i_start = 0, i_end;
        // global phi and weight (used for reconstruction)
        vector<dtype> cur_phi(combined_tsdf->phi.size());
        vector<dtype> cur_weight(combined_tsdf->m_weight.size());
        client_data.cur_depth_image = p_cur->getImage();
        client_data.cur_mask = cur_mask;
        client_data.T_mat = SE3::exp(twists[i]).matrix() * client_data.T_mat;
        for (int count = 0; count < num_threads; count++) {
            int offset = 0;
            if (remaining-- > 0) offset = 1;
            i_end = i_start + num_slices_each_thread + offset;
            threads[count] = thread(&TSDF::parallelComputePhi, combined_tsdf.get(), client_data, i_start, i_end,
                                    std::ref(cur_phi), std::ref(cur_weight));
            i_start = i_end;
        }
        for (int count = 0; count < num_threads; count++) {
            threads[count].join();
        }
        // fuse
        remaining = remaining_slices;
        i_start = 0;
        for (int count = 0; count < num_threads; count++) {
            int offset = 0;
            if (remaining-- > 0) offset = 1;
            i_end = i_start + num_slices_each_thread + offset;
            threads[count] = thread(&TSDF::parallelFuse, combined_tsdf.get(), client_data, i_start, i_end,
                                    cur_phi, cur_weight,
                                    std::ref(combined_tsdf->phi), std::ref(combined_tsdf->m_weight));
            i_start = i_end;
        }
        for (int count = 0; count < num_threads; count++) {
            threads[count].join();
        }

        p_ref = p_cur;
    }

    vector<Mat4> T_mats(num_images);
    T_mats[0] = Mat4::Identity();
    for (int i = 1; i < num_images; i++) {
        T_mats[i] = SE3::exp(twists[i]).matrix() * T_mats[i - 1];
    }
    for (int i = 0; i < num_images; i++) {
        cout << "Transformation Matrix " << i << ":\n";
        cout << T_mats[i].inverse() << "\n";
    }

    auto displayer = make_shared<Display>();
    displayer->Init();
//    displayer->addAxes();
    displayer->addIsoSurface(combined_tsdf->phi, combined_tsdf->getMinCoord(),
                             combined_tsdf->getDepth(),
                             combined_tsdf->getHeight(),
                             combined_tsdf->getWidth(),
                             resolution);
    displayer->startRender();
    displayer->startInteractor();

//    auto tsdf = make_shared<TSDF>();
//    tsdf->setIntrinsic(p_depth_img->getIntrinsic());
//    tsdf->setDelta(0.002);
//    tsdf->setEta(.002);
//    tsdf->setPaddingSize(3);
//    dtype resolution = .002;
//    cv::Mat mask = p_depth_img->getImage() != INF;
//    tsdf->Init(p_depth_img->getImage(), mask, resolution);
//
//    tsdf->estimateTwist(p_depth_img->getImage(), p_depth_img->getImage(), mask, mask, resolution, 50, .4);
//
//    auto displayer = make_shared<Display>();
//    displayer->Init();
//    displayer->addAxes();
//    displayer->addIsoSurface(tsdf->phi, tsdf->getMinCoord(), tsdf->getDepth(), tsdf->getHeight(), tsdf->getWidth(), resolution);
//    displayer->startRender();
}

