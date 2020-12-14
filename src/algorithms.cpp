//
// Created by himalaya on 12/7/20.
//

#include "algorithms.h"
#include "display.h"
#include <sstream>

using namespace std;
void testAll() {
    int num_images = 30;
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

    vector<Vec6> twists;
    twists.reserve(num_images);
    twists.emplace_back(Eigen::Matrix<dtype, 6, 1>::Identity());

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
        tsdf->setDelta(.002);
        tsdf->setEta(.002);
        tsdf->setPaddingSize(3);
        tsdf->setRender();
        auto start = std::chrono::high_resolution_clock::now();
        twists.emplace_back(tsdf->estimateTwist(p_ref->getImage(), p_cur->getImage(),
                                                ref_mask, cur_mask,
                                                resolution,
                                                15,
                                                .002,
                                                true));
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        cout << "duration: " << duration.count() << "ms" << endl;
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

