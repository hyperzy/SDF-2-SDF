//
// Created by himalaya on 12/7/20.
//

#include "algorithms.h"
#include <cmath>

using namespace std;
void testAll() {
    auto depth_img = std::make_shared<DepthImage>();
    depth_img->loadImage("../res/Synthetic_Bunny_Circle/depth_000000.exr");
    depth_img->showImage();
}

void OperatorTSDF::setDelta(dtype delta) {
    if (!(delta > 0 && delta < .005)) {
        cerr << "Recommended delta value is between 0 and 0.005m" << "\n";
    }
    m_delta = delta;
}

void OperatorTSDF::setEta(dtype eta) {
    m_eta = eta;
}

dtype OperatorTSDF::getDelta() const {
    if (m_delta == 0) {
        cerr << "delta is not set yet." << "\n";
    }
    return m_delta;
}

dtype OperatorTSDF::getEta() const {
    if (m_eta == 0) {
        cerr << "eta is not set yet." << "\n";
    }
    return m_eta;
}

void OperatorTSDF::setPaddingSize(int size) {
    m_padding_size = size;
}

void OperatorTSDF::setIntrinsic(const Mat3 &K) {
    m_fx = K(0, 0);
    m_fy = K(1, 1);
    m_cx = K(0, 2);
    m_cy = K(1, 2);
    m_1_fx = 1. / m_fx;
    m_1_fy = 1. / m_fy;
}

void OperatorTSDF::constructTSDF(Grid3d *p_grid, const cv::Mat &depth_image, dtype resolution,
                                      cv::InputOutputArray mask) {
    m_resolution = resolution;
    cv::Mat temp_mask;
    if (mask.empty()) {
        temp_mask = depth_image != INF;
    } else {
        temp_mask = mask.getMat();
    }
    vector<cv::Point> idx_roi;
    cv::findNonZero(temp_mask, idx_roi);
    // determine the volume of the reference frame.
    double z_min, z_max;
    cv::minMaxIdx(depth_image, &z_min, &z_max, nullptr, nullptr, temp_mask);
    m_min_coord(2) = z_min;
    m_max_coord(2) = z_max;
    // todo: parallelization of finding the min & max of x & y coord
    int ux_min = depth_image.cols, ux_max = 0;
    int uy_min = depth_image.rows, uy_max = 0;     // min & max of x & y in image coordinated system
    for (const auto &idx: idx_roi) {
        if (idx.x < ux_min) ux_min = idx.x;
        else if (idx.x > ux_max) ux_max = idx.x;
        if (idx.y < uy_min) uy_min = idx.y;
        else if (idx.y > uy_max) uy_max = idx.y;
    }
    m_min_coord(0) = m_1_fx * (ux_min - m_cx);
    m_min_coord(1) = m_1_fy * (uy_min - m_cy);
    m_max_coord(0) = m_1_fx * (ux_max - m_cx);
    m_max_coord(1) = m_1_fy * (uy_max - m_cy);

    cout << "max coord before padding: " << m_max_coord << endl;

    m_min_coord -= Vec3(resolution * m_padding_size);
    m_max_coord += Vec3(resolution * m_padding_size);
    int depth = ceil((m_max_coord(2) - m_min_coord(2)) / resolution);
    int height = ceil((m_max_coord(1) - m_min_coord(1)) / resolution);
    int width = ceil((m_max_coord(0) - m_min_coord(0)) / resolution);
    m_max_coord = m_min_coord + Vec3(width, height, depth) * resolution;
    cout << "max coord after padding" << m_max_coord << endl;
    p_grid->setDepth(depth);
    p_grid->setHeight(height);
    p_grid->setWidth(width);
    p_grid->Init();

}