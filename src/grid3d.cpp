//
// Created by himalaya on 12/6/20.
//

#include "grid3d.h"
#include <cmath> // round(), ceil
#include <fstream>

using namespace std;

GridBase::GridBase(DimUnit depth, DimUnit height, DimUnit width):
        m_depth(depth), m_height(height), m_width(width){
}

GridBase::GridBase():m_depth(0), m_height(0), m_width(0) {}

DimUnit GridBase::getDepth() const {
    return this->m_depth;
}

DimUnit GridBase::getWidth() const {
    return this->m_width;
}

DimUnit GridBase::getHeight() const {
    return this->m_height;
}

void GridBase::setDepth(int depth) {
    if (depth <= 0)
        cerr << "Cannot accept non-positive value." << "\n";
    m_depth = depth;
}

void GridBase::setHeight(int height) {
    if (height <= 0)
        cerr << "Cannot accept non-positive value." << "\n";
    m_height = height;
}

void GridBase::setWidth(int width) {
    if (width <= 0)
        cerr << "Cannot accept non-positive value." << "\n";
    m_width = width;
}

void Grid3d::savePhi(const std::string &file_path) {
    ofstream fout;
    fout.open(file_path, ios::binary | ios::out);
    if (!fout.is_open()) {
        cerr << "file not exits" << endl;
        exit(EXIT_FAILURE);
    }
    fout.write(reinterpret_cast<const char*>(&this->phi[0]), m_depth * m_height * m_width * sizeof(dtype));
    fout.close();
}

void Grid3d::readPhi(const std::string &file_path) {
    ifstream fin;
    fin.open(file_path, ios::binary | ios::in);
    if (!fin.is_open()) {
        cerr << "file not exits" << endl;
        exit(EXIT_FAILURE);
    }
    fin.read(reinterpret_cast<char *>(&this->phi[0]),m_depth * m_height * m_width * sizeof(dtype));
    fin.close();
}

Grid3d::Grid3d(DimUnit depth, DimUnit height, DimUnit width) : GridBase(depth, height, width) {

}

Grid3d::Grid3d(): GridBase() {

}

dtype Grid3d::getResolution() const {
    if (m_resolution <= 0) {
        cerr << "Resolution is not set yet." << endl;
    }
    return m_resolution;
}

void Grid3d::Init() {
    unsigned long total_num = m_height * m_width * m_depth;
    this->phi.resize(total_num, -INF);
    this->coord.resize(total_num);
}

void Grid3d::initCoord(Vec3 origin, dtype resolution) {
    m_resolution = resolution;
    m_min_coord = origin;
    m_max_coord = origin + Vec3(m_width, m_height, m_depth) * m_resolution;
    for (int i = 0; i < m_depth; i++) {
        for (int j = 0; j < m_height; j++) {
            for (int k = 0; k < m_width; k++) {
                auto idx = this->Index(i, j, k);
                this->coord[idx][0] = origin[0] + k * resolution;       // x
                this->coord[idx][1] = origin[1] + j * resolution;       // y
                this->coord[idx][2] = origin[2] + i * resolution;       // z
                this->coord[idx][3] = 1;                                // 1 for homogeneous coordinate
            }
        }
    }
}

TSDF::TSDF() {}

void TSDF::setDelta(dtype delta) {
    if (!(delta > 0 && delta < .005)) {
        cerr << "Recommended delta value is between 0 and 0.005m" << "\n";
    }
    m_delta = delta;
}

void TSDF::setEta(dtype eta) {
    m_eta = eta;
}

dtype TSDF::getDelta() const {
    if (m_delta == 0) {
        cerr << "delta is not set yet." << "\n";
    }
    return m_delta;
}

dtype TSDF::getEta() const {
    if (m_eta == 0) {
        cerr << "eta is not set yet." << "\n";
    }
    return m_eta;
}

Vec3 TSDF::getMinCoord() const {
    return m_min_coord;
}

void TSDF::setPaddingSize(int size) {
    m_padding_size = size;
}

void TSDF::setIntrinsic(const Mat3 &K) {
    m_fx = K(0, 0);
    m_fy = K(1, 1);
    m_cx = K(0, 2);
    m_cy = K(1, 2);
    m_1_fx = 1. / m_fx;
    m_1_fy = 1. / m_fy;
}

void TSDF::Init(const cv::Mat &depth_image, const cv::Mat &mask, dtype resolution) {
    if (m_fx == 0) {
        cerr << "Intrinsics not set yet." << "\n";
    }
    if (m_eta == 0 || m_delta == 0) {
        cerr << "delta or eta not set yet." << "\n";
    }
    m_resolution = resolution;
    vector<cv::Point> idx_roi;
    cv::findNonZero(mask, idx_roi);
    // determine the volume of the reference frame.
    double z_min, z_max;
    cv::minMaxIdx(depth_image, &z_min, &z_max, nullptr, nullptr, mask);
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
    m_min_coord(0) = m_1_fx * (ux_min - m_cx) * z_max;      // multiply with z_max instead of z_min for more tolerance
    m_min_coord(1) = m_1_fy * (uy_min - m_cy) * z_max;      // multiply with z_max instead of z_min for more tolerance
    m_max_coord(0) = m_1_fx * (ux_max - m_cx) * z_max;
    m_max_coord(1) = m_1_fy * (uy_max - m_cy) * z_max;

//    cout << "max coord before padding: " << m_max_coord << endl;

    m_min_coord -= Vec3(1., 1., 1.) * resolution * m_padding_size;
    m_max_coord += Vec3(1., 1., 1.) * resolution * m_padding_size;
    m_depth = ceil((m_max_coord(2) - m_min_coord(2)) / resolution) + 1;
    m_height = ceil((m_max_coord(1) - m_min_coord(1)) / resolution) + 1;
    m_width = ceil((m_max_coord(0) - m_min_coord(0)) / resolution) + 1;
    m_max_coord = m_min_coord + Vec3(m_width, m_height, m_depth) * resolution;
//    cout << "max coord after padding" << m_max_coord << endl;
    this->Grid3d::Init();
    m_weight.resize(phi.size(), 0);
    // compute truncated signed distance value (level set value)
    // todo: parallelization
    for (int i = 0; i < m_depth; i++) {
        for (int j = 0; j < m_height; j++) {
            for (int k = 0; k < m_width; k++) {
                auto idx = this->Index(i, j, k);
                this->coord[idx][0] = m_min_coord(0) + (k + .5) * resolution;       // x
                this->coord[idx][1] = m_min_coord(1) + (j + .5) * resolution;       // y
                this->coord[idx][2] = m_min_coord(2) + (i + .5) * resolution;       // z
                this->coord[idx][3] = 1;                                // 1 for homogeneous coordinate
                dtype z = coord[idx][2];
                dtype ux = round(coord[idx][0] * m_fx / z + m_cx);
                dtype uy = round(coord[idx][1] * m_fy / z + m_cy);
                dtype phi_val = depth_image.at<dtype>(uy, ux) - coord[idx][2];
                if (phi_val <= -m_delta)
                    phi[idx] = -1.;
                else if (phi_val >= m_delta)
                    phi[idx] = 1.;
                else
                    phi[idx] = phi_val / m_delta;
                if (phi_val > -m_eta)
                    m_weight[idx] = 1.;
            }
        }
    }
//    ofstream fout;
//    fout.open("../res/dist.txt", ios::out);
//    for (int z = 0; z < m_depth; z++) {
//        fout << "z = " << z << endl;
//        for (int x = 0; x < m_width; x++)
//        {fout << scientific << setprecision(3) << setw(5) << setfill('0') << (float)x << " "; }
//        fout << endl;
//        for (int y = 0; y < m_height; y++) {
//
//            for (int x = 0 ; x < m_width; x++) {
//                fout << scientific << setprecision(3) << setw(5) << setfill('0') << phi[this->Index(z, y, x)] << " ";
//            }
//            fout << endl;
//        }
//        fout << endl;
//    }
//    fout.close();

//    ofstream fout;
//    fout.open("../res/dist.txt", ios::out);
//    for (int x = 0; x < m_width; x++) {
//        fout << "x = " << x << endl;
//        for (int z = 0; z < m_depth; z++)
//        {fout << scientific << setprecision(3) << setw(5) << setfill('0') << (float)z << " "; }
//        fout << endl;
//        for (int y = 0; y < m_height; y++) {
//
//            for (int z = 0 ; z < m_depth; z++) {
//                fout << scientific << setprecision(3) << setw(5) << setfill('0') << phi[this->Index(z, y, x)] << " ";
//            }
//            fout << endl;
//        }
//        fout << endl;
//    }
//    fout.close();
}

#include <opencv2/highgui.hpp>
dtype TSDF::computePhiWeight(const cv::Mat &cur_depth_image, const cv::Mat &mask,
                             int i, int j, int k, const Mat4 &T_mat, dtype &weight) {
    auto idx = Index(i, j, k);
    auto cur_coord = coord[idx];        // coordinate in current frame
    cur_coord = T_mat * cur_coord;
    dtype z = cur_coord(2);
    dtype x = cur_coord(0), y = cur_coord(1);
    int ux = round(m_fx * x / z + m_cx);
    int uy = round(m_fy * y / z + m_cy);
    // the projection is out of the image scope (or ROI), so this 3D point must lie in the exterior of the object
    if (ux < 0 || ux >= mask.cols || uy < 0 || uy >= mask.rows || !mask.at<uchar>(uy, ux)) {
        weight = 1.;
        return 1.;
    }
    dtype phi_val = cur_depth_image.at<dtype>(uy, ux) - z;
    weight = phi_val > -m_eta ? 1. : 0;
    if (phi_val <= -m_delta) {
        return -1.;
    }
    else if (phi_val > m_delta)
        return 1.;
    else
        return phi_val / m_delta;
}

bool TSDF::computeGradient(const Mat4 &T_mat, int i, int j, int k, Vec6 &gradient) {
    auto idx = Index(i, j, k);
    Vec3 dphi_X;                            // derivative of phi w.r.t X
    vector<vector<int>> dirs{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int index = 0; index < 3; index++) {
        dtype post_phi = phi[Index(i + dirs[index][0], j + dirs[index][1], k + dirs[index][2])];
        dtype pre_phi = phi[Index(i - dirs[index][0], j - dirs[index][0], k - dirs[index][2])];
        dtype diff = post_phi - pre_phi;
        diff /= 2;
        // if the difference is exactly 1, we regard this voxel as beam region and set the gradient as  0
        if (abs(diff) == 1) {
            gradient.setZero();
            return false;
        }
        dphi_X(index) = diff;
    }
    if (!dphi_X.any()) {    // zero gradient means X locates far exterior of the object.
        gradient.setZero();
        return false;
    }
    auto ref_coord = coord[idx];            // coordinate in reference frame
    Eigen::Matrix<dtype, 3, 6> dX_twist;    // derivative of X w.r.t. twist.
    dX_twist.leftCols(3).setIdentity();
    Vec4 cur_coord = T_mat * ref_coord;     // coordinate in current frame
    Mat3 cur_coord_hat;
    cur_coord_hat << 0, -cur_coord(2), cur_coord(1),
            cur_coord(2), 0, -cur_coord(0),
            -cur_coord(1), cur_coord(0), 0;
    dX_twist.rightCols(3) = -cur_coord_hat;

    gradient = dX_twist.transpose() * dphi_X;
    return true;
}

Vec6 TSDF::estimateTwist(const cv::Mat &ref_depth_image, const cv::Mat &cur_depth_image, const cv::Mat &ref_mask,
                         const cv::Mat &cur_mask, dtype resolution,
                         int max_num_iteration,
                         dtype time_step) {
    this->Init(ref_depth_image, ref_mask, resolution);

    Vec6 twist;
    twist << 0, 0, 0, 0, 0, 0;
    int iter_count = 0;
    while (iter_count++ < max_num_iteration) {
        cout << "Iteration " << iter_count << ": \n";
        // todo: parallelization (or we can parallelize the whole process since aligning frame is a single task
        //  and we need to do it for multiple times).
        dtype err = 0;
        Eigen::Matrix<dtype, 6, 6> A;
        A.setZero();
        Vec6 b;
        b.setZero();
        Mat4 T_mat = SE3::exp(twist).matrix();
        // start from 1 and end at size - 1 for correctly computing central difference
        for (int i = 1; i < m_depth - 1; i++) {
            for (int j = 1; j < m_height - 1; j++) {
                for (int k = 1; k < m_width - 1; k++) {
                    dtype cur_weight, cur_phi;
                    cur_phi = computePhiWeight(cur_depth_image, cur_mask, i, j, k, T_mat, cur_weight);
                    // todo: error computation can be eliminated when process
                    auto idx = Index(i, j, k);
                    dtype ref_weight = m_weight[idx];
                    dtype ref_phi = phi[idx];
                    if (cur_weight == 0) {  // no contribution to gradient
                        err += pow(ref_weight * ref_phi, 2);
                        continue;
                    }
                    else {
                        err += pow(ref_weight * ref_phi - cur_weight * cur_phi, 2);
                    }
                    Vec6 dphi_twist;    // derivative of phi w.r.t. twist
                    if (!computeGradient(T_mat, i, j, k, dphi_twist)) {     // beam region or zero gradient region
                        continue;
                    }
                    auto dphi_twist_T = dphi_twist.transpose();
                    A += dphi_twist * dphi_twist_T;
                    b += (ref_phi * ref_weight - cur_phi + dphi_twist_T * twist) * dphi_twist;
                }
            }
        }
        cout << "A is: \n" << A << "\n";
        cout << "Registration error is: " << err / 2 << "\n";
        cout << "Twist is: " << twist.transpose() << "\n";
        cout << "Transformation Matrix is: \n" << SE3::exp(twist).matrix() << "\n";
        Vec6 approx_twist = A.inverse() * b;
        twist += time_step * (approx_twist - twist);
    }
    return twist;
}

void TSDF::computeAnotherPhi(const cv::Mat &depth_image, std::vector<dtype> &phi) {
    phi.resize(this->phi.size());
    for (int i = 0; i < m_depth; i++) {
        for (int j = 0; j < m_height; j++) {
            for (int k = 0; k < m_width; k++) {
                auto idx = this->Index(i, j, k);
                dtype z = coord[idx][2];
                dtype ux = round(coord[idx][0] * m_fx / z + m_cx);
                dtype uy = round(coord[idx][1] * m_fy / z + m_cy);
                phi[idx] = depth_image.at<dtype>(uy, ux) - coord[idx][2];
            }
        }
    }
}