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

//    cout << "max coord before padding:\n" << m_max_coord << endl;

    m_min_coord -= Vec3(1., 1., 1.) * resolution * m_padding_size;
    m_max_coord += Vec3(1., 1., 1.) * resolution * m_padding_size;
    m_depth = ceil((m_max_coord(2) - m_min_coord(2)) / resolution) + 1;
    m_height = ceil((m_max_coord(1) - m_min_coord(1)) / resolution) + 1;
    m_width = ceil((m_max_coord(0) - m_min_coord(0)) / resolution) + 1;
    m_max_coord = m_min_coord + Vec3(m_width, m_height, m_depth) * resolution;
//    cout << "max coord after padding:\n" << m_max_coord << endl;
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

dtype TSDF::computePhiWeight(const VarsVolumeParallel &client_data, int i, int j, int k, dtype &weight) {
    auto &height = client_data.volume_height, &width = client_data.volume_width;
    auto &T_mat = client_data.T_mat;
    auto &mask = client_data.cur_mask, &cur_depth_image = client_data.cur_depth_image;
    auto &eta = client_data.eta, &delta = client_data.delta;
    auto &fx = client_data.fx, &fy = client_data.fy, &cx = client_data.cx, &cy = client_data.cy;

    auto idx = ::Index(i, j, k, height, width);
    auto cur_coord = coord[idx];        // coordinate in current frame, vector coord is a shared member variable.
    cur_coord = T_mat * cur_coord;
    dtype z = cur_coord(2);
    dtype x = cur_coord(0), y = cur_coord(1);
    int ux = round(fx * x / z + cx);
    int uy = round(fy * y / z + cy);
    // the projection is out of the image scope (or ROI), so this 3D point must lie in the exterior of the object
    if (ux < 0 || ux >= mask.cols || uy < 0 || uy >= mask.rows || !mask.at<uchar>(uy, ux)) {
        weight = 1.;
        return 1.;
    }
    dtype phi_val = cur_depth_image.at<dtype>(uy, ux) - z;
    weight = phi_val > -eta ? 1. : 0;
    if (phi_val <= -delta) {
        return -1.;
    }
    else if (phi_val > delta)
        return 1.;
    else
        return phi_val / delta;
}

bool TSDF::computeGradient(const Mat4 &T_mat, int i, int j, int k, Vec6 &gradient) {
    auto idx = Index(i, j, k);
    Vec3 dphi_X;                            // derivative of phi w.r.t. X
    vector<vector<int>> dirs{{0, 0, 1}, {0, 1, 0}, {1, 0, 0}};
    for (int index = 0; index < 3; index++) {
        dtype post_phi = phi[Index(i + dirs[index][0], j + dirs[index][1], k + dirs[index][2])];
        dtype pre_phi = phi[Index(i - dirs[index][0], j - dirs[index][1], k - dirs[index][2])];
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

bool TSDF::computeGradient(const VarsVolumeParallel &client_data, int i, int j, int k, Vec6 &gradient) {
    auto &height = client_data.volume_height, &width = client_data.volume_width;
    auto &T_mat = client_data.T_mat;

    auto idx = ::Index(i, j, k, height, width);
    Vec3 dphi_X;                            // derivative of phi w.r.t. X
    vector<vector<int>> dirs{{0, 0, 1}, {0, 1, 0}, {1, 0, 0}};
    for (int index = 0; index < 3; index++) {
        dtype post_phi = phi[::Index(i + dirs[index][0], j + dirs[index][1], k + dirs[index][2],
                                    height, width)];
        dtype pre_phi = phi[::Index(i - dirs[index][0], j - dirs[index][1], k - dirs[index][2],
                                    height, width)];
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

#include "display.h"
Vec6 TSDF::estimateTwist(const cv::Mat &ref_depth_image, const cv::Mat &cur_depth_image, const cv::Mat &ref_mask,
                         const cv::Mat &cur_mask, dtype resolution,
                         int max_num_iteration,
                         dtype time_step,
                         bool is_parallel, int num_threads) {
    this->Init(ref_depth_image, ref_mask, resolution);

    Vec6 twist;
    twist << 0, 0, 0, 0, 0, 0;
    int iter_count = 0;
    vector<dtype> phi_cur(phi.size());
    Mat4 T_mat = SE3::exp(twist).matrix();

    auto displayer = make_shared<Display>();
    if (m_is_render) {
        displayer->Init();
        displayer->addAxes();
        displayer->addIsoSurface(phi, m_min_coord, m_depth, m_height, m_width, resolution);
        computeAnotherPhi(cur_depth_image, cur_mask, phi_cur, T_mat);
        displayer->addIsoSurface(phi_cur, m_min_coord, m_depth, m_height, m_width, resolution, "cur", "Silver");
        displayer->startRender();
    }

    while (iter_count++ < max_num_iteration) {
        cout << "Iteration " << iter_count << ": \n";
        dtype err = 0;
        Eigen::Matrix<dtype, 6, 6> A;
        A.setZero();
        Vec6 b;
        b.setZero();
        T_mat = SE3::exp(twist).matrix();
        if (is_parallel) {
            if (num_threads == -1) {
                num_threads = thread::hardware_concurrency();
            }
            else if (num_threads <= 0) {
                cerr << "wrong parameter: number of threads.\n";
                exit(EXIT_FAILURE);
            }
//            cout << num_threads << " threads are computing parallelly." << endl;
            vector<std::thread> threads;
            threads.reserve(num_threads);
            int num_slices_for_each_thread = m_depth / num_threads;
            int remaining_slices = m_depth % num_threads;
            int i_start = 0, offset = 0;      // assign one more slice to the first "remaining_slices"
            VarsVolumeParallel client_data(cur_depth_image, cur_mask, T_mat, m_height, m_width, m_delta,
                                           m_eta, m_fx, m_fy, m_cx, m_cy);
            vector<Eigen::Matrix<dtype, 6, 6>> As(num_threads);
            for_each(As.begin(), As.end(), [](Eigen::Matrix<dtype, 6, 6> &a) {a.setZero();});
            vector<Vec6> bs(num_threads);
            for_each(bs.begin(), bs.end(), [](Vec6 &b_) {b_.setZero();});
            vector<dtype> errs(num_threads, 0);
            for (int i = 0; i < num_threads; i++) {
                int start, end;
                if (remaining_slices-- > 0) {
                    offset = 1;
                }
                else {
                    offset = 0;
                }
                int i_end = i_start + num_slices_for_each_thread + offset;
                start = i_start - (i == 0 ? 0 : 1);     // padding 1 more slice for computing central difference
                end = i_end + (i == num_threads - 1 ? 0 : 1);
                threads.emplace_back(thread(&TSDF::parallelIterateVolume, this, client_data,
                                            start, end, std::ref(errs[i]), std::ref(As[i]), std::ref(bs[i])));
                i_start = i_end;
            }
            for (int i = 0; i < num_threads; i++) {
                threads[i].join();
            }
            // results reduction
            for (const auto &e: As) {
                A += e;
            }
            for (const auto &e: bs) {
                b += e;
            }
            for (const auto &e: errs) {
                err += e;
            }
        }
        else {
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
                        } else {
                            err += pow(ref_weight * ref_phi - cur_weight * cur_phi, 2);
                        }
                        Vec6 dphi_twist;    // derivative of phi w.r.t. twist
                        if (!computeGradient(T_mat, i, j, k, dphi_twist)) {     // beam region or zero gradient region
                            continue;
                        }
                        auto dphi_twist_T = dphi_twist.transpose();
                        A += dphi_twist * dphi_twist_T;
                        b += (ref_phi * ref_weight - cur_phi) * dphi_twist;
                    }
                }
            }
        }
//        cout << "A is: \n" << A << "\n";
//        cout << "Registration error is: " << err / 2 << "\n";
//        cout << "Twist is: " << twist.transpose() << "\n";
//        cout << "Pose Matrix is: \n" << SE3::exp(twist).inverse().matrix() << "\n";
        Vec6 delta_twist = A.inverse() * b;
        twist += time_step * delta_twist;
        if (m_is_render) {
            computeAnotherPhi(cur_depth_image, cur_mask, phi_cur, T_mat);
            displayer->updateIsoSurface(phi_cur, m_min_coord, m_depth, m_height, m_width, resolution, "cur");
            displayer->render();
        }
    }
    if (m_is_render) {
        displayer->close();
    }
    return twist;
}

void TSDF::computeAnotherPhi(const cv::Mat &depth_image, const cv::Mat &mask, std::vector<dtype> &phi,
                             const Mat4 &T_mat) {
    phi.resize(this->phi.size());
    for (int i = 0; i < m_depth; i++) {
        for (int j = 0; j < m_height; j++) {
            for (int k = 0; k < m_width; k++) {
                auto idx = this->Index(i, j, k);
                dtype weight;
                phi[idx] = computePhiWeight(depth_image, mask, i, j, k, T_mat, weight);
            }
        }
    }
}

void TSDF::parallelIterateVolume(const VarsVolumeParallel &client_data, int i_start, int i_end, dtype &err,
                                 Eigen::Matrix<dtype, 6, 6> &A, Vec6 &b) {
    auto &height = client_data.volume_height, &width = client_data.volume_width;
    auto &T_mat = client_data.T_mat;
    auto &cur_mask = client_data.cur_mask, &cur_depth_image = client_data.cur_depth_image;

    for (int i = i_start + 1; i < i_end - 1; i++) {
        for (int j = 1; j < height - 1; j++) {
            for (int k = 1; k < width - 1; k++) {
                dtype cur_weight, cur_phi;
                cur_phi = computePhiWeight(client_data, i, j, k, cur_weight);
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
                b += (ref_phi * ref_weight - cur_phi) * dphi_twist;
            }
        }
    }
}

void TSDF::parallelComputePhi(const VarsVolumeParallel &client_data, int i_start, int i_end, std::vector<dtype> &phi,
                              std::vector<dtype> &weight) {
    for (int i = i_start; i < i_end; i++) {
        for (int j = 0; j < client_data.volume_height; j++) {
            for (int k = 0; k < client_data.volume_width; k++) {
                auto idx = ::Index(i, j, k, client_data.volume_height, client_data.volume_width);
                phi[idx] = computePhiWeight(client_data, i, j, k, weight[idx]);
            }
        }
    }
}

void TSDF::parallelFuse(const VarsVolumeParallel &client_data, int i_start, int i_end,
                        const std::vector<dtype> &cur_phi, const std::vector<dtype> &cur_weight,
                        std::vector<dtype> &combined_phi, std::vector<dtype> &combined_weight) {
    int height = client_data.volume_height, width = client_data.volume_width;
    for (int i = i_start; i < i_end; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < width; k++) {
                auto idx = ::Index(i, j, k, height, width);
                const dtype &cur_w = cur_weight[idx];
                dtype &combined_w = combined_weight[idx];
                if (cur_w == 0 && combined_w == 0) {
                    continue;
                }
                else {
                    combined_phi[idx] = (cur_w * cur_phi[idx] + combined_w * combined_phi[idx]) / (cur_w + combined_w);
                    combined_w += cur_w;
                }
            }
        }
    }
}

void TSDF::parallelInitCoord(int i_start, int i_end, dtype resolution) {
    int height = m_height, width = m_width;
    Vec3 min_coord = m_min_coord;
    for (int i = i_start; i < i_end; i++) {
        for (int j = 0; j < height; j++) {
            for (int k = 0; k < width; k++) {
                auto idx = ::Index(i, j, k, height, width);
                this->coord[idx][0] = min_coord(0) + (k + .5) * resolution;       // x
                this->coord[idx][1] = min_coord(1) + (j + .5) * resolution;       // y
                this->coord[idx][2] = min_coord(2) + (i + .5) * resolution;       // z
                this->coord[idx][3] = 1;                                // 1 for homogeneous coordinate
            }
        }
    }
}

void TSDF::parallelInit(dtype resolution, int num_threads) {
    if (num_threads == -1) {
        num_threads = thread::hardware_concurrency();
    }
    else if (num_threads <= 0) {
        cerr << "wrong number of threads.\n";
        exit(EXIT_FAILURE);
    }
    vector<thread> threads;
    threads.reserve(num_threads);
    unsigned int size = m_height * m_width * m_depth;
    coord.resize(size);
    m_weight.resize(size);
    phi.resize(size);
    int num_slices_each_thread = m_depth / num_threads;
    int remaining_slices = m_depth % num_threads;
    int i_start = 0, i_end;

    for (int count = 0; count < num_threads; count++) {
        int offset = 0;
        if (remaining_slices-- > 0) offset = 1;
        i_end = i_start + num_slices_each_thread + offset;
        threads.emplace_back(thread(&TSDF::parallelInitCoord, this, i_start, i_end, resolution));
        i_start = i_end;
    }
    for (int count = 0; count < num_threads; count++) {
        threads[count].join();
    }
}