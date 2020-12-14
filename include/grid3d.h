//
// Created by himalaya on 12/6/20.
//

#ifndef SDF_2_SDF_GRID3D_H
#define SDF_2_SDF_GRID3D_H

#include "base.h"
#include <memory>

/****** Note ********/
/** 1-d array will be arranged as z->y->x order instead of intuitive x->y->z order
 *  so i, j, k correspond to z, y, x
 *  (depth, height, width)
 */

struct IndexSet {
    IndexSet(int i, int j, int k): i(i), j(j), k(k) {}
    int i, j, k;
};


class GridBase {
protected:
    // 0 for uninitialization
    DimUnit m_depth, m_height, m_width;
public:
    GridBase(DimUnit depth, DimUnit height, DimUnit width);
    GridBase();
    unsigned long Index(IdxType i, IdxType j, IdxType k) const;
    bool isValidRange(IdxType i, IdxType j, IdxType k) const;
    DimUnit getDepth() const;
    DimUnit getWidth() const;
    DimUnit getHeight() const;
    void setDepth(int depth);
    void setHeight(int height);
    void setWidth(int width);
    virtual void Init() = 0;
};

class Grid3d: public GridBase {
protected:
    dtype m_resolution;
    Vec3 m_min_coord;
    Vec3 m_max_coord;
public:
    // todo: dimension has not set yet
    Grid3d(DimUnit depth, DimUnit height, DimUnit width);
    Grid3d();
    // signed distance function
    std::vector<dtype> phi;
    std::vector<Vec4> coord;
    std::vector<IndexSet> front;
    void savePhi(const std::string &file_path);
    void readPhi(const std::string &file_path);
    void Init() override;
    // todo: use x_resolution, y_resolution, z_resolution
    void initCoord(Vec3 origin, dtype resolution);
    dtype getResolution() const;
};

inline unsigned long GridBase::Index(IdxType i, IdxType j, IdxType k) const {
    return i * m_width * m_height + j * m_width + k;
}

inline bool GridBase::isValidRange(IdxType i, IdxType j, IdxType k) const {
    return i >= 0 && i < m_depth && j >= 0 && j < m_height
           && k >= 0 && k < m_width;
}

inline unsigned long Index(int i, int j, int k, int height, int width) {
    return i * width * height + j * width + k;
}

// contain some critical variables for parallel processing TSDF volume
struct VarsVolumeParallel;

class TSDF: public Grid3d {
    dtype m_delta = 0;
    dtype m_eta = 0;
    dtype m_fx, m_fy, m_cx, m_cy;
    dtype m_1_fx, m_1_fy;
    int m_padding_size = 3;
    bool m_is_render = false;
public:
    TSDF();
    void setDelta(dtype delta);
    void setEta(dtype eta);
    dtype getDelta() const;
    dtype getEta() const;
    void setIntrinsic(const Mat3 &K);
    void setPaddingSize(int size);
    Vec3 getMinCoord() const;
    std::vector<dtype> m_weight;
    void setRender() {m_is_render = true;}
    /**
     * @brief Determine the volume with respect to the reference depth image.
     * @param depth_image Input reference depth image.
     * @param mask Mask for the ROI. Null if the input depth image is a synthetic one (with known maximum INF)
     * @param resolution
     */
    void Init(const cv::Mat &ref_depth_image, const cv::Mat &mask, dtype resolution);

    /**
     * @brief Compute phi vals & weights of voxels in the current frame
     * @param cur_depth_image
     * @param i
     * @param j
     * @param k
     * @param T_mat Transformation matrix mapping coordinate in ref-frame into cur-frame.
     * @param weight Output
     * @return phi value
     */
    dtype computePhiWeight(const cv::Mat &cur_depth_image, const cv::Mat &mask, int i, int j, int k,
                           const Mat4 &T_mat, dtype &weight);
    dtype computePhiWeight(const VarsVolumeParallel &client_data, int i, int j, int k, dtype &weight);


    /**
     * @brief Computing the gradient of phi val w.r.t. twist.
     * @param T_mat Transformation matrix that maps coordinate in ref-frame into cur-frame.
     * @param i
     * @param j
     * @param k
     * @param gradient Gradient vector in shape 6x1.
     * @return True if valid gradient exists else false.
     */
    bool computeGradient(const Mat4 &T_mat, int i, int j, int k, Vec6 &gradient);
    /**
     * @brief Method for parallel computing.
     * @param T_mat
     * @param i
     * @param j
     * @param k
     * @param volume_height
     * @param volume_width
     * @param gradient
     * @return
     */
    bool computeGradient(const VarsVolumeParallel &client_data, int i, int j, int k, Vec6 &gradient);

    /**
     * @brief Estimating the twist from refrence frame to
     * @param ref_depth_image
     * @param cur_depth_image
     * @param ref_mask
     * @param cur_mask
     * @param resolution
     * @param max_num_iteration
     * @param time_step
     * @param is_parallel Determine if use multi-threads processing
     * @param num_threads The default number of threads if 8. -1 for maximum feasible number. Must be non-negative
     *                     except -1.
     * @return Twist of 6 DoF.
     */
    Vec6 estimateTwist(const cv::Mat &ref_depth_image, const cv::Mat &cur_depth_image,
                       const cv::Mat &ref_mask, const cv::Mat &cur_mask,
                       dtype resolution,
                       int max_num_iteration,
                       dtype time_step,
                       bool is_parallel = false,
                       int num_threads = 8);


    void computeAnotherPhi(const cv::Mat &depth_image, const cv::Mat &mask, std::vector<dtype> &phi, const Mat4 &T_mat);

    void parallelIterateVolume(const VarsVolumeParallel &client_data,
                               int i_start, int i_end,
                               dtype &err, Eigen::Matrix<dtype, 6, 6> &A, Vec6 &b);
};
#endif //SDF_2_SDF_GRID3D_H
