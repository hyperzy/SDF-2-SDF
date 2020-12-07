//
// Created by himalaya on 12/7/20.
//

#ifndef SDF_2_SDF_ALGORITHMS_H
#define SDF_2_SDF_ALGORITHMS_H

#include "base.h"
#include "camera.h"
#include "grid3d.h"
#include <memory>

void testAll();

class OperatorTSDF {
    dtype m_delta = 0;
    dtype m_eta = 0;
    dtype m_fx, m_fy, m_cx, m_cy;
    dtype m_1_fx, m_1_fy;
    int m_padding_size = 3;
    Vec3 m_min_coord, m_max_coord;  // (x_min, y_min, z_min), (x_max, y_max, z_max)
    dtype m_resolution;
public:
    void setDelta(dtype delta);
    void setEta(dtype eta);
    dtype getDelta() const;
    dtype getEta() const;
    void setIntrinsic(const Mat3 &K);
    void setPaddingSize(int size);
    /**
     * @brief Determine the volume with respect to the referece depth image.
     * @param p_grid Pointer to the computational grid.
     * @param depth_image Input reference depth image.
     * @param mask Mask for the ROI. Null if the input depth image is a synthetic one (with known maximum INF)
     */
    void constructTSDF(Grid3d *p_grid, const cv::Mat &depth_image, dtype resolution, cv::InputOutputArray mask = cv::noArray());

    dtype getCurrentPhiWeight();
};

#endif //SDF_2_SDF_ALGORITHMS_H
