//
// Created by himalaya on 12/6/20.
//

#ifndef SDF_2_SDF_CAMERA_H
#define SDF_2_SDF_CAMERA_H
#include "base.h"
#include <string>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "base.h"
#include <string>
#include <opencv2/imgcodecs/imgcodecs.hpp>

class ImageBase {
protected:
    cv::Mat m_img;
    const cv::Mat *m_operator_img_ptr;
    Eigen::Matrix<dtype, 3, 4> m_pose_mat;
    Mat3 m_intrinsic_mat;
    Mat3 m_rot_mat;
    Vec3 m_t_vec;
    bool r_set, t_set, i_set;
    Vec2 m_top_left, m_bottom_right;            // not used for this project
    bool extent_set;                            // not used for this project
    int m_width, m_height;
    ImageBase();
    void _selectROI();
    void _showImage();
public:
    int id;
    virtual void loadImage(const std::string &filepath, int flag);
    virtual void loadKinect(const std::string &filepath) = 0;
    virtual void loadPose(const std::string &filepath);
    virtual void setPose(const Eigen::Matrix<dtype, 3, 4> &pose);
    virtual void loadIntrinsic(const std::string &filepath);
    virtual void setIntrinsic(const Mat3 &intrinsic);
    virtual void showImage() = 0;
    virtual double getMaxDepth() const = 0;
    virtual void selectROI() = 0;                                   // not used for this project
    void saveROI(std::ofstream &fout, int idx);                     // not used for this project
    void loadROI(std::ifstream &fin, int idx);                      // not used for this project
    const cv::Mat& getImage() const;
    Eigen::Matrix<dtype, 3, 4> getPose() const;
    Mat3 getIntrinsic() const;
    Mat3 getRotation() const;
    Vec3 getTranslation() const;
    Vec2 topLeft() const;                                           // not used for this project
    Vec2 bottomRight() const;                                       // not used for this project
    int getWidth() const;
    int getHeight() const;
    /**
     * @brief Transform inertial coordinate system from cam0 to the one centered at the central position.
     * Use if only if necessary.
     * @param rotation Rotation matrix mapping coordinate in cam0 system into new inertia system.
     * @param translation Translation vector mapping coordinate in cam0 system into new inertia system.
     */
    void transformInertia(const Mat3 &rotation, const Vec3 &translation);
};

class DepthImage: public ImageBase {
    double _max_depth;
//    cv::Mat m_img_for_show;
    void _genDisplayImg(cv::Mat &dst);
    bool __max_inf;
public:
    DepthImage();
    void loadKinect(const std::string &filepath) override;
    void loadImage(const std::string &filepath);
    void showImage() override;
    void denoieseImage();
    double getMaxDepth() const override;
    void selectROI() override;                                  // not used for this project
};

class RGBImage: public ImageBase {
private:
    void loadKinect(const std::string &filepath) override;
    double getMaxDepth() const override;
public:
    RGBImage();
    void loadImage(const std::string &filepath, int flag = cv::IMREAD_ANYCOLOR);
    void showImage() override;
    void selectROI() override;                                  // not used for this project
};

inline int ImageBase::getWidth() const {
    return this->m_width;
}

inline int ImageBase::getHeight() const {
    return this->m_height;
}
#endif //SDF_2_SDF_CAMERA_H
