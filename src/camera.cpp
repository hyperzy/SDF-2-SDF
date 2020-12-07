//
// Created by himalaya on 12/6/20.
//

#include "camera.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <fstream>
#include <iostream>
using namespace std;
using namespace cv;

ImageBase::ImageBase():
        m_operator_img_ptr(nullptr), r_set(false), t_set(false), extent_set(false),
        i_set(false) {

}

struct CallbackParams {
    cv::Rect rect;
    cv::Mat dst;
    std::string window_name;
    const cv::Mat &src;
    CallbackParams(const cv::Mat &srcm_img): src(srcm_img) {}
};
/**
 * @brief OpenCV mouse event interrupt.
 * @param event Mouse event
 * @param x Horizontal image location
 * @param y Vertical image location
 * @param flags
 * @param params
 */
static void onMouse(int event, int x, int y, int flags, void *params) {
    CallbackParams *parameter = (CallbackParams *)params;
    Rect &rect = parameter->rect;
    const Mat &src = parameter->src;
    Mat &dst = parameter->dst;
    string &window_name = parameter->window_name;
    if (event == EVENT_LBUTTONDOWN)
    {
        parameter->rect = Rect(x, y , 0 , 0);
        src.copyTo(dst);
        circle(dst, Point(x, y), 2, Scalar(255, 0, 0), FILLED);
        imshow(window_name, dst);
    }
//    // renew rectangular
//    else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_LBUTTON))
//    {
//        src.copyTo(dst);
//        imshow(window_name,dst);
//    }
        // draw rect when mouse is moving
    else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))
    {
        src.copyTo(dst);
        rectangle(dst, Point(rect.x, rect.y), Point(x, y), Scalar(240, 0, 0), 2);
        imshow(window_name, dst);
    }
    else if (event == EVENT_LBUTTONUP)
    {
        src.copyTo(dst);
        rectangle(dst, Point(rect.x, rect.y), Point(x, y), Scalar(240, 0, 0), 2);
        rect.width = abs(x - rect.x);
        rect.height = abs(y - rect.y);
        rect.x = min(rect.x, x);
        rect.y = min(rect.y, y);
        imshow(window_name, dst);
    }
}

void ImageBase::_showImage() {
    if (!this->m_operator_img_ptr || !this->m_operator_img_ptr->data) {
        cerr << "Empty Img" << endl;
    }
    else {
        namedWindow("img");
        imshow("img", *(this->m_operator_img_ptr));
        waitKey(0);
        destroyAllWindows();
    }
}

void ImageBase::_selectROI() {
    if (!this->m_operator_img_ptr || !this->m_operator_img_ptr->data) {
        cerr << "Empty Img" << endl;
    }
    else {
        struct CallbackParams parameters(*(this->m_operator_img_ptr));
        parameters.window_name = "ROI Selection";
        namedWindow(parameters.window_name, 0);
        setMouseCallback(parameters.window_name, onMouse, (void *)&parameters);
        imshow(parameters.window_name, parameters.src);
        while (true) {
            // press Enter to finish
            if (waitKey(0) == 13) break;
        }
        destroyAllWindows();
        this->m_top_left << parameters.rect.x, parameters.rect.y;
        this->m_bottom_right << parameters.rect.x + parameters.rect.width, parameters.rect.y + parameters.rect.height;
        this->extent_set = true;
    }
}
void ImageBase::loadImage(const std::string &filepath, int flag) {
    try {
        this->m_img = imread(filepath, flag);
        this->m_width = this->m_img.cols;
        this->m_height = this->m_img.rows;
        if (!this->m_img.data) {
            throw runtime_error("Invalid Image Path.");
        }
    }catch (exception const &e) {
        cerr << "Exception: " << e.what() << endl;
    }
}

void ImageBase::loadPose(const std::string &filepath) {
    ifstream fs(filepath, ifstream::in);
    try {
        if (!fs.is_open()) {
            throw runtime_error("Invalid Parameter Path.");
        }
        dtype mat[16];
        for (auto &entry: mat) {
            fs >> entry;
        }
        fs.close();
        Eigen::Map<Eigen::Matrix<dtype, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> P(mat, 4, 4);
        /**** P.inverse() is pretty critical since the input pose is relative to Camera0 which transform coordinates
         *  of current camera system to Camera0 (which is world)
         */
        this->setPose(P.inverse().topRows(3));
//        Mat3 r = P.block<3, 3>(0, 0);
//        Vec3 c = P.block<3, 1>(0, 3);
//        Eigen::Matrix<dtype, 3, 4> pose;
//        this->m_rot_mat = r;
//        this->m_t_vec = -r * c;
//        pose.block<3, 3>(0, 0) = r;
//        pose.block<3, 1>(0, 3) = this->m_t_vec;
//        this->m_pose_mat = pose;
//        this->r_set = this->t_set = true;
//        cout << this->m_pose_mat << endl;
    }catch (exception const &e) {
        cout << "Exception: " << e.what() << endl;
    }
}

void ImageBase::setPose(const Eigen::Matrix<dtype, 3, 4> &pose) {
    this->m_pose_mat = pose;
    this->m_rot_mat = this->m_pose_mat.block<3, 3>(0, 0);
    this->m_t_vec = this->m_pose_mat.block<3, 1>(0, 3);
    this->r_set = this->t_set = true;
}

void ImageBase::loadIntrinsic(const std::string &filepath) {
    ifstream fs(filepath, ifstream::in);
    if (!fs.is_open()) {
        cerr << "Invalid Parameter Path." << endl;
    }
    else {
        dtype mat[9];
        for (auto &entry: mat) {
            fs >> entry;
        }
        fs.close();

        this->m_intrinsic_mat = Eigen::Map<Eigen::Matrix<dtype, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(mat, 3, 3);
        this->i_set = true;
    }
}

void ImageBase::setIntrinsic(const Mat3 &intrinsic) {
    this->m_intrinsic_mat = intrinsic;
    this->i_set = true;
}


void ImageBase::saveROI(std::ofstream &fout, int idx) {
    if (!fout.is_open()) {
        cerr << "Wrong file path." << endl;
    }
    else {
        fout << idx << endl;
        fout << this->m_top_left << endl;
        fout << this->m_bottom_right << endl;
        fout << endl;
    }
}

void ImageBase::loadROI(std::ifstream &fin, int idx) {
    // todo: id and idx are used for information validation. e.g. id != idx blabla
    int id, x1, y1, x2, y2;
    if (!fin.is_open()) {
        cerr << "Wrong file path." << endl;
    }
    else {
        fin >> id >> x1 >> y1 >> x2 >> y2;
        this->m_top_left << x1, y1;
        this->m_bottom_right << x2, y2;
        this->extent_set = true;
    }
}

const Mat& ImageBase::getImage() const {
    return this->m_img;
}

Eigen::Matrix<dtype, 3, 4> ImageBase::getPose() const {
    return this->m_pose_mat;
}


Mat3 ImageBase::getIntrinsic() const {
    if (!this->i_set) cerr << "Intrinsic matrix is not set." << endl;
    return this->m_intrinsic_mat;
}

Mat3 ImageBase::getRotation() const {
    if (!this->r_set) cerr << "Rotation Matrix Is Not Set." << endl;
    return this->m_rot_mat;
}

Vec3 ImageBase::getTranslation() const {
    if (!this->t_set) cerr << "Translation Vector Is Not Set." << endl;
    return this->m_t_vec;
}

Vec2 ImageBase::topLeft() const {
    return this->m_top_left;
}

Vec2 ImageBase::bottomRight() const {
    return this->m_bottom_right;
}

void ImageBase::transformInertia(const Mat3 &rotation, const Vec3 &translation) {
    // be careful about the execution order
    this->m_t_vec = this->m_rot_mat * translation + this->m_t_vec;
    this->m_rot_mat = this->m_rot_mat * rotation;
    Eigen::Matrix<dtype, 3, 4> pose;
    pose.block<3, 3>(0, 0) = this->m_rot_mat;
    pose.rightCols(1) = this->m_t_vec;
    this->m_pose_mat = pose;
}

DepthImage::DepthImage():_max_depth(0) {
}

void DepthImage::loadImage(const std::string &filepath) {
    this->__max_inf = false;
//    try {
//        this->m_img = imread(filepath, CV_LOAD_IMAGE_ANYDEPTH);
//        if (!this->m_img.data) {
//            throw runtime_error("Invalid Image Path.");
//        }
//        double min_v;
//        minMaxIdx(this->m_img, &min_v, &this->_max_depth);
//    }catch (exception const &e) {
//        cerr << "Exception: " << e.what() << endl;
//    }
    // load the image
    cv::Mat depth_map = cv::imread( filepath, -1 );
    if (!depth_map.data) cerr << "Invalid Image Path." << endl;
    double min_v;
    auto inf_notation = std::numeric_limits<double>::infinity();
    minMaxIdx(depth_map, &min_v, &this->_max_depth);
//    minMaxIdx(depth_map, &min_v, &this->_max_depth);
    if (this->_max_depth == inf_notation) {
        cv::cvtColor( depth_map, depth_map, COLOR_RGB2GRAY );
        this->__max_inf = true;
        Mat mask = depth_map == inf_notation;
        depth_map.setTo(INF, mask);
    }
    // convert to meters
    depth_map.convertTo( depth_map, DTYPEC1, 0.001 );
    minMaxIdx(depth_map, &min_v, &this->_max_depth);
    this->m_img = depth_map.clone();
    this->m_width = this->m_img.cols;
    this->m_height = this->m_img.rows;
}

void DepthImage::loadKinect(const std::string &filepath) {
    this->__max_inf = false;
    try {
        this->m_img = imread(filepath, cv::IMREAD_ANYDEPTH);
        if (!this->m_img.data) {
            throw runtime_error("Invalid Image Path.");
        }
        double min_v;
        minMaxIdx(this->m_img, &min_v, &this->_max_depth);
    }catch (exception const &e) {
        cerr << "Exception: " << e.what() << endl;
    }
    this->m_width = this->m_img.cols;
    this->m_height = this->m_img.rows;
}

void DepthImage::_genDisplayImg(cv::Mat &dst) {
    if (!this->m_img.data) {
        cerr << "Empty Depth Image." << endl;
    }
    else {
        convertScaleAbs(this->m_img, dst, 255 / this->_max_depth);
    }
}

void DepthImage::showImage() {
    Mat img_for_show;
    if (this->__max_inf) {
        Mat mask = (this->m_img == this->_max_depth);
        img_for_show = this->m_img.clone();
        img_for_show.setTo(std::numeric_limits<double>::infinity(), mask);
//        this->m_operator_img_ptr = &(this->m_img);
        this->m_operator_img_ptr = &img_for_show;
    }
    else {
        this->_genDisplayImg(img_for_show);
        this->m_operator_img_ptr = &img_for_show;
    }
    this->_showImage();
    this->m_operator_img_ptr = nullptr;
}

void DepthImage::denoieseImage() {
    ximgproc::anisotropicDiffusion(this->m_img, this->m_img, 0.2, 1.1, 50);
}

double DepthImage::getMaxDepth() const {
    return this->_max_depth;
}

void DepthImage::selectROI() {
    Mat img_for_show;
    if (isinf(this->_max_depth)) {
        this->m_operator_img_ptr = &(this->m_img);
    }
    else {
        this->_genDisplayImg(img_for_show);
        this->m_operator_img_ptr = &img_for_show;
    }
    this->_selectROI();
    this->m_operator_img_ptr = nullptr;
}

RGBImage::RGBImage() {
}

void RGBImage::loadImage(const std::string &filepath, int flag) {
    try {
        this->m_img = imread(filepath, flag);
        this->m_width = this->m_img.cols;
        this->m_height = this->m_img.rows;
        if (!this->m_img.data) {
            throw runtime_error("Invalid Image Path.");
        }
    }catch (exception const &e) {
        cerr << "Exception: " << e.what() << endl;
        exit(EXIT_FAILURE);
    }
}

void RGBImage::loadKinect(const std::string &filepath) {
    cerr << "Cannot load depth image for RGB image object" << endl;
    exit(EXIT_FAILURE);
}

void RGBImage::showImage() {
    this->m_operator_img_ptr = &(this->m_img);
    this->_showImage();
    this->m_operator_img_ptr = nullptr;
}

double RGBImage::getMaxDepth() const {
    cerr << "No depth for RGB image." << endl;
    exit(EXIT_FAILURE);
}

void RGBImage::selectROI() {
    this->m_operator_img_ptr = &(this->m_img);
    this->_selectROI();
    this->m_operator_img_ptr = nullptr;
}
