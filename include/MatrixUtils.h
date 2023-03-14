#ifndef MATRIXUTILS_H
#define MATRIXUTILS_H

#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

// using namespace cv;
using namespace std;
using namespace Eigen;

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> homo_to_real_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_homo_in);

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in);

// make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
template <class T>
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat);


template <class T>
Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw);

Eigen::Quaterniond zyx_euler_to_quat(const double &roll, const double &pitch, const double &yaw);

void rot_to_euler_zyx(const Eigen::Matrix3f&R, float &roll, float &pitch, float &yaw);


bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &all_strings);

void read_yaml(const std::string &path_to_yaml, Eigen::Matrix3d & Kalib, float& depth_scale);

void LoadFileName(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps);

float bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2);
float bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2);
float bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2);


bool check_inside_box(const Vector2d& pt, const Vector2d& box_left_top, const Vector2d& box_right_bottom);

// make sure edges start from left to right
void align_left_right_edges(MatrixXd& all_lines);

// merge short edges into long. edges n*4  each edge should start from left to right! 
void merge_break_lines( const MatrixXd& all_lines,          
                        MatrixXd& merge_lines_out,          
                        double pre_merge_dist_thre,         
		                double pre_merge_angle_thre_degree, 
                        double edge_length_threshold);

void atan2_vector(const VectorXd& y_vec, const VectorXd& x_vec, VectorXd& all_angles);
void fast_RemoveRow(Eigen::MatrixXd& matrix,int rowToRemove, int& total_line_number);



#endif //MATRIX_UTILS_H
