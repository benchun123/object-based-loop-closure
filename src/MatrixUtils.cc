#include <MatrixUtils.h>

// using namespace cv;
using namespace std;
using namespace Eigen;

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pts_homo_out;
    int raw_rows = pts_in.rows();
    int raw_cols = pts_in.cols();

    pts_homo_out.resize(raw_rows + 1, raw_cols);
    pts_homo_out << pts_in,
        Matrix<T, 1, Dynamic>::Ones(raw_cols);
    return pts_homo_out;
}
template MatrixXd real_to_homo_coord<double>(const MatrixXd &);
template MatrixXf real_to_homo_coord<float>(const MatrixXf &);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> homo_to_real_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_homo_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> pt_out;
    if (pts_homo_in.rows() == 4)
        pt_out = pts_homo_in.head(3) / pts_homo_in(3);
    else if (pts_homo_in.rows() == 3)
        pt_out = pts_homo_in.head(2) / pts_homo_in(2);

    return pt_out;
}
template VectorXd homo_to_real_coord_vec<double>(const VectorXd &);
template VectorXf homo_to_real_coord_vec<float>(const VectorXf &);


template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pts_out(pts_homo_in.rows() - 1, pts_homo_in.cols());
    for (int i = 0; i < pts_homo_in.rows() - 1; i++)
        pts_out.row(i) = pts_homo_in.row(i).array() / pts_homo_in.bottomRows(1).array(); //replicate needs actual number, cannot be M or N

    return pts_out;
}
template MatrixXd homo_to_real_coord<double>(const MatrixXd &);
template MatrixXf homo_to_real_coord<float>(const MatrixXf &);


// make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
template <class T>
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat)
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter = 0;
    std::string line;
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);

    while (getline(filetxt, line))
    {
        T t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();

    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows

    return true;
}
template bool read_all_number_txt(const std::string, MatrixXd &);
template bool read_all_number_txt(const std::string, MatrixXi &);



template <class T>
Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw)
{
    T cp = cos(pitch);
    T sp = sin(pitch);
    T sr = sin(roll);
    T cr = cos(roll);
    T sy = sin(yaw);
    T cy = cos(yaw);

    Eigen::Matrix<T, 3, 3> R;
    R << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    return R;
}
template Matrix3d euler_zyx_to_rot<double>(const double &, const double &, const double &);
template Matrix3f euler_zyx_to_rot<float>(const float &, const float &, const float &);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw)
{
    T qw = q.w();
    T qx = q.x();
    T qy = q.y();
    T qz = q.z();

    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}
template void quat_to_euler_zyx<double>(const Eigen::Quaterniond &, double &, double &, double &);
template void quat_to_euler_zyx<float>(const Eigen::Quaternionf &, float &, float &, float &);

Eigen::Quaterniond zyx_euler_to_quat(const double &roll, const double &pitch, const double &yaw)
{
      double sy = sin(yaw*0.5);
      double cy = cos(yaw*0.5);
      double sp = sin(pitch*0.5);
      double cp = cos(pitch*0.5);
      double sr = sin(roll*0.5);
      double cr = cos(roll*0.5);
      double w = cr*cp*cy + sr*sp*sy;
      double x = sr*cp*cy - cr*sp*sy;
      double y = cr*sp*cy + sr*cp*sy;
      double z = cr*cp*sy - sr*sp*cy;
      return Eigen::Quaterniond(w,x,y,z);
}

void rot_to_euler_zyx(const Eigen::Matrix3f&R, float &roll, float &pitch, float &yaw)
{
    pitch = asin(-R(2, 0));

    if (abs(pitch - M_PI / 2) < 1.0e-3)
    {
        roll = 0.0;
        yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) + roll;
    }
    else if (abs(pitch + M_PI / 2) < 1.0e-3)
    {
        roll = 0.0;
        yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) - roll;
    }
    else
    {
        roll = atan2(R(2, 1), R(2, 2));
        yaw = atan2(R(1, 0), R(0, 0));
    }
}

bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &all_strings)
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    all_strings.clear();
    std::ifstream filetxt(txt_file_name.c_str());
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);
    int row_counter = 0;
    std::string line;
    while (getline(filetxt, line))
    {
        double t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            std::string classname;
            ss >> classname;
            all_strings.push_back(classname);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();
    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows
    return true;
}

void read_yaml(const std::string &path_to_yaml, Eigen::Matrix3d & Kalib, float& depth_scale)
{
    // string strSettingPath = path_to_dataset + "/ICL.yaml";
    cv::FileStorage fSettings(path_to_yaml, cv::FileStorage::READ);

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Load camera parameters from settings file
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    Kalib<< fx,  0,  cx,
            0,  fy,  cy,
            0,  0,   1;
	depth_scale = fSettings["DepthMapFactor"];
}

void LoadFileName(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    // std::cout << "strFile: " << strFile << std::endl;
    ifstream f;
    f.open(strFile.c_str());
    string s0;
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

float bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2)
{
    // using opencv with cv::Rect
    int overlap_area = (rect1 & rect2).area();
    return (float)overlap_area / ((float)(rect1.area() + rect2.area() - overlap_area));
}

float bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()));
}

float bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect2.area()));
}

bool check_inside_box(const Vector2d& pt, const Vector2d& box_left_top, const Vector2d& box_right_bottom)
{
    return box_left_top(0)<=pt(0) && pt(0)<=box_right_bottom(0) && box_left_top(1)<=pt(1) && pt(1)<=box_right_bottom(1);
}

// make sure edges start from left to right
void align_left_right_edges(MatrixXd& all_lines)
{
    for (int line_id=0; line_id < all_lines.rows(); line_id++)
    {
        if (all_lines(line_id,2) < all_lines(line_id,0))
        {
            Vector2d temp = all_lines.row(line_id).tail<2>();
            all_lines.row(line_id).tail<2>() = all_lines.row(line_id).head<2>();
            all_lines.row(line_id).head<2>() = temp;
        }
    }
}

// merge short edges into long. edges n*4  each edge should start from left to right! 
void merge_break_lines( const MatrixXd& all_lines,          
                        MatrixXd& merge_lines_out,          
                        double pre_merge_dist_thre,         
		                double pre_merge_angle_thre_degree, 
                        double edge_length_threshold)       
{
    bool can_force_merge = true;
    merge_lines_out = all_lines;
    int total_line_number = merge_lines_out.rows();  // line_number will become smaller and smaller, merge_lines_out doesn't change
    int counter = 0;
    double pre_merge_angle_thre = pre_merge_angle_thre_degree/180.0*M_PI;

    while ((can_force_merge) && (counter<500))
    {
	    counter++;
	    can_force_merge=false;
	    MatrixXd line_vector = merge_lines_out.topRightCorner(total_line_number,2) - merge_lines_out.topLeftCorner(total_line_number,2);

	    VectorXd all_angles; 
        atan2_vector(line_vector.col(1),line_vector.col(0),all_angles); // don't need normalize_to_pi, because my edges is from left to right, always [-90 90]
	    
        for (int seg1 = 0;seg1 < total_line_number - 1; seg1++) 
        {
		    for (int seg2 = seg1+1; seg2 < total_line_number; seg2++)
            {
                double diff = std::abs(all_angles(seg1) - all_angles(seg2));
                double angle_diff = std::min(diff, M_PI - diff);

                if (angle_diff < pre_merge_angle_thre)
                {
                    double dist_1ed_to_2 = (merge_lines_out.row(seg1).tail(2) - merge_lines_out.row(seg2).head(2)).norm();
                    double dist_2ed_to_1 = (merge_lines_out.row(seg2).tail(2) - merge_lines_out.row(seg1).head(2)).norm();

                    if ((dist_1ed_to_2 < pre_merge_dist_thre) || (dist_2ed_to_1 < pre_merge_dist_thre))
                    {
                        Vector2d merge_start, merge_end;
                        if (merge_lines_out(seg1,0) < merge_lines_out(seg2,0))
                            merge_start = merge_lines_out.row(seg1).head(2);
                        else
                            merge_start = merge_lines_out.row(seg2).head(2);
                        if (merge_lines_out(seg1,2) > merge_lines_out(seg2,2))
                            merge_end = merge_lines_out.row(seg1).tail(2);
                        else
                            merge_end = merge_lines_out.row(seg2).tail(2);
                        
                        double merged_angle = std::atan2(merge_end(1)-merge_start(1),merge_end(0)-merge_start(0));
                        
                        double temp = std::abs(all_angles(seg1) - merged_angle);
                        double merge_angle_diff = std::min( temp, M_PI-temp );
                        
                        if (merge_angle_diff < pre_merge_angle_thre)
                        {
                            merge_lines_out.row(seg1).head(2) = merge_start;
                            merge_lines_out.row(seg1).tail(2) = merge_end;
                            fast_RemoveRow(merge_lines_out, seg2, total_line_number);  //also decrease  total_line_number
                            can_force_merge = true;
                            break;
                        }
                    }
                }
		    }
            if (can_force_merge)
                break;			
	    }
    }

    if (edge_length_threshold > 0)
    {
        MatrixXd line_vectors = merge_lines_out.topRightCorner(total_line_number,2) - merge_lines_out.topLeftCorner(total_line_number,2);
        VectorXd line_lengths = line_vectors.rowwise().norm();

        int long_line_number = 0;
        MatrixXd long_merge_lines(total_line_number, 4);
        for (int i = 0; i < total_line_number; i++)
        {
            if (line_lengths(i) > edge_length_threshold)
            {
                long_merge_lines.row(long_line_number) = merge_lines_out.row(i);
                long_line_number++;
            }
        }
        merge_lines_out = long_merge_lines.topRows(long_line_number);
    }
    else
	    merge_lines_out.conservativeResize(total_line_number,NoChange);
}

void atan2_vector(const VectorXd& y_vec, const VectorXd& x_vec, VectorXd& all_angles)
{
    all_angles.resize(y_vec.rows());
    for (int i=0;i<y_vec.rows();i++)
	    all_angles(i)=std::atan2(y_vec(i),x_vec(i));  // don't need normalize_to_pi, because my edges is from left to right, always [-90 90]
}

void fast_RemoveRow(MatrixXd& matrix,int rowToRemove, int& total_line_number)
{
    matrix.row(rowToRemove) = matrix.row(total_line_number-1);
    total_line_number--;
}