#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace Eigen;

// make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat)
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
        double t;
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

void DistanceMatrix(vector<vector<float> > &centerpoint, Eigen::MatrixXi& label_matrix, Eigen::MatrixXf& distance_matrix)
{

    int num = centerpoint.size();
    distance_matrix.resize(num, num);
    label_matrix.resize(num, num);
    for (size_t i = 0; i < centerpoint.size(); i++)
    {
        Eigen::VectorXi node_vec_label(num);
        Eigen::VectorXf node_vec_dist(num);
        float x0 = centerpoint[i][0];
        float y0 = centerpoint[i][1];
        float z0 = centerpoint[i][2];
        for (size_t j = 0; j < centerpoint.size(); j++)
        {
            float x1 = centerpoint[j][0];
            float y1 = centerpoint[j][1];
            float z1 = centerpoint[j][2];
            // node_vec_dist(j) = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
            node_vec_dist(j) = sqrt(pow(x0-x1,2)+ pow(y0-y1,2));
            // node_vec_label(j) = j;
            node_vec_label(j) = centerpoint[j][3];
        }
        // std::cout << "node_vec_dist: " << node_vec_dist.transpose() << std::endl;
        // sort the vector by distance
        for (size_t m = 0; m < node_vec_dist.size(); m++)
            for (size_t n = m+1; n < node_vec_dist.size(); n++)
        {
            if(node_vec_dist(m) > node_vec_dist(n))
            {
                float dis_tmp = node_vec_dist(m);
                node_vec_dist(m) = node_vec_dist(n);
                node_vec_dist(n) = dis_tmp;
                int label_tmp = node_vec_label(m);
                node_vec_label(m) = node_vec_label(n);
                node_vec_label(n) = label_tmp;
            }
        }
        distance_matrix.row(i) = node_vec_dist;
        label_matrix.row(i) = node_vec_label;
        // std::cout << "node_vec_label: " << node_vec_label.transpose() << std::endl;
    }
}

void OrientHistDes(vector<vector<float> > &centerpoint, Eigen::MatrixXi& orient_matrix)
{
    int num = centerpoint.size();
    orient_matrix.resize(num, 48);
    for (size_t i = 0; i < centerpoint.size(); i++)
    {
        Eigen::VectorXi hist_vec(48);
        hist_vec.setZero();
        Eigen::VectorXi node_vec_label(num);
        Eigen::VectorXf node_vec_orient(num);
        float x0 = centerpoint[i][0];
        float y0 = centerpoint[i][1];
        float z0 = centerpoint[i][2];
        for (size_t j = 0; j < centerpoint.size(); j++)
        {
            float x1 = centerpoint[j][0];
            float y1 = centerpoint[j][1];
            float z1 = centerpoint[j][2];
            // node_vec_dist(j) = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
            float orient_diff = atan2(y1-y0,x1-x0); // atan2 = gives angle value between -pi and pi
            int idx = centerpoint[j][3];
            // node_vec_label(j) = j;
            node_vec_label(j) = centerpoint[j][3];

            // // put into matrix
            // if(orient_diff >= -M_PI && orient_diff < -M_PI/2)
            //     hist_vec(3*6+idx) += 1;
            // if(orient_diff >= -M_PI/2 && orient_diff < 0)
            //     hist_vec(2*6+idx) += 1;
            // if(orient_diff >= 0 && orient_diff < M_PI/2)
            //     hist_vec(1*6+idx) += 1;
            // if(orient_diff >= M_PI/2 && orient_diff < M_PI)
            //     hist_vec(0*6+idx) += 1;

            int div=8;
            for (size_t k = 0; k < div; k++)
            {
                if(orient_diff >= -M_PI+k*2*M_PI/div && orient_diff < -M_PI+(k+1)*2*M_PI/div)
                    hist_vec(k*6+idx) += 1;
            }

        }
        // std::cout << "node_vec_dist: " << node_vec_dist.transpose() << std::endl;
        // sort the vector by distance
        // for (size_t m = 0; m < node_vec_dist.size(); m++)
        //     for (size_t n = m+1; n < node_vec_dist.size(); n++)
        // {
        //     if(node_vec_dist(m) > node_vec_dist(n))
        //     {
        //         float dis_tmp = node_vec_dist(m);
        //         node_vec_dist(m) = node_vec_dist(n);
        //         node_vec_dist(n) = dis_tmp;
        //         int label_tmp = node_vec_label(m);
        //         node_vec_label(m) = node_vec_label(n);
        //         node_vec_label(n) = label_tmp;
        //     }
        // }
        // distance_matrix.row(i) = node_vec_dist;
        // label_matrix.row(i) = node_vec_label;
        // std::cout << "node_vec_label: " << node_vec_label.transpose() << std::endl;
        orient_matrix.row(i) = hist_vec;
        // std::cout << "hist_vec: " << hist_vec.transpose() << std::endl;
    }

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

void show_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& insertCloud)
{
    //cout<<" has: "<<temp->points.size()<<" points"<<endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(insertCloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(insertCloud, rgb, "Point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "Point cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(1000);
    }
}

void pointVisuallize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud2,MatrixXf R, MatrixXf T)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZRGB>);
    //obtain the clouds.
    Matrix4f TransforMatrix = Matrix4f::Identity();
    for(int row = 0; row<3; row++){
        for(int col=0; col<3; col++){
            TransforMatrix(row, col) = R(row, col);
        }
    }
    TransforMatrix(0, 3) = T(0, 0);
    TransforMatrix(1, 3) = T(1, 0);
    TransforMatrix(2, 3) = T(2, 0);

    // TransforMatrix(2, 3) = TransforMatrix(2, 3)+80;  
    *temp1 = *insertCloud1;
    *temp2 = *insertCloud2;
    // transformPointCloud (*temp1, *temp1, TransforMatrix);
    transformPointCloud (*temp2, *temp2, TransforMatrix);
    *temp = *temp1 + *temp2;
    show_point_cloud(temp);
}

Eigen::MatrixXd compute3D_BoxCorner(Eigen::Vector3d& location, Eigen::Vector3d& dimension, double& ry)
{
    MatrixXd corners_body(3, 8);
    corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
                    1, -1, -1, 1, 1, -1, -1, 1,
                    1, 1, 1, 1, -1, -1, -1, -1;
    Matrix3d scale_mat = dimension.asDiagonal();
    Matrix3d rot;
    rot << cos(ry), -sin(ry), 0,
        sin(ry), cos(ry), 0,
        0, 0, 1;                          // rotation around z (up), world coordinate

    MatrixXd corners_without_center = rot * scale_mat * corners_body;
    MatrixXd corners_3d(3, 8);
    for (size_t i = 0; i < 8; i++)
    {
      corners_3d(0,i) = corners_without_center(0,i) + location(0);
      corners_3d(1,i) = corners_without_center(1,i) + location(1);
      corners_3d(2,i) = corners_without_center(2,i) + location(2);
    }
    return corners_3d;
}

int main(int argc, const char * argv[])
{
    if(argc!=2)
    {
        cout << "usage: ./ustc_test ~/dataset/multi_robot_slam/" << endl;
        return 1;
    }

    string dir1 = argv[1];
    string dir2 = argv[1];

    Eigen::MatrixXf offline_mapobjects(1,9); // obj_class, xyz, yaw, whl, id
    Eigen::MatrixXf offline_mappoints(1,8); // obj_class, xyz, pt_class, obj_idx, observe_id, pt_id
    Eigen::MatrixXf offline_mapkeyframe(1,8); // keyframe_id, xyz
    if(read_all_number_txt(dir1+"/MapObjects.txt", offline_mapobjects)==false)
        return -1;
    if(read_all_number_txt(dir1+"/MapPoints.txt", offline_mappoints)==false)
        return -1;
    if(read_all_number_txt(dir2+"/MapKeyframe.txt", offline_mapkeyframe)==false)
        return -1;
    // std::cout << offline_mappoints << std::endl;

    Eigen::MatrixXi label_converter (6,2);
    label_converter << 56, 0, 60, 1, 62, 2, 64, 3, 57, 4, 66, 5;

    //generate the correspondent rgb value of all labels in the segmentation image
    Eigen::MatrixXi color (11,3); // point cloud color
    color << 175, 6, 140,   65, 54, 217,   156, 198, 23,  184, 145, 182,
            211, 80, 208,  232, 250, 80,  234, 20, 250,  99, 242, 104,
            142, 1, 246,   81, 13, 36,    112, 105, 191;

    // offline data visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZRGB>);
    // add map points
    for (size_t i = 0; i < offline_mappoints.rows(); i++)
    {
        int obj_idx = (int)offline_mappoints(i, 5);
        int obj_class = (int)offline_mappoints(i, 0);
        pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
        pt_tmp.x = offline_mappoints(i,1);
        pt_tmp.y = offline_mappoints(i,2);
        pt_tmp.z = offline_mappoints(i,3);
        pt_tmp.r = color(obj_class%10, 0);
        pt_tmp.g = color(obj_class%10, 1);
        pt_tmp.b = color(obj_class%10, 2);
        cloud_raw->points.push_back(pt_tmp);
    }
    // add keyframe
    for (size_t i = 0; i < offline_mapkeyframe.rows(); i++)
    {
        pcl::PointXYZRGB pt_tmp; // id x,y,z
        pt_tmp.x = offline_mapkeyframe(i,1);
        pt_tmp.y = offline_mapkeyframe(i,2);
        pt_tmp.z = offline_mapkeyframe(i,3);
        pt_tmp.r = 0.0f;
        pt_tmp.g = 0.0f;
        pt_tmp.b = 255.0f;
        cloud_raw->points.push_back(pt_tmp);
    }
    // visualization
    bool show_orgin_point = true;
    if(show_orgin_point)
        show_point_cloud(cloud_raw);


    // visualization map object
    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("viewer");
    viewer->setBackgroundColor(255, 255, 255);
    viewer->addCoordinateSystem(1);
    pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgb(cloud_raw);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_raw, rgb, "show cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "show cloud");
    for (size_t obj_id = 0; obj_id < offline_mapobjects.rows(); obj_id++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr CuboidCornerCloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
        Eigen::Vector3d raw_centroid = offline_mapobjects.row(obj_id).segment(1,3).cast<double>();
        double yaw = double(offline_mapobjects(obj_id, 4));
        Eigen::Vector3d raw_dimension = offline_mapobjects.row(obj_id).segment(5,3).cast<double>();
        // std::cout << "raw_2d_objs: " << raw_2d_objs.transpose() << std::endl;
        // std::cout << "raw_centroid: " << raw_centroid.transpose() << std::endl;
        // std::cout << "raw_dimension: " << raw_dimension.transpose() << std::endl;
        // std::cout << "yaw: " << yaw << std::endl;
        Eigen::MatrixXd corners_3d = compute3D_BoxCorner(raw_centroid, raw_dimension, yaw);
        // std::cout << "corners_3d \n " << corners_3d << std::endl;
        for (int n=0; n<corners_3d.cols(); n++ )
        {
            pcl::PointXYZRGB p;
            p.x = corners_3d(0,n);
            p.y = corners_3d(1,n);
            p.z = corners_3d(2,n);
            p.r = color((int)offline_mapobjects(obj_id,0)%10,0);
            p.g = color((int)offline_mapobjects(obj_id,0)%10,1);
            p.b = color((int)offline_mapobjects(obj_id,0)%10,2);
            // std::cout << "p.x " << p.x <<  " p.x " << p.y <<  " p.z " << p.x << std::endl;
            CuboidCornerCloud->points.push_back(p);
        }

        Eigen::MatrixXi line_list(2,12); // point order of line
        line_list << 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3,
                    1, 2, 3, 0, 5, 6, 7, 4, 4, 5, 6, 7;
        std::string cuboid_corner_name = "cuboid_corner_"+std::to_string(obj_id);
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> corner_show(CuboidCornerCloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (CuboidCornerCloud, corner_show, cuboid_corner_name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cuboid_corner_name);
        // show edges
        for (size_t line_id = 0; line_id < line_list.cols(); line_id++)
        {
            std::string line_name = "cuboid_"+std::to_string(obj_id)+"_line_"+std::to_string(line_id);
            int sta_id = line_list(0, line_id);
            int end_id = line_list(1, line_id);
            viewer->addLine<pcl::PointXYZRGB> (CuboidCornerCloud->points[sta_id], CuboidCornerCloud->points[end_id], line_name);
        }
    }
    if(show_orgin_point)
    {
        while(!viewer->wasStopped())
        {
            viewer->spinOnce(1000);
        }            
    }

/*
    Eigen::MatrixXd final_matches_eigen(4,2);
    final_matches_eigen << 0,10, 1,9, 2,11, 3,8;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_objects( new pcl::PointCloud<pcl::PointXYZRGB>() );
    for (size_t obj_id = 0; obj_id < offline_mapobjects.rows(); obj_id++)
    {
        pcl::PointXYZRGB p;
        p.x = offline_mapobjects(obj_id,1);
        p.y = offline_mapobjects(obj_id,2);
        p.z = offline_mapobjects(obj_id,3);
        p.r = color((int)offline_mapobjects(obj_id,0)%10,0);
        p.g = color((int)offline_mapobjects(obj_id,0)%10,1);
        p.b = color((int)offline_mapobjects(obj_id,0)%10,2);
        matched_objects->points.push_back(p);
    }
    for (size_t line_id = 0; line_id < final_matches_eigen.rows(); line_id++)
    {
        std::string line_name = "match_"+std::to_string(line_id);
        int sta_id = final_matches_eigen(line_id, 0);
        int end_id = final_matches_eigen(line_id, 1) + offline_mapobjects.rows()/2;
        viewer->addLine<pcl::PointXYZRGB> (matched_objects->points[sta_id], matched_objects->points[end_id], line_name);
    }
    // for (size_t m = 0; m < final_matches_eigen.rows(); m++)
    //     for (size_t n = m+1; n < final_matches_eigen.rows(); n++)
    // {
    //     std::string line_name_1 = "match_"+std::to_string(m)+std::to_string(n)+"_1";
    //     std::string line_name_2 = "match_"+std::to_string(m)+std::to_string(n)+"_2";
    //     int sta_id_1 = final_matches_eigen(m, 0);
    //     int end_id_1 = final_matches_eigen(n, 0);
    //     int sta_id_2 = final_matches_eigen(m, 1) + offline_mapobjects.rows()/2;
    //     int end_id_2 = final_matches_eigen(n, 1) + offline_mapobjects.rows()/2;
    //     // std::cout << "sta_id_1 " << sta_id_1 << " end_id_1 " << end_id_1 << std::endl;
    //     // std::cout << "sta_id_2 " << sta_id_2 << " end_id_2 " << end_id_2 << std::endl;
    //     viewer->addLine<pcl::PointXYZRGB> (matched_objects->points[sta_id_1], matched_objects->points[end_id_1], 0.0f,1.0f,0.0f, line_name_1);
    //     viewer->addLine<pcl::PointXYZRGB> (matched_objects->points[sta_id_2], matched_objects->points[end_id_2], 0.0f,1.0f,0.0f, line_name_2);
    // }
    if(show_orgin_point)
    {
        while(!viewer->wasStopped())
        {
            viewer->spinOnce(1000);
        }            
    }

*/

    vector<vector<float> > object_center; // obj_idx, xyz+class+id
    vector<vector<vector<float> > > object_asso_points; // obj_idx, associate_pt, pt_xyz
    for (size_t obj_id = 0; obj_id < offline_mapobjects.rows(); obj_id++)
    {
        int obj_idx = offline_mapobjects(obj_id, 8);
        float class_name = offline_mapobjects(obj_id, 0);
        for (size_t k = 0; k < label_converter.rows(); k++)
        {
            if (offline_mapobjects(obj_id, 0) == label_converter(k,0))
                class_name = label_converter(k,1);
        }
        float x_c = offline_mapobjects(obj_id, 1);
        float y_c = offline_mapobjects(obj_id, 2);
        float z_c = offline_mapobjects(obj_id, 3);
        std::vector<float> center_tmp = {x_c, y_c, z_c, class_name, float(obj_idx)};
        object_center.push_back(center_tmp);

        vector<vector<float> > asso_point_tmp;
        for (size_t pt_id = 0; pt_id < offline_mappoints.rows(); pt_id++)
        {
            if((int)offline_mappoints(pt_id, 5) == obj_idx )
            {
                std::vector<float> v = {offline_mappoints(pt_id,1), offline_mappoints(pt_id,2), offline_mappoints(pt_id,3), class_name, float(obj_idx)};
                asso_point_tmp.push_back(v);
            }
        }
        object_asso_points.push_back(asso_point_tmp);
    }

    for (size_t i = 0; i < object_center.size(); i++)
        std::cout << "object_center: " << object_center[i][0] << " " << object_center[i][1] << " " << object_center[i][2] 
                << " " << object_center[i][3] << " " << object_center[i][4] << std::endl;
    
    if(object_center.size()<6)
    {
        std::cout << " no object loop because object size < 6 "  << std::endl;
        return -1;
    }



    // start graph matching
    std::cout << " ----------- start detect loop ----------- "  << std::endl;
    // separate map object into two group
    vector<vector<float> > centerpoint1;
    vector<vector<float> > centerpoint2;
    int cpoint1_num = object_center.size()/2;
    for (size_t i = 0; i < cpoint1_num; i++)
    {
        vector<float> point_tmp; // x,y,z,label
        point_tmp.resize(5);
        point_tmp[0] = object_center[i][0];
        point_tmp[1] = object_center[i][1];
        point_tmp[2] = object_center[i][2];
        point_tmp[3] = object_center[i][3]; // label
        point_tmp[4] = object_center[i][4]; // id
        centerpoint1.push_back(point_tmp);
    }
    for (size_t j = cpoint1_num; j < object_center.size(); j++)
    {
        vector<float> point_tmp; // x,y,z,label
        point_tmp.resize(5);
        point_tmp[0] = object_center[j][0];
        point_tmp[1] = object_center[j][1];
        point_tmp[2] = object_center[j][2];
        point_tmp[3] = object_center[j][3]; // label
        point_tmp[4] = object_center[j][4]; // id
        centerpoint2.push_back(point_tmp);
    }


    // check loop detection
    // step 2: construct orient histogram descriptor
    Eigen::MatrixXi orient_mat_1, orient_mat_2;
    OrientHistDes(centerpoint1, orient_mat_1);
    OrientHistDes(centerpoint2, orient_mat_2);
    // std::cout << "orient_mat_1: \n" << orient_mat_1 << std::endl;
    // std::cout << "orient_mat_2: \n" << orient_mat_2 << std::endl;


    // step 3: get a coarse matches
    std::vector<Eigen::Vector3f> coarse_matches; // id1, id2, similarity_score
    // return coarse_matches;
    // FindCoarseMatches(centerpoint1, centerpoint2, orient_mat_1, orient_mat_2);
    int Num1 = orient_mat_1.rows();
    int Num2 = orient_mat_2.rows();
    for (size_t i = 0; i < Num1; i++)
    {
        float score = 0;
        int label_1 = centerpoint1[i][3];
        int idx_1 = centerpoint1[i][4];
        Eigen::VectorXi vec1 = orient_mat_1.row(i);
        // std::cout << "vec 1: " << vec1.transpose() << std::endl;
        for (size_t j = 0; j < Num2; j++)
        {
            int label_2 = centerpoint2[j][3];
            int idx_2 = centerpoint2[j][4];
            Eigen::VectorXi vec2 = orient_mat_2.row(j);
            // if(label_1 != label_2 ) // check if label are the same and over 4 different id
            if(label_1 != label_2 || idx_2-idx_1 < 20) // check if label are the same and over 4 different id
            {
                // std::cout << "label_1 " << label_1 << " label_2 " << label_2 << std::endl;
                // std::cout << "idx_1 " << idx_1 << " idx_2 " << idx_2 << std::endl;
                score=0;
                continue;
            }

            // calculate the similarity by dot production: s=sum(A*B)/sqrt(sum(A^2)*sum(B^2))
            float SumTop = 0;
            for(int row = 0; row<vec1.size(); row++)
            {
                SumTop = SumTop + (vec1(row)*vec2(row));
            }
            float SumBottomLeft = 0;
            float SumBottomRight = 0;
            for(int row = 0; row<vec1.size(); row++)
            {
                SumBottomLeft = SumBottomLeft + pow(vec1(row), 2);
                SumBottomRight = SumBottomRight + pow(vec2(row), 2);
            }
            float SumBottom = sqrt(SumBottomLeft*SumBottomRight);
            if(SumBottom==0)
                score = 0;
            else
                score = abs(SumTop/SumBottom);

            Eigen::Vector3f score_mat(i,j,score);
            coarse_matches.push_back(score_mat);
            cout<< "score_mat(" << i << "," << j << "): "<< score <<endl;
        }
    }

    // checkpoint 2: coase match size()
    if(coarse_matches.size()<3)
    {
        std::cout << "coarse_matches < 3, no loop closure" << std::endl;
        return -1;
    }

    int topk = coarse_matches.size();
    if(topk > 8)
        topk = 8;
    float min_similarity_thre = 0.3; // min_similarity score
    std::vector<Eigen::Vector3f> coarse_matches_topk;
    // FindTopKMatch(coarse_matches, topk, min_similarity_thre);
    for (size_t i = 0; i < coarse_matches.size(); i++)
        for (size_t j = i+1; j < coarse_matches.size(); j++)
    {
        if(coarse_matches[i](2) < coarse_matches[j](2))
        {
            Eigen::VectorXf tmp = coarse_matches[i];
            coarse_matches[i] = coarse_matches[j];
            coarse_matches[j] = tmp;
        }
    }
    // all matches have high similarity and at most 8 matches.
    for (size_t i = 0; i < topk; i++)
    {
        if(coarse_matches[i](2)>min_similarity_thre)
        {
            coarse_matches_topk.push_back(coarse_matches[i]);
            std::cout << "top_k_coarse_match: " << coarse_matches[i].transpose() << std::endl;
        }
        else
            break;
    }


    // step 4: geometry verification
    // step 4.1: check the scale of corresponding distance are similar
    std::vector<Eigen::VectorXf> geometry_matches; // match_id(2*3), error, scale 
    // FindGeometryMatches(centerpoint1, centerpoint2, coarse_matches_topk)
    size_t coarse_match_cnt = coarse_matches_topk.size();
    for (size_t i = 0; i < coarse_match_cnt; i++)
        for (size_t j = i+1; j < coarse_match_cnt; j++)
            for (size_t k = j+1; k < coarse_match_cnt; k++)
    {
        // check point id not duplicated
        Eigen::Vector3f match_i = coarse_matches_topk[i];
        Eigen::Vector3f match_j = coarse_matches_topk[j];
        Eigen::Vector3f match_k = coarse_matches_topk[k];
        if(match_i(0) == match_j(0) || match_i(1) == match_j(1) ||
        match_j(0) == match_k(0) || match_j(1) == match_k(1) ||
        match_i(0) == match_k(0) || match_i(1) == match_k(1) )
        {
            // std::cout << "wrong loop" << std::endl;
            continue;
        }
        // compute 2D distance and compare the scale
        float x0 = centerpoint1[match_i(0)][0];
        float y0 = centerpoint1[match_i(0)][1];
        float x1 = centerpoint1[match_j(0)][0];
        float y1 = centerpoint1[match_j(0)][1];
        float x2 = centerpoint1[match_k(0)][0];
        float y2 = centerpoint1[match_k(0)][1];
        float dist_1 = sqrt(pow(x0-x1,2)+ pow(y0-y1,2));
        float dist_2 = sqrt(pow(x1-x2,2)+ pow(y1-y2,2));
        float dist_3 = sqrt(pow(x0-x2,2)+ pow(y0-y2,2));
        float u0 = centerpoint2[match_i(1)][0];
        float v0 = centerpoint2[match_i(1)][1];
        float u1 = centerpoint2[match_j(1)][0];
        float v1 = centerpoint2[match_j(1)][1];
        float u2 = centerpoint2[match_k(1)][0];
        float v2 = centerpoint2[match_k(1)][1];
        float len_1 = sqrt(pow(u0-u1,2)+ pow(v0-v1,2));
        float len_2 = sqrt(pow(u1-u2,2)+ pow(v1-v2,2));
        float len_3 = sqrt(pow(u0-u2,2)+ pow(v0-v2,2));
        float scale_1 = dist_1/len_1;
        float scale_2 = dist_2/len_2;
        float scale_3 = dist_3/len_3;
        Eigen::Vector3f scale_vec(scale_1, scale_2, scale_3);
        float scale_ave = scale_vec.sum()/scale_vec.size();
        float error = pow(scale_1-scale_ave,2) + pow(scale_2-scale_ave,2) + pow(scale_3-scale_ave,2);
        std::cout << "id: " << match_i.head(2).transpose() << " " << match_j.head(2).transpose() << " " << match_k.head(2).transpose() << " ";
        std::cout << "scale: " << scale_1 << " " << scale_2 << " " << scale_3 << " " << error << std::endl;

        Eigen::VectorXf scale_mat(8); 
        scale_mat << match_i(0), match_i(1), match_j(0), match_j(1),
                    match_k(0), match_k(1), error, scale_ave;
        geometry_matches.push_back(scale_mat);
    }
    // checkpoint 3: geometry matches size()
    if(geometry_matches.size()<1)
    {
        std::cout << "geometry_matches < 1, no loop closure" << std::endl;
        return -1;
    }

    // sort by error
    std::vector<Eigen::VectorXf> geometry_matches_topk;
    int topk2 = geometry_matches.size();
    float min_scale_thre = 0.01; // min_similarity score
    // FindTopKMatch(coarse_matches, topk2, min_similarity_thre);
    for (size_t i = 0; i < geometry_matches.size(); i++)
        for (size_t j = i+1; j < geometry_matches.size(); j++)
    {
        if(geometry_matches[i](6) > geometry_matches[j](6))
        {
            Eigen::VectorXf tmp = geometry_matches[i];
            geometry_matches[i] = geometry_matches[j];
            geometry_matches[j] = tmp;
        }
    }
    for (size_t i = 0; i < topk2; i++)
    {
        if(geometry_matches[i](6)<min_scale_thre)
        {
            geometry_matches_topk.push_back(geometry_matches[i]);
            std::cout << "top_k_geometry_match: " << geometry_matches_topk[i].transpose() << std::endl;
        }
        else
            break;
    }


    // step 5: collect all possible matches
    std::vector<Eigen::Vector3f> final_matches; //id1, id2, scale
    // FindFinalMatches(geometry_matches_topk)
    for (size_t i = 0; i < geometry_matches_topk.size(); i++)
        for (size_t j = 0; j < 3; j++) // every toplḱ match has 3 id paar
    {
        Eigen::Vector3f id_tmp;
        bool label_exist = false;
        id_tmp(0) = geometry_matches_topk[i](2*j); // 0,2,4
        id_tmp(1) = geometry_matches_topk[i](2*j+1); // 1,3,5
        id_tmp(2) = geometry_matches_topk[i](7);
        // check if matched paar already exist
        for (size_t row = 0; row < final_matches.size(); row++)
        {
            if(id_tmp(0)==final_matches[row](0) || id_tmp(1)==final_matches[row](0))
            {
                label_exist = true;
                continue;
            }
        }
        if(label_exist)
            label_exist = false;
        else
            final_matches.push_back(id_tmp);
    }
    for (size_t i = 0; i < final_matches.size(); i++)
        std::cout << "final_matches: " << final_matches[i].transpose() << std::endl;
    for (size_t i = 0; i < final_matches.size(); i++)
        std::cout << "final_matches_id: " << centerpoint1[final_matches[i](0)][4] << " " << centerpoint2[final_matches[i](1)][4]<< std::endl;


    if(final_matches.size() < 3)
    {
        std::cout << "no loop "  << std::endl;
        return -1;
    }


    // Eigen::MatrixXd final_matches_eigen(final_matches.size(),2);
    // for (size_t obj_id = 0; obj_id < final_matches_eigen.rows(); obj_id++)
    // {
    //     final_matches_eigen(obj_id, 0) = final_matches[obj_id](0);
    //     final_matches_eigen(obj_id, 1) = final_matches[obj_id](1);
    // }
    // final_matches_eigen << 0,7, 1,8, 4,5;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_objects( new pcl::PointCloud<pcl::PointXYZRGB>() );
    // for (size_t obj_id = 0; obj_id < offline_mapobjects.rows(); obj_id++)
    // {
    //     pcl::PointXYZRGB p;
    //     p.x = offline_mapobjects(obj_id,1);
    //     p.y = offline_mapobjects(obj_id,2);
    //     p.z = offline_mapobjects(obj_id,3);
    //     p.r = color((int)offline_mapobjects(obj_id,0)%10,0);
    //     p.g = color((int)offline_mapobjects(obj_id,0)%10,1);
    //     p.b = color((int)offline_mapobjects(obj_id,0)%10,2);
    //     matched_objects->points.push_back(p);
    // }
    // // for (size_t line_id = 0; line_id < final_matches_eigen.rows(); line_id++)
    // // {
    // //     std::string line_name = "match_"+std::to_string(line_id);
    // //     int sta_id = final_matches_eigen(line_id, 0);
    // //     int end_id = final_matches_eigen(line_id, 1) + offline_mapobjects.rows()/2;
    // //     viewer->addLine<pcl::PointXYZRGB> (matched_objects->points[sta_id], matched_objects->points[end_id], line_name);
    // // }
    // for (size_t m = 0; m < final_matches_eigen.rows(); m++)
    //     for (size_t n = m+1; n < final_matches_eigen.rows(); n++)
    // {
    //     std::string line_name_1 = "match_"+std::to_string(m)+std::to_string(n)+"_1";
    //     std::string line_name_2 = "match_"+std::to_string(m)+std::to_string(n)+"_2";
    //     int sta_id_1 = final_matches_eigen(m, 0);
    //     int end_id_1 = final_matches_eigen(n, 0);
    //     int sta_id_2 = final_matches_eigen(m, 1) + offline_mapobjects.rows()/2;
    //     int end_id_2 = final_matches_eigen(n, 1) + offline_mapobjects.rows()/2;
    //     // std::cout << "sta_id_1 " << sta_id_1 << " end_id_1 " << end_id_1 << std::endl;
    //     // std::cout << "sta_id_2 " << sta_id_2 << " end_id_2 " << end_id_2 << std::endl;
    //     viewer->addLine<pcl::PointXYZRGB> (matched_objects->points[sta_id_1], matched_objects->points[end_id_1], 0.0f,1.0f,0.0f, line_name_1);
    //     viewer->addLine<pcl::PointXYZRGB> (matched_objects->points[sta_id_2], matched_objects->points[end_id_2], 0.0f,1.0f,0.0f, line_name_2);
    // }
    // if(show_orgin_point)
    // {
    //     while(!viewer->wasStopped())
    //     {
    //         viewer->spinOnce(1000);
    //     }            
    // }



    // calculate scale
    int obj_id_11 = final_matches[0](0);
    int obj_id_12 = final_matches[1](0);
    int obj_id_13 = final_matches[2](0);
    int obj_id_21 = final_matches[0](1);
    int obj_id_22 = final_matches[1](1);
    int obj_id_23 = final_matches[2](1);
    float x0 = centerpoint1[obj_id_11][0];
    float y0 = centerpoint1[obj_id_11][1];
    float z0 = centerpoint1[obj_id_11][2];
    float x1 = centerpoint1[obj_id_12][0];
    float y1 = centerpoint1[obj_id_12][1];
    float z1 = centerpoint1[obj_id_12][2];
    float x2 = centerpoint1[obj_id_13][0];
    float y2 = centerpoint1[obj_id_13][1];
    float z2 = centerpoint1[obj_id_13][2];
    float dist_1 = sqrt(pow(x0-x1,2)+ pow(y0-y1,2) + pow(z0-z1,2));
    float dist_2 = sqrt(pow(x1-x2,2)+ pow(y1-y2,2) + pow(z1-z2,2));
    float dist_3 = sqrt(pow(x0-x2,2)+ pow(y0-y2,2) + pow(z0-z2,2));
    float u0 = centerpoint2[obj_id_21][0];
    float v0 = centerpoint2[obj_id_21][1];
    float w0 = centerpoint2[obj_id_21][2];
    float u1 = centerpoint2[obj_id_22][0];
    float v1 = centerpoint2[obj_id_22][1];
    float w1 = centerpoint2[obj_id_22][2];
    float u2 = centerpoint2[obj_id_23][0];
    float v2 = centerpoint2[obj_id_23][1];
    float w2 = centerpoint2[obj_id_23][2];
    float len_1 = sqrt(pow(u0-u1,2)+ pow(v0-v1,2) + pow(w0-w1,2));
    float len_2 = sqrt(pow(u1-u2,2)+ pow(v1-v2,2) + pow(w1-w2,2));
    float len_3 = sqrt(pow(u0-u2,2)+ pow(v0-v2,2) + pow(w0-w2,2));
    float scale_1 = dist_1/len_1;
    float scale_2 = dist_2/len_2;
    float scale_3 = dist_3/len_3;
    Eigen::Vector3f scale_vec(scale_1, scale_2, scale_3);
    float scale_ave = scale_vec.sum()/scale_vec.size();
    float error = pow(scale_1-scale_ave,2) + pow(scale_2-scale_ave,2) + pow(scale_3-scale_ave,2);
    std::cout << "scale: " << scale_1 << " " << scale_2 << " " << scale_3 << " " << error << std::endl;


    // add points to point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    {
        // std::cout << "here 1 " << std::endl;
        vector<vector<float> > asso_point_tmp = object_asso_points[obj_id_11];
        for (size_t pt_id = 0; pt_id < asso_point_tmp.size(); pt_id++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            pt_tmp.x = asso_point_tmp[pt_id][0];
            pt_tmp.y = asso_point_tmp[pt_id][1];
            pt_tmp.z = asso_point_tmp[pt_id][2];
            pt_tmp.r = color((int)asso_point_tmp[pt_id][3]%10, 0);
            pt_tmp.g = color((int)asso_point_tmp[pt_id][3]%10, 1);
            pt_tmp.b = color((int)asso_point_tmp[pt_id][3]%10, 2);
            cloud1->points.push_back(pt_tmp);
        }            
        
        // std::cout << "here 2 " << std::endl;
        asso_point_tmp = object_asso_points[obj_id_12];
        for (size_t pt_id = 0; pt_id < asso_point_tmp.size(); pt_id++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            pt_tmp.x = asso_point_tmp[pt_id][0];
            pt_tmp.y = asso_point_tmp[pt_id][1];
            pt_tmp.z = asso_point_tmp[pt_id][2];
            pt_tmp.r = color((int)asso_point_tmp[pt_id][3]%10, 0);
            pt_tmp.g = color((int)asso_point_tmp[pt_id][3]%10, 1);
            pt_tmp.b = color((int)asso_point_tmp[pt_id][3]%10, 2);
            cloud1->points.push_back(pt_tmp);
        }      
        
        // std::cout << "here 3 " << std::endl;
        asso_point_tmp = object_asso_points[obj_id_13];
        for (size_t pt_id = 0; pt_id < asso_point_tmp.size(); pt_id++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            pt_tmp.x = asso_point_tmp[pt_id][0];
            pt_tmp.y = asso_point_tmp[pt_id][1];
            pt_tmp.z = asso_point_tmp[pt_id][2];
            pt_tmp.r = color((int)asso_point_tmp[pt_id][3]%10, 0);
            pt_tmp.g = color((int)asso_point_tmp[pt_id][3]%10, 1);
            pt_tmp.b = color((int)asso_point_tmp[pt_id][3]%10, 2);
            cloud1->points.push_back(pt_tmp);
        }          
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud1);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, rgb, "Point cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "Point cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
        while(!viewer->wasStopped())
        {
            viewer->spinOnce(1000);
        }
    
    }

    {
        int cpoint1_num = object_asso_points.size()/2;
        vector<vector<float> > asso_point_tmp = object_asso_points[obj_id_21+cpoint1_num];
        for (size_t pt_id = 0; pt_id < asso_point_tmp.size(); pt_id++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            pt_tmp.x = asso_point_tmp[pt_id][0];
            pt_tmp.y = asso_point_tmp[pt_id][1];
            pt_tmp.z = asso_point_tmp[pt_id][2];
            pt_tmp.r = color((int)asso_point_tmp[pt_id][3]%10, 0);
            pt_tmp.g = color((int)asso_point_tmp[pt_id][3]%10, 1);
            pt_tmp.b = color((int)asso_point_tmp[pt_id][3]%10, 2);
            cloud2->points.push_back(pt_tmp);
        }            
        asso_point_tmp = object_asso_points[obj_id_22+cpoint1_num];
        for (size_t pt_id = 0; pt_id < asso_point_tmp.size(); pt_id++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            pt_tmp.x = asso_point_tmp[pt_id][0];
            pt_tmp.y = asso_point_tmp[pt_id][1];
            pt_tmp.z = asso_point_tmp[pt_id][2];
            pt_tmp.r = color((int)asso_point_tmp[pt_id][3]%10, 0);
            pt_tmp.g = color((int)asso_point_tmp[pt_id][3]%10, 1);
            pt_tmp.b = color((int)asso_point_tmp[pt_id][3]%10, 2);
            cloud2->points.push_back(pt_tmp);
        }      
        asso_point_tmp = object_asso_points[obj_id_23+cpoint1_num];
        for (size_t pt_id = 0; pt_id < asso_point_tmp.size(); pt_id++)
        {
            pcl::PointXYZRGB pt_tmp; // x,y,z,rgb
            pt_tmp.x = asso_point_tmp[pt_id][0];
            pt_tmp.y = asso_point_tmp[pt_id][1];
            pt_tmp.z = asso_point_tmp[pt_id][2];
            pt_tmp.r = color((int)asso_point_tmp[pt_id][3]%10, 0);
            pt_tmp.g = color((int)asso_point_tmp[pt_id][3]%10, 1);
            pt_tmp.b = color((int)asso_point_tmp[pt_id][3]%10, 2);
            cloud2->points.push_back(pt_tmp);
        } 
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud2);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud2, rgb, "Point cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "Point cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
        while(!viewer->wasStopped())
        {
            viewer->spinOnce(1000);
        }         
    }


    // scale the points
    for (size_t i = 0; i < cloud2->points.size(); i++)
    {
        cloud2->points[i].x = cloud2->points[i].x * scale_ave;
        cloud2->points[i].y = cloud2->points[i].y * scale_ave;
        cloud2->points[i].z = cloud2->points[i].z * scale_ave;
    }

    // calculate R and T using ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud2);         
    icp.setInputTarget(cloud1);       
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);   // calculate from source to target


    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    Eigen::Matrix3f R = icp.getFinalTransformation().block(0,0,3,3);
    Eigen::Vector3f T = icp.getFinalTransformation().col(3).head(3);

    Eigen::Vector3f eulerAngle;
    rot_to_euler_zyx(R, eulerAngle(0), eulerAngle(1), eulerAngle(2));
    eulerAngle(0) = (eulerAngle(0)/3.1415926)*180;
    eulerAngle(1) = (eulerAngle(1)/3.1415926)*180;
    eulerAngle(2) = (eulerAngle(2)/3.1415926)*180;
    cout<<"Eular angle: "<<eulerAngle.transpose()<<endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_new = Final.makeShared();
    *Final_new += *cloud1;
    show_point_cloud(Final_new);


    return 1;

}
