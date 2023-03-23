#include <iostream>
#include <utility>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>    // 可视化
#include <pcl/features/principal_curvatures.h>
#include <boost/thread/thread.hpp>

using namespace std;

bool concatenateCurvature(pcl::PointCloud<pcl::PointNormal>& cloud, pcl::PointCloud<pcl::PrincipalCurvatures>& curv, std::pair<float, float>& curvature_range)
{
    float min_c, max_c;
    min_c = FLT_MAX;
    max_c = -FLT_MAX;
    if (cloud.points.size()!=curv.points.size())
    {
        std::cout <<"Point numbers not equal!" <<std::endl;
        return false;
    }
	for (int i = 0; i < cloud.points.size(); i++){
		//平均曲率
		float curvature = ((curv)[i].pc1 + (curv)[i].pc2) / 2;
		//高斯曲率
		//float curvature = (*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2;
        cloud.points[i].curvature = curvature;
        if (curvature > max_c) max_c = curvature;
        if (curvature < min_c) min_c = curvature;
	}
    curvature_range.first = min_c;
    curvature_range.second = max_c;
    return true;
}

bool colorPointsbyCurvature (pcl::PointCloud<pcl::PointXYZRGBNormal>& rgb_cloud, pcl::PointCloud<pcl::PointNormal>& cloud, std::pair<float, float>& curv_range )
{
    int r_begin, r_end, g_begin, g_end, b_begin, b_end;
    r_begin = 255; r_end=0; //if (r_end<r_begin) swap(r_end, r_begin);
    g_begin=0; g_end=255; //if (g_end<g_begin) swap(g_end, g_begin);
    b_begin=0; b_end=0; //if (b_end<b_begin) swap(b_end, b_begin);

    float curv_min = curv_range.first;
    float curv_max = curv_range.second;

    rgb_cloud.points.resize(cloud.points.size());
    for(int i=0; i<cloud.points.size(); i++)
    {
        rgb_cloud.points[i].x = cloud.points[i].x;
        rgb_cloud.points[i].y = cloud.points[i].y;
        rgb_cloud.points[i].z = cloud.points[i].z;
        rgb_cloud.points[i].normal_x = cloud.points[i].normal_x;
        rgb_cloud.points[i].normal_y = cloud.points[i].normal_y;
        rgb_cloud.points[i].normal_z = cloud.points[i].normal_z;
        rgb_cloud.points[i].curvature = cloud.points[i].curvature;
        {
            rgb_cloud.at(i).r = r_begin + static_cast<int>(static_cast<float>(r_end - r_begin)* ((cloud.points[i].curvature - curv_range.first)/(curv_range.second - curv_range.first)));
            rgb_cloud.at(i).g = g_begin + static_cast<int>(static_cast<float>(g_end - g_begin)* ((cloud.points[i].curvature - curv_range.first)/(curv_range.second - curv_range.first)));
            rgb_cloud.at(i).b = b_begin + static_cast<int>(static_cast<float>(b_end - b_begin)* ((cloud.points[i].curvature - curv_range.first)/(curv_range.second - curv_range.first)));
        }
    }
    return true;
}


int main()
{
    // -----------------------加载点云--------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jeff/codes/fast_lio2/src/FAST_LIO2/PCD/scans.pcd", *cloud) == -1)
    {
        PCL_ERROR("读取源标点云失败 \n");
        return (-1);
    }
    cout << "从点云中读取 " << cloud->size() << " 个点" << endl;

    //-----------------------法线估计-------------------------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setNumberOfThreads(4);//设置openMP的线程数
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //-----------------连接XYZ和法向量字段--------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    cout << "normals calculated." << endl;

    //计算点云曲率
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud);
	pc.setInputNormals(normals);
	pc.setSearchMethod(tree);
	pc.setRadiusSearch(0.02);//设置搜索半径，可设为0.01
	//pc.setKSearch(5);
	pc.compute(*cloud_curvatures);

    std::pair<float, float> curvature_range;
    if (!concatenateCurvature(*cloud_with_normals, *cloud_curvatures, curvature_range))
    {
        std::cout <<"error!"<<std::endl;
    }

    cout <<"curvature range: [" << curvature_range.first << ", " << curvature_range.second << "]"<<endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    colorPointsbyCurvature(*cloud_rgb, *cloud_with_normals, curvature_range);

    cout <<"Color changed. Start saving pcd."<< endl;

    std::string pcdname = "/home/jeff/codes/fast_lio2/src/FAST_LIO2/PCD/rgb_scan.pcd";

    if (pcl::io::savePCDFileBinary(pcdname.c_str(), *cloud_rgb)==-1)// scan 
    {
        cout << "Saving pcd error occurred" << endl;
    }

    return 0;
}

