#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

typedef vector<Vector4d, Eigen::aligned_allocator<Vector4d>> PointCloud; // 点云类型

// 文件路径
string left_file = "./../stereo/left.png";
string right_file = "./../stereo/right.png";

// 在pangolin中画图，已写好，无需调整
int showPointCloud(const PointCloud &pointcloud);

int main(int argc, char **argv)
{

    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 基线
    double b = 0.573;

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); // 神奇的参数
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left.rows; v++)
    {
        for (int u = 0; u < left.cols; u++)
        {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
            {
                continue;
            }

            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));

            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
    }

    cv::imshow("disparity", disparity / 96.0);
    cv::waitKey(0);

    // 画出点云
    showPointCloud(pointcloud);
    return 0;
}

int showPointCloud(const PointCloud &pointcloud)
{

    if (pointcloud.empty())
    {
        cerr << "Point cloud is empty!" << endl;
        return 0;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // 绘制坐标轴
        glColor3f(1.0f, 0.0f, 0.0f); // 绘制X轴 设置颜色为红色(RGB)
        pangolin::glDrawLine(0, 0, 0, 1, 0, 0);
        glColor3f(0.0f, 1.0f, 0.0f); // 绘制Y轴 设置颜色为绿色(RGB)
        pangolin::glDrawLine(0, 0, 0, 0, 1, 0);
        glColor3f(0.0f, 0.0f, 1.0f); // 绘制Z轴 设置颜色为蓝色(RGB)
        pangolin::glDrawLine(0, 0, 0, 0, 0, 1);

        // 绘制点云
        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p : pointcloud)
        {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
    }

    return 1;
}
