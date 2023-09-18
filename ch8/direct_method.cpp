#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// baseline
double baseline = 0.573;
// paths
string left_file = "./../left.png";
string disparity_file = "./../disparity.png";
boost::format fmt_others("./../%06d.png"); // other files "%06d"表示一个六位数的整数

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// class for accumulator jacobians in parallel
class JacobianAccumulator
{
public:
    JacobianAccumulator(
        const cv::Mat &img1_,
        const cv::Mat &img2_,
        const VecVector2d &px_ref_,
        const vector<double> depth_ref_,
        Sophus::SE3d &T21_) : img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_)
    {
        projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
    }

    /// accumulate jacobians in a range
    void accumulate_jacobian(const cv::Range &range); // 用于累积Jacobian矩阵的函数，该函数可能会由多个线程并行调用

    /// get hessian matrix
    Matrix6d hessian() const { return H; }

    /// get bias
    Vector6d bias() const { return b; }

    /// get total cost
    double cost_func() const { return cost; }

    /// get projected points
    VecVector2d projected_points() const { return projection; }

    /// reset h, b, cost to zero
    void reset()
    {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }

private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;      // 2D点
    const vector<double> depth_ref; // 深度
    Sophus::SE3d &T21;              // 位姿 - 优化变量
    VecVector2d projection;         // projected points

    std::mutex hessian_mutex;      // 定义互斥锁hessian_mutex 此处仅定义了一个std::mutex类型的变量 暂无实际作用
    Matrix6d H = Matrix6d::Zero(); // H b cost是线程共享的，需要使用互斥锁进行保护
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) // 实现方式/原理 可参考optical_flow.cpp
{
    // boundary check
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols)
        x = img.cols - 1;
    if (y >= img.rows)
        y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)]; // 指向图像(x,y)位置的uchar指针 img.step是图像每行的字节数
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step + 0] +
        xx * yy * data[img.step + 1]);
}

int main(int argc, char **argv)
{

    cv::Mat left_img = cv::imread(left_file, 0); // always convert image to the single channel grayscale image
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 2000;
    int boarder = 20;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++)
    {
        int x = rng.uniform(boarder, left_img.cols - boarder); // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder); // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;

    for (int i = 1; i < 6; i++)
    { // 1~5
        std::cout << "正在估计图像" << (fmt_others % i).str() << "位姿" << std::endl;
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);

        // try single layer by uncomment this line
        std::cout << "单层直接法" << std::endl;
        DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);

        std::cout << "多层直接法" << std::endl;
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref); // 越往后 光流变化越大
    }
    return 0;
}

void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{

    const int iterations = 10;
    double cost = 0, lastCost = 0;
    auto t1 = chrono::steady_clock::now();
    JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, T21);

    for (int iter = 0; iter < iterations; iter++)
    {
        jaco_accu.reset();
        cv::parallel_for_(cv::Range(0, px_ref.size()),
                          std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));

        // 获取最终 依据全部参考点及其邻域获得的 H&b
        Matrix6d H = jaco_accu.hessian();
        Vector6d b = jaco_accu.bias();

        // solve update and put it into estimation
        Vector6d update = H.ldlt().solve(b);
        ;
        T21 = Sophus::SE3d::exp(update) * T21;
        cost = jaco_accu.cost_func(); // 当前优化误差

        if (std::isnan(update[0]))
        {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost)
        {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        if (update.norm() < 1e-3)
        {
            // converge
            break;
        }

        lastCost = cost;
        cout << "iteration: " << iter << ", cost: " << cost << endl;
    }

    cout << "T21 = \n"
         << T21.matrix() << endl;
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "direct method for single layer: " << time_used.count() << endl;

    // plot the projected pixels here
    cv::Mat img2_show;
    cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
    VecVector2d projection = jaco_accu.projected_points(); // 包括图1中所有的参考点在图2的投影
    for (size_t i = 0; i < px_ref.size(); ++i)
    {
        auto p_ref = px_ref[i];
        auto p_cur = projection[i];
        if (p_cur[0] > 0 && p_cur[1] > 0) // 这个点没有被投影到相机外面 被第二个图像观测到了
        {
            cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);  // 投影点
            cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]), // 光流
                     cv::Scalar(0, 250, 0));
        }
    }
    cv::imshow("current", img2_show);
    cv::waitKey();
}

void JacobianAccumulator::accumulate_jacobian(const cv::Range &range)
{

    // parameters
    const int half_patch_size = 1;
    int cnt_good = 0;
    Matrix6d hessian = Matrix6d::Zero(); // 每个线程中，局部的H,b,和 cost，只在当前线程中有效
    Vector6d bias = Vector6d::Zero();
    double cost_tmp = 0;

    for (size_t i = range.start; i < range.end; i++)
    {

        // compute the projection in the second image
        Eigen::Vector3d point_ref = // 相机1坐标系坐标 = 深度 x 归一化坐标
            depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
        Eigen::Vector3d point_cur = T21 * point_ref; // 相机2坐标系坐标
        if (point_cur[2] < 0)                        // depth invalid
            continue;

        float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy; // 相机2像素坐标
        if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
            v > img2.rows - half_patch_size)
            continue;

        projection[i] = Eigen::Vector2d(u, v);                       // 相机2像素坐标 - 私有成员
        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2], // 相机2坐标系坐标
            Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;     // 用于雅可比计算的中间变量
        cnt_good++;

        // and compute error and jacobian
        for (int x = -half_patch_size; x <= half_patch_size; x++)
            for (int y = -half_patch_size; y <= half_patch_size; y++)
            {

                double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) - // 图1参考点光度
                               GetPixelValue(img2, u + x, v + y);                        // 图2投影位置光度
                Matrix26d J_pixel_xi;                                                    // 投影对变换的倒数
                Eigen::Vector2d J_img_pixel;                                             // 像素梯度

                J_pixel_xi(0, 0) = fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) = fy * Z_inv;
                J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) = fy * X * Z_inv;

                J_img_pixel = Eigen::Vector2d(
                    0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
                    0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y)));

                // total jacobian
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose(); // 1*2 x 2*6 -> 6*1

                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;
            }
    }

    if (cnt_good) // 所有局部计算完成之后，更新类级别的成员变量。减少锁的竞争，提高性能。
    {
        // set hessian, bias and cost
        // 并通过 std::unique_lock 加锁，以确保对成员变量 H, b, 和 cost 的更新是线程安全的
        unique_lock<mutex> lck(hessian_mutex); // hessian_mutex为私有成员
        H += hessian;
        b += bias;
        cost += cost_tmp / cnt_good;
    }
}

void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++)
    {
        if (i == 0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            cv::Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale)); // 缩小图像
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;      // backup the old values 全局变量 传值
    for (int level = pyramids - 1; level >= 0; level--) // 由低分辨率 -> 高分辨率
    {
        std::cout << "金字塔第" << level << "层" << std::endl;
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px : px_ref)
        {
            px_ref_pyr.push_back(scales[level] * px); // int x Vector2d -> Vector2d
        }

        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21); // T21作为下一层的初始值
    }
}
