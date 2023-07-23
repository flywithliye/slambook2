#include <iostream>
#include <iomanip>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

#include <pangolin/pangolin.h>

// rotation matrix R
struct RotationMatrix
{
    Matrix3d matrix = Matrix3d::Identity();
};
// 重载 << 运算符，用于输出 RotationMatrix 对象
ostream &operator<<(ostream &out, const RotationMatrix &r)
{
    out.setf(ios::fixed);
    Matrix3d matrix = r.matrix;
    out << '=';
    out << "[" << setprecision(2) << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],"
        << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],"
        << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
    return out;
}
// 重载 >> 运算符，用于输入 RotationMatrix 对象
istream &operator>>(istream &in, RotationMatrix &r)
{
    return in;
}

// translation vector t
struct TranslationVector
{
    Vector3d trans = Vector3d(0, 0, 0);
};
// 重载 << 运算符，用于输出 TranslationVector 对象
ostream &operator<<(ostream &out, const TranslationVector &t)
{
    out << "=[" << t.trans(0) << ',' << t.trans(1) << ',' << t.trans(2) << "]";
    return out;
}
// 重载 >> 运算符，用于输入 TranslationVector 对象
istream &operator>>(istream &in, TranslationVector &t)
{
    return in;
}

// quaternion q
struct QuaternionDraw
{
    Quaterniond q;
};
// 重载 << 运算符，用于输出 QuaternionDraw 对象
ostream &operator<<(ostream &out, const QuaternionDraw quat)
{
    auto c = quat.q.coeffs();
    out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
    return out;
}
// 重载 >> 运算符，用于输入 QuaternionDraw 对象
istream &operator>>(istream &in, const QuaternionDraw quat)
{
    return in;
}

int main(int argc, char **argv)
{
    pangolin::CreateWindowAndBind("visualize geometry", 1000, 600);
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)); // 设置相机的位置、观察的目标点以及上方向

    // OpenGlRenderState 用于管理OpenGL渲染状态
    // ProjectionMatrix：投影矩阵，用于定义透视或正交投影。
    // ModelViewMatrix：模型视图矩阵，用于定义物体的位置、旋转和缩放。

    // ProjectionMatrix 投影矩阵 -> 相机成像模型
    // w: 屏幕宽度（像素）。
    // h: 屏幕高度（像素）。
    // fu: 水平焦距。
    // fv: 垂直焦距。
    // u0: 图像中心点在 x 轴上的位置。
    // v0: 图像中心点在 y 轴上的位置。
    // zNear: 近平面到相机位置的距离。
    // zFar: 远平面到相机位置的距离。

    // ModelViewMatrix 模型视图矩阵 -> 确定相机位姿态
    // ex、ey、ez：相机的位置坐标。
    // lx、ly、lz：相机所看的目标点的位置坐标。
    // up：相机的上方向（AxisDirection 枚举类型，可能是 +X、-X、+Y、-Y、+Z 或 -Z）。

    const int UI_WIDTH = 500;

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    // ui
    pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
    pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
    pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
    pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    while (!pangolin::ShouldQuit())
    {   
        // 清空颜色缓冲区和深度缓冲区，以准备进行新的渲染
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 激活相机视角
        d_cam.Activate(s_cam);

        // 获取模型视图矩阵
        // 模型视图矩阵是一个OpenGL中的变换矩阵，包含了旋转、平移和缩放等变换信息
        // 将物体的局部坐标系转换到世界坐标系中 Twc
        pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
        Matrix<double, 4, 4> m = matrix;

        // 更新旋转矩阵 R
        // 在将模型视图矩阵的旋转部分提取出来时，它需要进行转置操作。
        // 在OpenGL中，旋转矩阵采用列主序（column-major）的方式存储，即旋转矩阵的每一列代表一个坐标轴的方向向量。
        // 而在Eigen库（使用Matrix类型表示矩阵）中，默认采用的是行主序（row-major）的方式存储矩阵，即矩阵的每一行代表一个坐标轴的方向向量。
        RotationMatrix R;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R.matrix(i, j) = m(j, i);
        rotation_matrix = R;

        // 更新平移向量 t
        TranslationVector t;
        t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));
        t.trans = -R.matrix * t.trans;  // todo
        translation_vector = t;
        
        // 更新欧拉角 rpy
        TranslationVector euler;
        euler.trans = R.matrix.eulerAngles(2, 1, 0);
        euler_angles = euler;

        // 更新四元数 q
        QuaternionDraw quat;
        quat.q = Quaterniond(R.matrix);
        quaternion = quat;

        // 绘制一个有颜色的立方体
        glColor3f(1.0, 1.0, 1.0);
        pangolin::glDrawColouredCube();

        // 绘制原始坐标轴
        glLineWidth(3);
        glColor3f(0.8f, 0.f, 0.f);  // x轴红色
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(10, 0, 0);
        glColor3f(0.f, 0.8f, 0.f);  // y轴绿色
        glVertex3f(0, 0, 0);
        glVertex3f(0, 10, 0);
        glColor3f(0.2f, 0.2f, 1.f);  // z轴蓝色
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 10);
        glEnd();

        // 完成此帧的渲染
        pangolin::FinishFrame();
    }
}
