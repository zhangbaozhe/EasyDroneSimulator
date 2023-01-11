//
// Created by Baozhe Zhang on 2023/1/11.
//

#ifndef EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_DISPLAYHANDLER_H_
#define EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_DISPLAYHANDLER_H_

#include <pangolin/pangolin.h>
#include "QuadrotorSimulator.h"

namespace ez_drone_sim
{


struct RotationMatrix {
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
};

std::ostream &operator<<(std::ostream &out, const RotationMatrix &r) {
  out.setf(std::ios::fixed);
  Eigen::Matrix3d matrix = r.matrix;
  out << '=';
  out << "[" << std::setprecision(2) << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],\n"
      << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],\n"
      << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
  return out;
}

std::istream &operator>>(std::istream &in, RotationMatrix &r) {
  return in;
}

struct TranslationVector {
  Eigen::Vector3d trans = Eigen::Vector3d(0, 0, 0);
};

std::ostream &operator<<(std::ostream &out, const TranslationVector &t) {
  out << "=[" << t.trans(0) << ',' << t.trans(1) << ',' << t.trans(2) << "]";
  return out;
}

std::istream &operator>>(std::istream &in, TranslationVector &t) {
  return in;
}

struct QuaternionDraw {
  Eigen::Quaterniond q;
};

std::ostream &operator<<(std::ostream &out, const QuaternionDraw quat) {
  auto c = quat.q.coeffs();
  out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
  return out;
}

std::istream &operator>>(std::istream &in, const QuaternionDraw quat) {
  return in;
}


class DisplayHandler
{
 public:
  DisplayHandler(const std::shared_ptr<QuadrotorSimulator> &quad_simulator_ptr,
                 const std::string &window_name) :
                 quad_simulator_ptr_(quad_simulator_ptr),
                 window_name_(window_name),
                 display_thread_(&DisplayHandler::startDisplay, this)
  {
  }

  ~DisplayHandler()
  {
    if (display_thread_.joinable())
      display_thread_.join();
  }

  // main function for thread
  void startDisplay();

  void finish()
  {
    if (display_thread_.joinable())
      display_thread_.join();
  }

 private:
  std::shared_ptr<QuadrotorSimulator> quad_simulator_ptr_;
  std::string window_name_;
  std::thread display_thread_;
  bool isFinish = false;
};

void DisplayHandler::startDisplay() {
  const int width = 640;
  const int height = 480;
  pangolin::CreateWindowAndBind(window_name_, width, height);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(width, height, 420, 420, 500, 300, 0.1, 1000),
      pangolin::ModelViewLookAt(-10, -10, 10, 0, 0, 0, pangolin::AxisZ)
      );
  const int UI_WIDTH = 500;

  pangolin::View &d_cam = pangolin::CreateDisplay().
      SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f / 480.0f).
      SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
  pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
  pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
  pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  while (!pangolin::ShouldQuit() && !isFinish) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1.0);

    d_cam.Activate(s_cam);

    pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
    Eigen::Matrix<double, 4, 4> m = matrix;

    RotationMatrix R;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        R.matrix(i, j) = m(j, i);
    rotation_matrix = R;

    TranslationVector t;
    t.trans = Eigen::Vector3d(m(0, 3), m(1, 3), m(2, 3));
    t.trans = -R.matrix * t.trans;
    translation_vector = t;

    TranslationVector euler;
    euler.trans = R.matrix.eulerAngles(2, 1, 0);
    euler_angles = euler;

    QuaternionDraw quat;
    quat.q = Eigen::Quaterniond(R.matrix);
    quaternion = quat;

    auto x = quad_simulator_ptr_->getState();
    Eigen::Vector3d r(x(0), x(1), x(2));
    Eigen::Quaterniond q(x(6), x(7), x(8), x(9));

    glColor3f(1.0, 0.0, 0.0);
    Eigen::Vector3d l(-0.5, -0.5, -0.5);
    Eigen::Vector3d h(0.5, 0.5, 0.5);
    Eigen::AlignedBox<double, 3> box(l, h);
    Eigen::Isometry3d transform;
    transform.rotate(q.toRotationMatrix());
    transform.pretranslate(r);
    box.transform(transform);

//    pangolin::glDrawAlignedBox(box);
    // FIXME:
    pangolin::glDrawAxis(transform.matrix(), 1.0);

//    pangolin::glDrawColouredCube();
    // draw the original axis
    glLineWidth(3);
    glColor3f(0.8f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(10, 0, 0);
    glColor3f(0.f, 0.8f, 0.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 10, 0);
    glColor3f(0.2f, 0.2f, 1.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 10);
    glEnd();

    pangolin::FinishFrame();
  }
}

}

#endif //EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_DISPLAYHANDLER_H_
