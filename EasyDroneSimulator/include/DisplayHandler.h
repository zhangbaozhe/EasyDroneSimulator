//
// Created by Baozhe Zhang on 2023/1/11.
//

#ifndef EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_DISPLAYHANDLER_H_
#define EASYDRONESIMULATOR_EASYDRONESIMULATOR_INCLUDE_DISPLAYHANDLER_H_

#include <pangolin/pangolin.h>
#include "QuadrotorSimulator.h"

namespace ez_drone_sim
{

class DisplayHandler
{
 public:
  DisplayHandler(const std::shared_ptr<QuadrotorSimulator> &quad_simulator_ptr,
                 const std::string &window_name) :
                 quad_simulator_ptr_(quad_simulator_ptr),
                 window_name_(window_name)
  {
  }

  ~DisplayHandler()
  {
  }

  // main function for thread
  void startDisplay();



 private:
  std::shared_ptr<QuadrotorSimulator> quad_simulator_ptr_;
  std::string window_name_;
  bool isFinish = false;
};

void DisplayHandler::startDisplay() {
  const int width = 640;
  const int height = 480;
  pangolin::CreateWindowAndBind(window_name_, width, height);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(width, height, 420, 420, 500, 300, 0.1, 1000),
      pangolin::ModelViewLookAt(10, -1, 50, 0, 0, 0, pangolin::AxisZ)
      );
  const int UI_WIDTH = 500;

  pangolin::View &d_cam = pangolin::CreateDisplay().
      SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f).
      SetHandler(new pangolin::Handler3D(s_cam));




  while (!pangolin::ShouldQuit() && !isFinish) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1.0);

    d_cam.Activate(s_cam);


    auto x = quad_simulator_ptr_->getState();
    Eigen::Vector3f r(x(0), x(1), x(2));
    Eigen::Quaternionf q(x(6), x(7), x(8), x(9));

    glLineWidth(5);
    glColor3f(1.0, 0.0, 0.0);
    const double d = 0.5;
    Eigen::Isometry3f transform;
    transform.rotate(q.toRotationMatrix());
    transform.pretranslate(r);
    Eigen::Vector3f p1(d, -d, -d);
    Eigen::Vector3f p2(d, d, -d);
    Eigen::Vector3f p3(-d, d, -d);
    Eigen::Vector3f p4(-d, -d, -d);
    Eigen::Vector3f p5(d, -d, d);
    Eigen::Vector3f p6(d, d, d);
    Eigen::Vector3f p7(-d, d, d);
    Eigen::Vector3f p8(-d, -d, d);
    p1 = r + q.toRotationMatrix() * p1;
    p2 = r + q.toRotationMatrix() * p2;
    p3 = r + q.toRotationMatrix() * p3;
    p4 = r + q.toRotationMatrix() * p4;
    p5 = r + q.toRotationMatrix() * p5;
    p6 = r + q.toRotationMatrix() * p6;
    p7 = r + q.toRotationMatrix() * p7;
    p8 = r + q.toRotationMatrix() * p8;

    glBegin(GL_LINES);
    // 1 -> 2
    glVertex3fv(p1.data());
    glVertex3fv(p2.data());
    // 2 -> 3
    glVertex3fv(p2.data());
    glVertex3fv(p3.data());
    // 3 -> 4
    glVertex3fv(p3.data());
    glVertex3fv(p4.data());
    // 4 -> 1
    glVertex3fv(p4.data());
    glVertex3fv(p1.data());
    // 5 -> 6
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3fv(p5.data());
    glVertex3fv(p6.data());
    // 6 -> 7
    glVertex3fv(p6.data());
    glVertex3fv(p7.data());
    // 7 -> 8
    glVertex3fv(p7.data());
    glVertex3fv(p8.data());
    // 8 -> 5
    glVertex3fv(p8.data());
    glVertex3fv(p5.data());
    // 1 -> 5
    glColor3f(1.0f, 0.f, 0.f);
    glVertex3fv(p1.data());
    glVertex3fv(p5.data());
    // 2 -> 6
    glVertex3fv(p2.data());
    glVertex3fv(p6.data());
    // 3 -> 7
    glVertex3fv(p3.data());
    glVertex3fv(p7.data());
    // 4 -> 8
    glVertex3fv(p4.data());
    glVertex3fv(p8.data());

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
