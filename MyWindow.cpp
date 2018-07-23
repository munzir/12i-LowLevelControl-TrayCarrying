/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "MyWindow.hpp"

#include <iostream>

//====================================================================
MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),
    mController(_controller),
    mCircleTask(false) {
  assert(_controller != nullptr);

  // Set the initial target positon to the initial position of the end effector
  // mTargetPosition = mController->getEndEffector("right")->getTransform().translation();
  // mLeftTargetPosition << 0.4, 0.1, 0.5;
  // mRightTargetPosition << 0.4, -0.1, 0.5;
  // mLeftTargetRPY << 0.0, 0.0, 0.0;
  // mRightTargetRPY << 0.0, 0.0, 0.0;
  
  Eigen::Matrix<double, 4, 4> baseTf = mController->mRobot->getBodyNode(0)->getTransform().matrix();
  double psi =  atan2(baseTf(0,0), -baseTf(1,0));
  Eigen::Transform<double, 3, Eigen::Affine> Tf0 = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  Tf0.rotate(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d Rot0 = (Tf0.matrix().block<3, 3>(0, 0)).transpose();
  Eigen::Vector3d xyz0 = (mController->mRobot->getPositions()).segment(3,3);
  
  mLeftTargetPosition = Rot0*(mController->getEndEffector("left")->getTransform().translation() - xyz0);
  mRightTargetPosition = Rot0*(mController->getEndEffector("right")->getTransform().translation() - xyz0);


  mLeftTargetRPY = dart::math::matrixToEulerXYZ(Rot0*mController->getEndEffector("left")->getTransform().rotation());
  mRightTargetRPY = dart::math::matrixToEulerXYZ(Rot0*mController->getEndEffector("right")->getTransform().rotation());
}

//====================================================================
MyWindow::~MyWindow() {}

//====================================================================
void MyWindow::timeStepping() {
  if (mCircleTask) {
    static double time = 0.0;
    const double dt = 0.0005;
    const double radius = 0.6;
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.1, 0.0);

    mLeftTargetPosition = center;
    mLeftTargetPosition[0] = radius * std::sin(time);
    mLeftTargetPosition[1] = 0.25 * radius * std::sin(time);
    mLeftTargetPosition[2] = radius * std::cos(time);

    mRightTargetPosition = center;
    mRightTargetPosition[0] = radius * std::sin(time);
    mRightTargetPosition[1] = 0.25 * radius * std::sin(time);
    mRightTargetPosition[2] = radius * std::cos(time);

    time += dt;
  }

  // Update the controller and apply control force to the robot
  mController->update(mLeftTargetPosition, mRightTargetPosition, mLeftTargetRPY, mRightTargetRPY);

  // Step forward the simulation
  mWorld->step();
}

//====================================================================
void MyWindow::drawWorld() const {
  // Draw the target position
  if (mRI) {
    // Get Frame 0 Transformation in Tf0
    Eigen::Matrix<double, 4, 4> baseTf = mController->mRobot->getBodyNode(0)->getTransform().matrix();
    double psi =  atan2(baseTf(0,0), -baseTf(1,0));
    Eigen::Transform<double, 3, Eigen::Affine> Tf0 = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Tf0.rotate(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d Rot = Tf0.matrix().block<3, 3>(0, 0);
    Eigen::Vector3d xyz0 = (mController->mRobot->getPositions()).segment(3,3);

    // Draw Left Target Frame
    Eigen::Matrix3d mat;
    mat =  Eigen::AngleAxisd(mLeftTargetRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(mLeftTargetRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(mLeftTargetRPY(2), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d localTranslation; 
    double axisLength = 0.1;
    double axisWidth = 0.003;
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mLeftTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mLeftTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mLeftTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();

    // Draw Right Target Frame
    mat =  Eigen::AngleAxisd(mRightTargetRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(mRightTargetRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(mRightTargetRPY(2), Eigen::Vector3d::UnitZ());
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mRightTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mRightTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mRightTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();


    // Draw Left End-Effector Frame
    Eigen::Vector3d eeRPY = dart::math::matrixToEulerXYZ(mController->getEndEffector("left")->getTransform().rotation());
    Eigen::Vector3d eePosition = mController->getEndEffector("left")->getTransform().translation();
    
    mat =  Eigen::AngleAxisd(eeRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(eeRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(eeRPY(2), Eigen::Vector3d::UnitZ());
    axisLength = 0.08;
    axisWidth = 0.006;
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();


    // Draw Right End-Effector Frame
    eeRPY = dart::math::matrixToEulerXYZ(mController->getEndEffector("right")->getTransform().rotation());
    eePosition = mController->getEndEffector("right")->getTransform().translation();
    
    mat =  Eigen::AngleAxisd(eeRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(eeRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(eeRPY(2), Eigen::Vector3d::UnitZ());
    axisLength = 0.08;
    axisWidth = 0.006;
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();


    // mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    // mRI->pushMatrix();
    // mRI->translate( \
    //   (mController->mRobot->getPositions()).segment(3,3) \
    //   + Tf0.matrix().block<3, 3>(0, 0)*mLeftTargetPosition);
    // mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    // mRI->popMatrix();    

    // mRI->setPenColor(Eigen::Vector3d(0.0, 0.4, 0.2));
    // mRI->pushMatrix();
    // mRI->translate( \
    //   (mController->mRobot->getPositions()).segment(3,3) \
    //   + Tf0.matrix().block<3, 3>(0, 0)*mRightTargetPosition);
    // mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    // mRI->popMatrix();   

    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(mWorld->getSkeleton("krang")->getCOM());
    mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    mRI->popMatrix();    
    

  }

  // Draw world
  SimWindow::drawWorld();
}

//====================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  double incremental = 0.01;
  Eigen::Matrix3d rot;
  // Eigen::IOFormat commaSep(Eigen::StreamPrecision, 0, ", ", "\n", "", "", "",  "");

  switch (_key) {
    case 'c':  // print debug information
      if (mCircleTask) {
        std::cout << "Circle task [off]." << std::endl;
        mCircleTask = false;
      }
      else {
        std::cout << "Circle task [on]." << std::endl;
        mCircleTask = true;
      }
      break;

    case 'q':
      mLeftTargetPosition[0] -= incremental;
      break;
    case 'w':
      mLeftTargetPosition[0] += incremental;
      break;
    case 'a':
      mLeftTargetPosition[1] -= incremental;
      break;
    case 's':
      mLeftTargetPosition[1] += incremental;
      break;
    case 'z':
      mLeftTargetPosition[2] -= incremental;
      break;
    case 'x':
      mLeftTargetPosition[2] += incremental;
      break;
    
    case '[':
      mRightTargetPosition[0] -= incremental;
      break;
    case ']':
      mRightTargetPosition[0] += incremental;
      break;
    case ';':
      mRightTargetPosition[1] -= incremental;
      break;
    case '\'':
      mRightTargetPosition[1] += incremental;
      break;
    case '.':
      mRightTargetPosition[2] -= incremental;
      break;
    case '/':
      mRightTargetPosition[2] += incremental;
      break;
    
    case 'r':
      mLeftTargetRPY[0] -= incremental;
      break;
    case 't':
      mLeftTargetRPY[0] += incremental;
      break;
    case 'f':
      mLeftTargetRPY[1] -= incremental;
      break;
    case 'g':
      mLeftTargetRPY[1] += incremental;
      break;
    case 'v':
      mLeftTargetRPY[2] -= incremental;
      break;
    case 'b':
      mLeftTargetRPY[2] += incremental;
      break;

    case 'i':
      mRightTargetRPY[0] -= incremental;
      break;
    case 'o':
      mRightTargetRPY[0] += incremental;
      break;
    case 'j':
      mRightTargetRPY[1] -= incremental;
      break;
    case 'k':
      mRightTargetRPY[1] += incremental;
      break;
    case 'n':
      mRightTargetRPY[2] -= incremental;
      break;
    case 'm':
      mRightTargetRPY[2] += incremental;
      break;

    case 'd':
      cout << (mController->mRobot->getPositions().transpose()) << endl;
      break;

    case 'e':
      rot <<  1, 0,  0,
              0, 0, -1,
              0, 1,  0;
      mLeftTargetRPY = dart::math::matrixToEulerXYZ(rot);
      break;

    case 'p':
      rot << -1, 0, 0,
              0, 0, 1,
              0, 1, 0;
      mRightTargetRPY = dart::math::matrixToEulerXYZ(rot);      
      break;

    case 'h':
      mLeftTargetPosition[0] = (mLeftTargetPosition[0] + mRightTargetPosition[0])/2;
      mRightTargetPosition[0] = mLeftTargetPosition[0];

      mLeftTargetPosition[1] = (mLeftTargetPosition[1] - mRightTargetPosition[1])/2;
      mRightTargetPosition[1] = -mLeftTargetPosition[1];

      mLeftTargetPosition[2] = (mLeftTargetPosition[2] + mRightTargetPosition[2])/2;
      mRightTargetPosition[2] = mLeftTargetPosition[2];

      cout << "left Ref: " << mLeftTargetPosition << endl;
      cout << "right Ref: " << mRightTargetPosition << endl;
      break;
      

    default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}
