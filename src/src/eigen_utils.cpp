 /*
  * Copyright (c) 2013, Willow Garage, Inc.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  *     * Redistributions of source code must retain the above copyright
  *       notice, this list of conditions and the following disclaimer.
  *     * Redistributions in binary form must reproduce the above copyright
  *       notice, this list of conditions and the following disclaimer in the
  *       documentation and/or other materials provided with the distribution.
  *     * Neither the name of the Willow Garage, Inc. nor the names of its
  *       contributors may be used to endorse or promote products derived from
  *       this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *
  * Author: Mario Prats
  * Much of this code has been adapted from the ViSP library (http://www.irisa.fr/lagadic/visp)
  */
#include <math.h>
#include "sdf_3d_reconstruction/eigen_utils.h"
#include<Eigen/Eigen>

#ifndef EIGENUTILS_H
#define EIGENUTILS_H
namespace eigen_utils {
 
 static const double ang_min_sinc = 1.0e-8;
 static const double ang_min_mc = 2.5e-4;
 
 double f_sinc(double sinx, double x)
 {
   if (fabs(x) < ang_min_sinc) return 1.0 ;
   else  return (sinx / x) ;
 }
 
 double f_mcosc(double cosx, double x)
 {
   if (fabs(x) < ang_min_mc) return 0.5 ;
   else  return ((1.0 - cosx) / x / x) ;
 }
 
 double f_msinc(double sinx, double x)
 {
   if (fabs(x) < ang_min_mc) return (1. / 6.0) ;
   else  return ((1.0 - sinx / x) / x / x) ;
 }
 
 Eigen::Affine3d UThetaToAffine3d(const Eigen::Vector3d &u)
 {
   Eigen::Affine3d rd;
   double theta, si, co, sinc, mcosc;
 
   theta = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
   si = sin(theta);
   co = cos(theta);
   sinc = f_sinc(si,theta);
   mcosc = f_mcosc(co,theta);
 
   rd(0,0) = co + mcosc*u[0]*u[0];
   rd(0,1) = -sinc*u[2] + mcosc*u[0]*u[1];
   rd(0,2) = sinc*u[1] + mcosc*u[0]*u[2];
   rd(1,0) = sinc*u[2] + mcosc*u[1]*u[0];
   rd(1,1) = co + mcosc*u[1]*u[1];
   rd(1,2) = -sinc*u[0] + mcosc*u[1]*u[2];
   rd(2,0) = -sinc*u[1] + mcosc*u[2]*u[0];
   rd(2,1) = sinc*u[0] + mcosc*u[2]*u[1];
   rd(2,2) = co + mcosc*u[2]*u[2];
 
   return rd;
 }
 
 Eigen::Affine3d direct_exponential_map(const Eigen::VectorXd &v, double delta_t)
 {
   double theta,si,co,sinc,mcosc,msinc;
   Eigen::Vector3d u;
   Eigen::Affine3d rd;
   rd.setIdentity();
   Eigen::Vector3d dt;
 
   Eigen::VectorXd v_dt = v * delta_t;
 
   u[0] = v_dt[3];
   u[1] = v_dt[4];
   u[2] = v_dt[5];
 
   rd = UThetaToAffine3d(u);
 
   theta = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
   si = sin(theta);
   co = cos(theta);
   sinc = f_sinc(si,theta);
   mcosc = f_mcosc(co,theta);
   msinc = f_msinc(si,theta);
 
   dt[0] = v_dt[0] * (sinc + u[0]*u[0]*msinc)
                       + v_dt[1]*(u[0]*u[1]*msinc - u[2]*mcosc)
                       + v_dt[2]*(u[0]*u[2]*msinc + u[1]*mcosc);
 
   dt[1] = v_dt[0] * (u[0]*u[1]*msinc + u[2]*mcosc)
                       + v_dt[1]*(sinc + u[1]*u[1]*msinc)
                       + v_dt[2]*(u[1]*u[2]*msinc - u[0]*mcosc);
 
   dt[2] = v_dt[0] * (u[0]*u[2]*msinc - u[1]*mcosc)
                       + v_dt[1]*(u[1]*u[2]*msinc + u[0]*mcosc)
                       + v_dt[2]*(sinc + u[2]*u[2]*msinc);
 
   Eigen::Affine3d Delta;
   Delta.setIdentity();
   Delta = rd;
   Delta(0,3) = dt[0];
   Delta(1,3) = dt[1];
   Delta(2,3) = dt[2];
 
   return Delta;
 }
}

#endif
