/*
 * Copyright (c) 2018 Smart Mapping Tech Co., Ltd. All rights reserved.
 *
 * Created by Shuang Song on 5/28/18.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ETHZASL_MSF_NOROS_MY_TYPES_H
#define ETHZASL_MSF_NOROS_MY_TYPES_H

#include <Eigen/Dense>
#include <memory>
typedef struct Meas_ {
  typedef std::shared_ptr<Meas_> Ptr;
  uint64_t timestamp;
  uint32_t seq;
  enum { IMU, VICON };
  virtual int type() const = 0;
} Meas;

typedef struct IMUMeas_ : public Meas_ {
  typedef std::shared_ptr<IMUMeas_> Ptr;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
  virtual int type() const { return IMU; }
} IMUMeas;

typedef struct VICONMeas_ : public Meas_ {
  typedef std::shared_ptr<VICONMeas_> Ptr;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  virtual int type() const { return VICON; }
} VICONMeas;

#endif //ETHZASL_MSF_NOROS_MY_TYPES_H
