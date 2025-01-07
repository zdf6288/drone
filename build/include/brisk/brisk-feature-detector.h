/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 Copyright (C) 2020  Smart Robotics Lab, Imperial College London
 Stefan Leutenegger

 Copyright (C) 2024  Smart Robotics Lab, Technical University of Munich
 Stefan Leutenegger

 All rights reserved.

 This is the Author's implementation of BRISK: Binary Robust Invariant 
 Scalable Keypoints [1]. Various (partly unpublished) extensions are provided, 
 some of which are described in [2].

 [1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary 
     Robust Invariant Scalable Keypoints. In Proceedings of the IEEE 
     International Conference on Computer Vision (ICCV), 2011.

 [2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for 
     Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.
 
 This file is part of BRISK.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BRISK_BRISK_FEATURE_DETECTOR_H_
#define BRISK_BRISK_FEATURE_DETECTOR_H_

#include <vector>
#include <agast/wrap-opencv.h>

#include <brisk/internal/macros.h>

namespace brisk {
#if CV_MAJOR_VERSION < 3
class BriskFeatureDetector : public cv::FeatureDetector {
#else
class BriskFeatureDetector : public cv::Feature2D {
#endif
 public:
  BriskFeatureDetector(int threshold, int octaves = 3,
                       bool suppressScaleNonmaxima = true);
  virtual ~BriskFeatureDetector() { }

  void ComputeScale(const cv::Mat& image,
                    std::vector<cv::KeyPoint>& keypoints) const;
#if CV_MAJOR_VERSION >= 3
  void detect(cv::InputArray images,
              std::vector<cv::KeyPoint>& keypoints,
              cv::InputArray mask = cv::noArray()) override {
    detectImpl(images.getMat(), keypoints, mask.getMat());
  }
#endif
protected:
#if CV_MAJOR_VERSION < 3
  void detectImpl(const cv::Mat& image,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat& mask = cv::Mat()) const override;
#else
  void detectImpl(const cv::Mat& image,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat& mask = cv::Mat()) const;
#endif

  int _threshold;
  int _octaves;
  bool _suppressScaleNonmaxima;
};
}  // namespace brisk

#endif  // BRISK_BRISK_FEATURE_DETECTOR_H_
