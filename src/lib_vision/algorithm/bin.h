/**
 * \file	bin.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_BINS_H_
#define VISION_FILTER_BINS_H_

#include <opencv2/opencv.hpp>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/moments.h>
#include <lib_vision/algorithm/features.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <fann.h>

#define NOPE_NEURAL_OUTPUT "UNKNOWN"
#define FIRST_NEURAL_OUTPUT "gr_one"
#define SECOND_NEURAL_OUTPUT "gr_two"
#define THIRD_NEURAL_OUTPUT "gr_three"
#define FOUR_NEURAL_OUTPUT "gr_four"
#define NB_OUTPUT 7

class Bin : public Features {
 public:
  // Creates a bin object from the points it receive
  Bin(const cv::Mat binImage, const Features feature);

  ~Bin();

  typedef Bin *Ptr;

  // returns the instance's pointer
  Ptr GetPtr();

  // Returns the bin name
  std::string GetBinName();

  // return the bin image
  cv::Mat GetImage();

  float *GetNetworkValue();

  // Run the network
  void RunNeuralNetwork(struct fann *&neuralNet);

  // Run the trainning output
  void OutputTrainingFrame(std::ofstream &trainningFile);

 private:
  /// Methods to use when the bins are found and the members are sets
  void EvalWeight();

  void outputImageAsArray();

  Ptr ptr;

  cv::Mat binaryBinImage;
  int imageWidth, imageHeight, totalPixel;
  std::string bestNeuronalMatch;

  float *inputValues;
  float *outputValues;

  int maxScoreIndex;
  int secondMaxScoreIndex;
  float maxScore;
  float secondMaxScore;

  cv::Point2f originalCenter;
  cv::Size originalSize;
  float originalAngle;
};

#endif
