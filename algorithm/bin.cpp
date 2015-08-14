/**
 * \file	bin.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/bin.h>

//==============================================================
//
Bin::Bin(const cv::Mat binImage, const Features feature) : Features(feature) {
  ptr = this;

  bestNeuronalMatch = NOPE_NEURAL_OUTPUT;
  maxScoreIndex = -1;
  secondMaxScoreIndex = -1;
  maxScore = -10000.0;
  secondMaxScore = -10000.0;

  if (binImage.empty()) return;
  binaryBinImage = binImage;
  imageWidth = binImage.cols;
  imageHeight = binImage.rows;
  totalPixel = imageHeight * imageWidth;
  inputValues = NULL;
  outputValues = NULL;
  outputImageAsArray();
}

//==============================================================
//
Bin::~Bin() {}

//==============================================================
//
void Bin::outputImageAsArray() {
  inputValues = (float *)calloc(totalPixel, sizeof(float));
  int k = 0;

  for (int i = 0; i < imageHeight; i++) {
    float *ptr = binaryBinImage.ptr<float>(i);
    for (int j = 0; j < imageWidth; j++) {
      inputValues[k] = ptr[j];
      k++;
    }
  }
}

//==============================================================
//
Bin *Bin::GetPtr() { return ptr; }

//==============================================================
//
cv::Mat Bin::GetImage() { return this->binaryBinImage; }

//==============================================================
//
std::string Bin::GetBinName() { return bestNeuronalMatch; }

//==============================================================
//
float *Bin::GetNetworkValue() { return this->outputValues; }

//==============================================================
//
void Bin::OutputTrainingFrame(std::ofstream &trainningFile) {
  for (int i = 0; i < totalPixel; i++) {
    trainningFile << std::fixed << std::setprecision(7) << inputValues[i]
                  << " ";
  }
  trainningFile << "\n";
}

//==============================================================
//
void Bin::RunNeuralNetwork(struct fann *&neuralNet) {
  if (binaryBinImage.empty()) return;

  outputValues = fann_run(neuralNet, inputValues);

  for (int i = 0; i < NB_OUTPUT; ++i) {
    float score = outputValues[i];
    // printf("%f ", outputValues[i]);
    if (score > maxScore) {
      secondMaxScore = maxScore;
      maxScore = score;
      maxScoreIndex = i;
    } else if (score > secondMaxScore) {
      secondMaxScore = score;
      secondMaxScoreIndex = i;
    }
  }
  // std::cout << std::endl;

  EvalWeight();
}

//==============================================================
//
void Bin::EvalWeight() {
  // condition for hit score
  if (maxScore > 0.75f && maxScore - secondMaxScore > 0.5f) {
    switch (maxScoreIndex) {
      default:
        bestNeuronalMatch = NOPE_NEURAL_OUTPUT;
        break;
      case 0:
      case 1:
        bestNeuronalMatch = FIRST_NEURAL_OUTPUT;
        break;
      case 2:
      case 3:
        bestNeuronalMatch = SECOND_NEURAL_OUTPUT;
        break;
      case 4:
      case 5:
        bestNeuronalMatch = THIRD_NEURAL_OUTPUT;
        break;
      case 6:
        bestNeuronalMatch = FOUR_NEURAL_OUTPUT;
        break;
    }
  }
}
