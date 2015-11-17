/**
 * \file	filter_factory.cc
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Pierluc Bédard <pierlucbed@gmail.com>
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "lib_vision/filter_factory.h"
namespace lib_vision {
// KEEPING A REFERENCE TO GlobalParamHandler. VERY IMPORTANT
Filter *FilterFactory::createInstance(const std::string &name,
                                      const GlobalParamHandler &globalParams) {
  if (name == "Blurr") {
    return new Blurr(globalParams);
  }
  if (name == "Dilate") {
    return new Dilate(globalParams);
  }
  if (name == "Erode") {
    return new Erode(globalParams);
  }
  if (name == "MissionTestFakeString") {
    return new MissionTestFakeString(globalParams);
  }
  if (name == "TestFilter") {
    return new TestFilter(globalParams);
  }
  if (name == "BuoySingle") {
    return new BuoySingle(globalParams);
  }
  if (name == "Morphology") {
    return new Morphology(globalParams);
  }
  if (name == "OriginalImage") {
    return new OriginalImage(globalParams);
  }
  if (name == "Scharr") {
    return new Scharr(globalParams);
  }
  if (name == "ScharrAdding") {
    return new ScharrAdding(globalParams);
  }
  if (name == "StatsThreshold") {
    return new StatsThreshold(globalParams);
  }
  if (name == "SubtractAllPlanes") {
    return new SubtractAllPlanes(globalParams);
  }
  if (name == "Threshold") {
    return new Threshold(globalParams);
  }
  if (name == "BuoySingle") {
    return new BuoySingle(globalParams);
  }
  if (name == "Rotate") {
    return new Rotate(globalParams);
  }
  if (name == "FenceDetector") {
    return new FenceDetector(globalParams);
  }
  if (name == "ImageAccumulator") {
    return new ImageAccumulator(globalParams);
  }
  if (name == "ObjectFeatureCalculator") {
    return new ObjectFeatureCalculator(globalParams);
  }
  if (name == "TrainDetector") {
    return new TrainDetector(globalParams);
  }
  // if (name == "LSDFiltering") {
  //  return new LSDFiltering(globalParams);
  //}
  if (name == "ObjectFinder") {
    return new ObjectFinder(globalParams);
  }
  if (name == "PipeDetector") {
    return new PipeDetector(globalParams);
  }
  if (name == "TrackDetector") {
    return new TrackDetector(globalParams);
  }
  if (name == "Sobel") {
    return new Sobel(globalParams);
  }
  if (name == "DeloreanDetector") {
    return new DeloreanDetector(globalParams);
  }
  if (name == "SubmarineFrameMasker") {
    return new SubmarineFrameMasker(globalParams);
  }
  if (name == "InRange") {
    return new InRange(globalParams);
  }
  if (name == "ConvexHull") {
    return new ConvexHull(globalParams);
  }
  if (name == "TorpedoesDetector") {
    return new TorpedoesDetector(globalParams);
  }
  if (name == "Laplacian") {
    return new Laplacian(globalParams);
  }

  if (name == "Canny") {
    return new Canny(globalParams);
  }
  if (name == "HoughLine") {
    return new HoughLine(globalParams);
  }
  if (name == "AdaptiveThreshold") {
    return new AdaptiveThreshold(globalParams);
  }
  if (name == "HandleDetector") {
    return new HandleDetector(globalParams);
  }
  if (name == "WhiteNoiseTakedown") {
    return new WhiteNoiseTakedown(globalParams);
  }
  if (name == "BilateralFilter") {
    return new BilateralFilter(globalParams);
  }
  // Case were not found.
  return nullptr;
}

std::string FilterFactory::GetFilterList() {
  return "Blurr;Dilate;Erode;MissionTestFakeString;TestFilter;"
         "BuoySingle;Morphology;OriginalImage;Scharr;ScharrAdding;"
         "StatsThreshold;SubtractAllPlanes;Threshold;BuoySingle;Rotate;"
         "FenceDetector;ImageAccumulator;ObjectFeatureCalculator;"
         "TrainDetector;ObjectFinder;PipeDetector;TrackDetector;Sobel;"
         "DeloreanDetector;SubmarineFrameMasker;InRange;ConvexHull;"
         "TorpedoesDetector;Laplacian;Canny;HoughLine;AdaptiveThreshold;"
         "HandleDetector;WhiteNoiseTakedown;BilateralFilter;";
}

}  // namespace lib_vision
