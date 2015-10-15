// file      Filter.h

#ifndef VISION_FILTER_FILTER_FACTORY_H_
#define VISION_FILTER_FILTER_FACTORY_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <string>

// All filter files are included here
#include <lib_vision/filters/blurr.h>
#include <lib_vision/filters/dilate.h>
#include <lib_vision/filters/erode.h>
#include <lib_vision/filters/mission_test_fake_string.h>
#include <lib_vision/filters/test_filter.h>
#include <lib_vision/filters/buoy_single.h>
#include <lib_vision/filters/morphology.h>
#include <lib_vision/filters/original_image.h>
#include <lib_vision/filters/scharr.h>
#include <lib_vision/filters/schar_adding.h>
#include <lib_vision/filters/stats_threshold.h>
#include <lib_vision/filters/subtract_all_planes.h>
#include <lib_vision/filters/threshold.h>
#include <lib_vision/filters/rotate.h>
#include <lib_vision/filters/fence_detector.h>
#include <lib_vision/filters/image_accumulator.h>
#include <lib_vision/filters/object_feature_calculator.h>
#include <lib_vision/filters/train_detector.h>
//#include <lib_vision/filters/lsd_filtering.h>
#include <lib_vision/filters/object_finder.h>
#include <lib_vision/filters/pipe_detector.h>
#include <lib_vision/filters/track_detector.h>
#include <lib_vision/filters/sobel.h>
#include <lib_vision/filters/delorean_detector.h>
#include <lib_vision/filters/submarine_frame_masker.h>
#include <lib_vision/filters/in_range.h>
#include <lib_vision/filters/convex_hull.h>
#include <lib_vision/filters/torpedoes_detector.h>
#include <lib_vision/filters/laplacian.h>
#include <lib_vision/filters/canny.h>
#include <lib_vision/filters/hough_line.h>
#include <lib_vision/filters/adaptive_threshold.h>
#include <lib_vision/filters/handle_detector.h>
#include <lib_vision/filters/white_noise_takedown.h>
#include <lib_vision/filters/bilateral_filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

// Class that provides an interface
// for the vision_filter project.
// It enables instantiation via a string
// and holds the list of all the filters.
class FilterFactory {
 public:
  // KEEPING A REFERENCE TO GlobalParamHandler. VERY IMPORTANT
  static Filter *createInstance(const std::string &name,
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

  static std::string GetFilterList() {
    return "Blurr;Dilate;Erode;MissionTestFakeString;TestFilter;"
           "BuoySingle;Morphology;OriginalImage;Scharr;ScharrAdding;"
           "StatsThreshold;SubtractAllPlanes;Threshold;BuoySingle;Rotate;"
           "FenceDetector;ImageAccumulator;ObjectFeatureCalculator;"
           "TrainDetector;ObjectFinder;PipeDetector;TrackDetector;Sobel;"
           "DeloreanDetector;SubmarineFrameMasker;InRange;ConvexHull;"
           "TorpedoesDetector;Laplacian;Canny;HoughLine;AdaptiveThreshold;"
           "HandleDetector;WhiteNoiseTakedown;BilateralFilter;";
  }
};

}  // namespace vision_filter

#endif  // VISION_FILTER_FILTER_FACTORY_H_
