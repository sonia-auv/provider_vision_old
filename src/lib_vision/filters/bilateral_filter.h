#ifndef VISION_FILTER_BILATERALFILTER_H_
#define VISION_FILTER_BILATERALFILTER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
//#include <opencv2/imgproc/imgproc.hpp>

namespace vision_filter {

//==============================================================================
// C L A S S E S

    class BilateralFilter : public Filter {
    public:
      //============================================================================
      // C O N S T R U C T O R S   A N D   D E S T R U C T O R

      explicit BilateralFilter(const GlobalParamHandler &globalParams)
              : Filter(globalParams),
                _enable("Enable", false, parameters_),
                _diameter("Diameter", -100, 0, 100, parameters_),
                _sigma_color("Sigm_color", 0, 0, 300, parameters_),
                _sigma_space("Sigma_space", 0, 0, 300, parameters_) {
        setName("BilateralFilter");
      }

      virtual ~BilateralFilter() {}

      //============================================================================
      // P U B L I C   M E T H O D S

      virtual void execute(cv::Mat &image) {
        if (_enable()) {
          cv::Mat blurred;
          cv::bilateralFilter(image, blurred, _diameter(), _sigma_color(), _sigma_space() );

          blurred.copyTo(image);
        }
      }

    private:
      // Params
      BooleanParameter _enable;
      IntegerParameter _diameter, _sigma_color, _sigma_space;
    };

}  // namespace vision_filter

#endif  // VISION_FILTER_INRANGE_H_
