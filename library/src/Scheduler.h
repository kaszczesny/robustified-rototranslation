/**
 * @file Scheduler.h Scheduler class definition
 *
 * @author Krzysztof Szczęsny, Jan Twardowski
 * @date Created on: 19 March 2017
 * @version 1.0
 * @copyright Copyright (c) 2017 Krzysztof Szczęsny, Jan Twardowski
 * @par License
 *
 * MIT License
 *
 * Copyright (c) 2017 Krzysztof Szczęsny, Jan Twardowski
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "internal.h"
#include "DataEater/EdgeExtractor.h"
#include "DataEater/EdgeTracker.h"
#include "DataEater/Mapper.h"
#include "DataEater/ScaleEstimator.h"

using namespace Constants;
using namespace Structs;
using namespace EdgeAlgorithms;

/**
 * @brief Buffer management, buffer scheduling
 * @todo everything
 */
class Scheduler {
	public:
		
		
		/// Halts threads operation (probably not resumeable afterwards)
		void Stop(void);

	#ifndef TEST
	private:
	#endif
		/**
		 * Loads the binary configuration file generated beforehead by CalibrationHelper
		 * @param[in] calibration_config path to the calbration file
		 */
		void LoadCalibrationConfig(const char *calibration_config);
		
		/// @name Submodules
		///@{
		DataEater::EdgeExtractor edge_extractor_;
		DataEater::EdgeTracker edge_tracker_;
		DataEater::Mapper mapper_;
		DataEater::ScaleEstimator scale_estimator_;
		///@}
		
		cv::Size_<uint32_t> size_; ///< Downscaled image size
		
		cv::Mat_<floating> undistort_map_; ///< Reverse distortion map
		/**
		 * Intrinsic parameters matrix:
		 * @verbatim
		 * [ fc_x    s   cc_x ]
		 * [   0   fc_y  cc_y ]
		 * [   0     0     1  ], where:
		 *   fc_k - focal length along axis k [px?]
		 *   cc_k - k-coordinate of principial point
		 *     s  - skew factor
		 * @endverbatim
		 */
		cv::Matx<floating, 3, 3> intrinsic_;
		
		frame_output_t *previous_frame_; ///< state extracted from previous frame
		frame_output_t *current_frame_; ///< state being extracted from current frame

		uint32_t frame_seq_id_; ///< current sequence id of frame_input
};

#endif
