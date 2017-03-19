/**
 * @file common.h Definitions of structures that should be visible to user
 *
 * @author Krzysztof Szczęsny, Jan Twardowski
 * @date Created on: 16 March 2017
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

#ifndef COMMON_H_
#define COMMON_H_

/// @cond
#include <stdint.h>
/// @endcond

/**
 * @brief The type of every floating point variable.
 * Provides an easy way to switch all calculations to double precision.
 * @note When switching to double, various float literals should also be converted.
 * @note does it make any sense, considering that OpenCV was built for doubles? or was it?
 * http://answers.opencv.org/question/66830/how-to-use-floating-point-support-in-opencv/
 */
typedef float floating;

/// Optional library configuration parameters
/// @todo add other stuff (such as flags for denoising, stabilization, object recognition)
struct config_t {
	/// Initializes everything as falsy
	config_t(void);
	
	/**
	 * @brief Semi-compound initialization
	 * @param[in] camera_height_ assumed constant camera height above the ground [m]
	 */
	config_t(floating camera_height_):
		camera_height(camera_height_)
	{};
	
	floating camera_height; ///< Assumed constant camera height above the ground [m] (for scale estimation)
};

/// Processed (not raw) GPS information
/// @todo revise fields
struct gps_t {
	gps_t() = default; ///< Force POD
	
	bool valid; ///< Whether struct was filled with any data
	floating latitude; ///< Latitude coefficient
	floating longitude; ///< Longitude coeddificient
	floating quality; ///< GPS quality metric (HDOP). The less the better
};

/// Algorithms for edge detection and their config structures
namespace EdgeAlgorithms {
/// Algorithms for edge detection
enum edge_algorithm_t {
	EDGE_ALGORITHM_CANNY, ///< OpenCV Canny
};

/// Parameteres for cv::Canny
struct canny_params_t {
	floating threshold1; ///< First hysteresis threshold
	floating threshold2; ///< Second hysteresis threshold
	int apertureSize; ///< Sobel operator size (default: 3)
	bool L2gradient; ///< Whether to use L2 or L1 norm (default: false)
};

/// Union-like struct
struct edge_algorithm_config_t {
	edge_algorithm_t tag; ///< Algorithm choice
	union {
		canny_params_t canny_params; ///< Canny parameters
	}; ///< Algorithm parameters
};


} // namespace EdgeAlgorithms

#endif

