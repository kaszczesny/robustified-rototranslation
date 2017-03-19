/**
 * @file internal.h Definitions of structures and constants for internal usage
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

#ifndef INTERNAL_H_
#define INTERNAL_H_

#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include "common.h"

/// Various constants used throught library
namespace Constants {
	constexpr floating INITIAL_RHO = 1.f; ///< Inverse depth initial value
	constexpr floating INITIAL_SIGMA = 1e5f; ///< Inverse depth uncertainty initial value
}

/// Structures common for multiple library classes
namespace Structs {

/// All information about 1 frame that is held in frame buffer
/// @todo maybe would be nice to POD
struct frame_input_t {
	cv::Mat_<uint8_t> img; ///< The downscaled image
	gps_t gps; ///< World coordinates of the frame
	uint64_t timestamp; ///< frame creation timestamp [ns]
	uint32_t seq; ///< frame sequence number
};

/// Descriptor of a edge pixel, as in the paper "Realtime Edge Based Visual Odometry for a Monocular Camera"
struct keyline_t {
	keyline_t() = default; ///< Force POD
	
	floating q_x; ///< Subpixel x position in image
	floating q_y; ///< Subpixel y position in image
	
	floating m_x; ///< Edge's local third derivative vector x coefficient
	floating m_y; ///< Edge's local third derivative vector y coefficient
	
	floating rho; ///< Estimated inverse depth
	floating sigma; ///< Estimated inverse depth uncertainty
};

/// Everything squeezed out of a frame: list of edges (represented by keypoints vector), total and last translation & rotation
/// @todo preallocation?
/// @todo remove dimensions of there are assumptions about camera location
/// @todo how to initialize direction to be consistent with GPS
struct frame_output_t {
	std::vector<std::vector<keyline_t>> edges; ///< Edges consisting of pixels (keylines)
	
	cv::Vec<floating, 3> v; ///< Translation vector as of this frame
	cv::Vec<floating, 3> omega; ///< Rotation vector as of this frame
	cv::Matx<floating, 3, 3> R_omega; ///< Rotation matrix corresponding to the rotation vector
	
	cv::Vec<floating, 3> position; ///< Total camera position as of this frame
	cv::Vec<floating, 3> direction; ///< Total camera direction as of this frame

	/**
	 * @brief Updates R_omega, based on omega.
	 * @param[in] omega_ new rotation vector
	 */
	void UpdateR_omega(floating omega_);
};

} //namespace Structs

#endif

