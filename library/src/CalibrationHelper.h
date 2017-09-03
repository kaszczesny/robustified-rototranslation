/**
 * @file CalibrationHelper.h CalibrationHelper class definition.
 * This header can be included by application using the library.
 *
 * @author Krzysztof Szczęsny, Jan Twardowski
 * @date Created on: 14 March 2017
 * @version 1.0
 * @copyright Copyright (c) 2017 Krzysztof Szczęsny, Jan Twardowski
 * @pre OpenCV 3.1
 * @pre C++11
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

#ifndef CALIBRATION_HELPER_H_
#define CALIBRATION_HELPER_H_

#include "common.h"

/**
 * Collects photos of checkerboard pattern photos, performs OpenCV calibration,
 * computes undistortion map and saves it along with intrinsic camera parameters.
 * @todo everything
 * @todo somehow facilitate nonparametric calibration?
 * @todo native resolution?
 */
class CalibrationHelper {
	public:
		/**
		 * Constructor.
		 * @note use case: pass configuration (pattern size, winsize, zerozone, FAST_CHECK etc
		 */
		CalibrationHelper();
		
		/**
		 * Dealloc everything.
		 */
		~CalibrationHelper();
		
	#ifndef TEST
	private:
	#endif
		
};

#endif
