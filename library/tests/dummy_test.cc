/**
 * @file dummy_test.cc Dummy unit tests
 *
 * @author Krzysztof Szczęsny
 * @date Created on: 6 March 2017
 * @version 1.0
 * @copyright Copyright (c) 2017 Krzysztof Szczęsny, Jan Twardowski
 * @pre OpenCV 3.1
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

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tests.h"
#include "../src/RobustifiedRototranslation.h"

using namespace cv;

namespace UnitTests {

/**
 * Sanity check whether OpenCV works.
 * @param[in] arg unused
 * @return false on test failure
 */
bool dummy_test(void *arg) {
	(void)arg; //avoid unused argument error

	Mat im = imread("../tex/img/agh.jpg");
	Mat kernel = (Mat_<char>(2,2) <<
		1, 0,
		0, -1);

	filter2D(im, im, CV_8U, kernel);

#ifndef NO_DISPLAY
	imshow("im", im);
	waitKey(1500);
#endif
	
	Mat im_ref = imread("../data/dummy_test.png");

	return countNonZero(im != im_ref) == 0
		? true
		: false;
}

/**
 * Another OpenCV test.
 * @param[in] arg unused
 * @return false on test failure
 */
bool dummy_test2(void *arg) {
	(void)arg; //avoid unused argument error

	Mat im = imread("../tex/img/agh.jpg");
	Mat im2 = Mat::zeros( im.size(), CV_32FC1 );
	
	cvtColor(im, im, COLOR_BGR2GRAY);
	cornerHarris(im, im2, 5, 3, 0.04);

	normalize( im2, im2, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	convertScaleAbs( im2, im2 );

#ifndef NO_DISPLAY
	imshow("im", im2);
	waitKey(1500);
#endif
	
	Mat im_ref = imread("../data/dummy_test2.png");
	cvtColor(im_ref, im_ref, COLOR_BGR2GRAY);
	
	return countNonZero(im2 != im_ref) == 0
		? true
		: false;
}

/**
 * Tests whether librr is available (i.e. linked properly).
 * @param[in] arg unused
 * @return always true (this is compile-time test)
 */
bool library_dummy_test(void *arg) {
	(void)arg; //avoid unused argument error
	
	RobustifiedRototranslation rr;
	return true;
}

/**
 * A test to show how to use tests with arguments.
 * @param[in] n number to be tested
 * @return false if n is odd
 */
bool dummy_is_even(int n) {
	return n % 2 == 0;
}

/**
 * A test to show how to use tests with struct arguments.
 * @param[in] s structure to be tested
 * @return false if s.a or s.b are odd
 */
bool dummy_are_even(dummy_are_even_t s) {
	if (s.c && s.c[0]) {
		std::cout << s.c << std::endl;
	}
	return s.a % 2 == 0 && s.b % 2 == 0;
}

/**
 * A 2nd test to show how to use tests with struct arguments.
 * @param[in] s structure to be tested
 * @return false if s.a or s.b are not equal to s.c
 */
bool dummy_addition(dummy_addition_t s) {
	return s.a + s.b == s.c;
}

} //namespace UnitTests

