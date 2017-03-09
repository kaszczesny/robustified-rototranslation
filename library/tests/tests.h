/**
 * @file tests.h Definitions of unit test functions
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

#ifndef TESTS_H
#define TESTS_H

bool dummy_test(void *);
bool dummy_test2(void *);
bool dummy_is_even(int n);
/**
 * @brief Example structure that could be passed into unit test function as argument
 */
struct dummy_are_even_t {
	/// There must be a constructor that takes an int (default_arg)
	dummy_are_even_t(int) {}; 
	/// Facilitates vector initialization
	dummy_are_even_t(int a, int b, const char* c): a(a), b(b), c(c) {};
	int a;
	int b;
	const char* c;
};
bool dummy_are_even(dummy_are_even_t s);

/**
 * @brief Example structure that could be passed into unit test function as argument but different
 */
struct dummy_addition_t {
	/// There must be a constructor that takes an int (default_arg)
	dummy_addition_t(int) {}; 
	/// Facilitates vector initialization
	dummy_addition_t(int a, int b, int c): a(a), b(b), c(c) {};
	int a;
	int b;
	int c;
};
bool dummy_addition(dummy_addition_t s);
#endif
