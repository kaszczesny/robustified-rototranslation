/**
 * @file tests.cc Unit test framework
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
#include <sstream>
#include <vector>
#include <unistd.h>
#include "tests.h"

using namespace std;

static const char* tty_red = "\033[1;31m"; ///< enables bold red font
static const char* tty_green = "\033[1;32m"; ///< enables bold green font
static const char* tty_magenta = "\033[1;35m"; ///< enables bold magenta font
static const char* tty_normal = "\033[0m"; ///< return font to normal

/**
 * Performs unit tests that take no parameters.
 * @tparam arg type of the single argument that will be supplied to unit testing functions.
 * @param[in] title title of the test suite
 * @param[in] f_ptrs vector of pointer to unit tests
 * @param[in] f_args vector of arguments for f_ptrs - each entry is one argument for one call. Can be omitted.
 * @return false on any test failure
 */
template<typename arg> bool run_tests( const char* title, const vector< bool (*)(arg) >& f_ptrs,
	const vector<arg>& f_args = vector<arg>()
) {
	bool success = true, test_success;
	arg default_arg(0);
	stringstream ss; //so that internal OpenCV error messages don't interrupt this message

	cout << tty_magenta << "---=== " << title << " ===---" << tty_normal << endl;

	if (f_args.size() > 0 && f_args.size() != f_ptrs.size()) {
		cout << "  Function and argument mismatch!" << endl;
		return false;
	}

	for (unsigned int i = 0; i < f_ptrs.size(); i++) {
		ss << "  Test " << i+1 << "/" << f_ptrs.size() << ": ";

		try {
			if (f_args.size() > 0) {
				test_success = f_ptrs[i](f_args[i]);
			} else {
				test_success = f_ptrs[i](default_arg);
			}
		} catch (const std::exception& ex) {
			test_success = false;
			cout << ex.what() << flush;
		}

		if (test_success) {
			ss << "passed.";
			cout << tty_green << ss.str() << tty_normal << endl;
		} else {
			ss << "FAILED!";
			cout << tty_red << ss.str() << tty_normal << endl;
		}
		
		success &= test_success;
		ss.str(""); //clear
	}

	cout << endl;
	return success;
}

/**
 * The main function for unit testing.
 * Each unit test function must take 1 parameter and return a boolean:
 * false on test failure, true on success.
 * Similiar tests can be grouped together.
 * @return 0 if all tests were passed
 */
int main() {
	bool success = true;

	//don't use colors if not using terminal
	if (!isatty(STDOUT_FILENO)) {
		tty_red = tty_green = tty_magenta = tty_normal = "";
	}

	/* Unit test demonstration - 1 */
	vector< bool (*)(int) > int_funs { //functions
		dummy_is_even,
		dummy_is_even
	};
	vector<int> int_args {0, 2}; //and their arguments
	// vector<int> int_args {1, 2}; //this one will obviously not pass
	success &= run_tests<int>("Unit test demonstration - 1", int_funs, int_args);


	/* Unit test demonstration - 2 */
	vector< bool (*)(dummy_are_even_t) > dummy_are_even_t_funs {
		dummy_are_even,
		dummy_are_even,
		dummy_are_even
	};
	vector<dummy_are_even_t> dummy_are_even_t_args {{0, 2, "A message from the depths"}, {4, 6, ""}, {0, 0, "abc"}};
	success &= run_tests<dummy_are_even_t>("Unit test demonstration - 2", dummy_are_even_t_funs, dummy_are_even_t_args);


	/* OpenCV test - to check if everything is linked as expected */
	vector< bool (*)(void *) > void_funs {
		dummy_test
	};
	success &= run_tests< void * >("Dummy OpenCV tests", void_funs);


	cout << endl << (success ? tty_green : tty_red)
		<< (success ? "All tests were successful." : "Some tests have failed.") << tty_normal << endl;
	return !success;
}
