/**
 * @file RobustifiedRototranslation.cc RobustifiedRototranslation class implementation
 *
 * @author Krzysztof Szczęsny, Jan Twardowski
 * @date Created on: 14 March 2017
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

#include <iostream>
#include "RobustifiedRototranslation.h"
#include "internal.h"
#include "Scheduler.h"

using namespace Constants;
using namespace PublicStructs;
using namespace Structs;
using namespace EdgeAlgorithms;

RobustifiedRototranslation::RobustifiedRototranslation(uint32_t img_width, uint32_t img_height, const char *calibration_config, edge_algorithm_config_t edge_algorithm_config, const config_t *config) :
	scheduler_(nullptr)
{
	(void)img_width;
	(void)img_height;
	(void)calibration_config;
	(void)edge_algorithm_config;
	(void)config;
}

RobustifiedRototranslation::~RobustifiedRototranslation() {
	this->Stop();
	delete this->scheduler_;
}

bool RobustifiedRototranslation::NewFrame(uint8_t *img, const gps_t &gps) {
	(void)img;
	(void)gps;
	return true;
}

//RobustifiedRototranslation::GetFeedback() {}

void RobustifiedRototranslation::Stop(void) {
	
}

RobustifiedRototranslation::RobustifiedRototranslation(void) :
	scheduler_(nullptr)
{
	std::cout << "RR object created" << std::endl;
}
