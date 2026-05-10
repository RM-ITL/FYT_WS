// Copyright (c) 2010, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef CV_BRIDGE__IMAGE_ENCODINGS_HPP_
#define CV_BRIDGE__IMAGE_ENCODINGS_HPP_

#include <string>

namespace cv_bridge
{

/// \brief Names for the various color encodings.
///
/// These are merely convenience aliases for the corresponding OpenCV constants.
/// The convention is to use the OpenCV encoding name with underscores replaced by
/// camel case.
namespace image_encodings
{
const std::string BGR8 = "bgr8";
const std::string RGB8 = "rgb8";
const std::string MONO8 = "mono8";
const std::string BGR16 = "bgr16";
const std::string RGB16 = "rgb16";
const std::string MONO16 = "mono16";

const std::string BGRA8 = "bgra8";
const std::string RGBA8 = "rgba8";
const std::string BGRA16 = "bgra16";
const std::string RGBA16 = "rgba16";

// OpenCV CvMat types
const std::string TYPE_8UC1 = "8UC1";
const std::string TYPE_8UC2 = "8UC2";
const std::string TYPE_8UC3 = "8UC3";
const std::string TYPE_8UC4 = "8UC4";
const std::string TYPE_8SC1 = "8SC1";
const std::string TYPE_8SC2 = "8SC2";
const std::string TYPE_8SC3 = "8SC3";
const std::string TYPE_8SC4 = "8SC4";
const std::string TYPE_16UC1 = "16UC1";
const std::string TYPE_16UC2 = "16UC2";
const std::string TYPE_16UC3 = "16UC3";
const std::string TYPE_16UC4 = "16UC4";
const std::string TYPE_16SC1 = "16SC1";
const std::string TYPE_16SC2 = "16SC2";
const std::string TYPE_16SC3 = "16SC3";
const std::string TYPE_16SC4 = "16SC4";
const std::string TYPE_32SC1 = "32SC1";
const std::string TYPE_32SC2 = "32SC2";
const std::string TYPE_32SC3 = "32SC3";
const std::string TYPE_32SC4 = "32SC4";
const std::string TYPE_32FC1 = "32FC1";
const std::string TYPE_32FC2 = "32FC2";
const std::string TYPE_32FC3 = "32FC3";
const std::string TYPE_32FC4 = "32FC4";
const std::string TYPE_64FC1 = "64FC1";
const std::string TYPE_64FC2 = "64FC2";
const std::string TYPE_64FC3 = "64FC3";
const std::string TYPE_64FC4 = "64FC4";

/// \brief Returns true if the encoding is a color image.
bool isColor(const std::string & encoding);

/// \brief Returns true if the encoding is a mono image.
bool isMono(const std::string & encoding);

/// \brief Returns true if the encoding is a BGR color image.
bool isBGR(const std::string & encoding);

/// \brief Returns true if the encoding is a RGB color image.
bool isRGB(const std::string & encoding);

/// \brief Returns true if the encoding is a BGRA color image.
bool isBGRA(const std::string & encoding);

/// \brief Returns true if the encoding is a RGBA color image.
bool isRGBA(const std::string & encoding);

/// \brief Returns true if the encoding is 8-bit per channel.
bool is8bit(const std::string & encoding);

/// \brief Returns true if the encoding is 16-bit per channel.
bool is16bit(const std::string & encoding);

}  // namespace image_encodings

}  // namespace cv_bridge

#endif  // CV_BRIDGE__IMAGE_ENCODINGS_HPP_
