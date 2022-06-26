/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "view.h"

#include <iostream>

#include <util/common.h>

orthosfm::View::View(const unsigned int& id, const std::string& imagePath) {
    m_id = id;
    m_imagePath = boost::filesystem::absolute(imagePath).string();
    m_display_name = "[View " + zfill(m_id, 4) + "]";

    // Extract the image name
    if(!imagePath.empty()) {
        m_imageName = boost::filesystem::path(imagePath).stem().string();

        std::cout << m_display_name << ": " << m_imagePath << std::endl;
    }
}

void orthosfm::View::loadPixelData(const int& downscaleFactor) {
    // Load image data as colored image (3 Channel of type unsigned char)
    cv::Mat originalPixels = cv::imread(m_imagePath, cv::IMREAD_COLOR);

    cv::Size imgSize = cv::Size((int)((double)originalPixels.cols / (double)downscaleFactor), (int)((double)originalPixels.rows / (double)downscaleFactor));

    // Downscale
    cv::resize(originalPixels, m_pixels, imgSize, cv::INTER_LINEAR);

    // Set image width and size to local variables
    m_width = m_pixels.cols;
    m_height = m_pixels.rows;

    // Also load and downscale the mask
    if(!m_maskPath.empty()) {
        // Read in as grayscale
        cv::Mat originalMaskPixels = cv::imread(m_maskPath, cv::IMREAD_GRAYSCALE);

        // Resize
        cv::resize(originalMaskPixels, m_maskPixels, imgSize, cv::INTER_LINEAR);
    }

}

const int &orthosfm::View::getWidth() const {
    return m_width;
}

const int &orthosfm::View::getHeight() const {
    return m_height;
}

SiftData &orthosfm::View::getSiftData() {
    return m_siftData;
}

const cv::Mat &orthosfm::View::getPixels() const {
    return m_pixels;
}

const unsigned int &orthosfm::View::getID() const{
    return m_id;
}

const std::string &orthosfm::View::getDisplayName() const {
    return m_display_name;
}

const SiftData &orthosfm::View::getSiftData() const {
    return m_siftData;
}

const bool orthosfm::View::hasMask() const {
    return m_maskPixels.cols > 0 && m_maskPixels.rows > 0;
}

void orthosfm::View::findCorrespondingMask(const std::string &maskFolder) {
    // Get the file name of the image file (without extension)
    std::string filename = boost::filesystem::path(m_imagePath).stem().string();

    // Search for a mask with the pattern "{imageName}_mask.png"
    if(boost::filesystem::exists(maskFolder + "/" + filename + "_mask.png")) {
        m_maskPath = maskFolder + "/" + filename + "_mask.png";
    }
    else if(boost::filesystem::exists(maskFolder + "/" + filename + ".png")) {
        m_maskPath = maskFolder + "/" + filename + ".png";
    }
    else {
        std::cout << "WARNING: Failed to find the mask file '" << filename + "_mask.png' inside the masking folder" << std::endl;
    }
}

const bool orthosfm::View::isPixelMaskedIn(float x, float y) const {
    // Create point for lookup
    cv::Point point(
            (int)fmax(fmin(x, m_maskPixels.rows-1), 0),
            (int)fmax(fmin(y, m_maskPixels.cols-1), 0)
    );

    // Get the color at the pixel position
    uchar color = m_maskPixels.at<uchar>(point);

    // Check for brightness (16 was chosen somewhat arbitrary, as the image should only contain values of 0 and 255)
    return color > 16;
}

void orthosfm::View::setDimensions(int width, int height) {
    m_width = width;
    m_height = height;
}

const std::string &orthosfm::View::getImageName() const {
    return m_imageName;
}

const std::string &orthosfm::View::getImagePath() const {
    return m_imagePath;
}


