/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_VIEW_H
#define ORTHO_SFM_VIEW_H

#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <cuda_sift/cudaSift.h>

namespace orthosfm
{
	class View
	{
	 public:
		explicit View(const unsigned int& id, const std::string& imagePath);

		void loadPixelData(const int& downscaleFactor);

		const int& getWidth() const;
		const int& getHeight() const;
		const unsigned int& getID() const;
		const std::string& getDisplayName() const;
		const std::string& getImageName() const;
		const std::string& getImagePath() const;

		SiftData& getSiftData();
		const SiftData& getSiftData() const;
		const cv::Mat& getPixels() const;

		void findCorrespondingMask(const std::string& maskFolder);
		const bool hasMask() const;
		const bool isPixelMaskedIn(float x, float y) const;

		void setDimensions(int width, int height);
	 private:
		// Main properties of the view
		unsigned int m_id;

		// The ID of the camera in the format "[ViewXXXX]"
		std::string m_display_name;
		std::string m_imageName;

		// Path to the actual image file
		std::string m_imagePath;
		std::string m_maskPath;

		// Pixel data
		cv::Mat m_pixels;
		cv::Mat m_maskPixels;

		// Image dimensions
		int m_width = 0;
		int m_height = 0;

		// Store SIFT Features (temporary only during extraction and matching phase)
		SiftData m_siftData;
	};
}

#endif //ORTHO_SFM_VIEW_H


