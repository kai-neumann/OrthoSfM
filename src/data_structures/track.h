/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#ifndef ORTHO_SFM_TRACK_H
#define ORTHO_SFM_TRACK_H

#include <vector>
#include <ostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace orthosfm
{

	struct Feature
	{
		unsigned int viewID; // The ID of the view on which this feature was observed
		unsigned int localFeatureID; // Local index inside the per-view feature array
		unsigned int globalFeatureID; // Global index over all features of all views
		float x; // Pixel coordinate
		float y; // Pixel coordinate
		unsigned int r = 0; // Red color channel
		unsigned int g = 0; // Green color channel
		unsigned int b = 0; // Blue color channel


		// Constructor without initializing color data
		Feature(unsigned int viewID, unsigned int localFeatureID, unsigned int globalFeatureID, float x, float y)
		{
			this->viewID = viewID;
			this->localFeatureID = localFeatureID;
			this->globalFeatureID = globalFeatureID;
			this->x = x;
			this->y = y;
		}

		// Constructor with initialization of color data
		Feature(unsigned int viewID, unsigned int localFeatureID, unsigned int globalFeatureID, float x, float y, cv::Mat pixelData)
		{
			this->viewID = viewID;
			this->localFeatureID = localFeatureID;
			this->globalFeatureID = globalFeatureID;
			this->x = x;
			this->y = y;

			// Create point for lookup
			cv::Point point(
				(int)fmax(fmin(x, pixelData.rows - 1), 0),
				(int)fmax(fmin(y, pixelData.cols - 1), 0)
			);

			// Get the color at the pixel position
			cv::Vec3b color = pixelData.at<cv::Vec3b>(point);

			// BGR
			b = color[0];
			g = color[1];
			r = color[2];
		}

	};

	class Track
	{
	/*
	 * A Track is a collection of features that correspond to the same real-world point
	 */
	 public:
		// Default constructor
		Track() = default;

		void add(Feature feature);
		bool contains(const Feature& feature) const;
		bool containsCamera(int id) const;
		const Feature& get(int i) const;
		Feature& get(int i);
		const Eigen::Vector2f getFeaturePosition(int i) const;
		const unsigned int size() const;

		// Getter and setter for the point
		const Eigen::Vector4d& getPoint() const;
		Eigen::Vector4d& getPoint();
		void setPoint(Eigen::Vector4d point);
		const bool hasPoint() const;
		void invalidatePoint();

		// Comparison operator
		bool operator==(const Track& other);

		// For easy printing
		friend std::ostream& operator<<(std::ostream& os, const Track& track);
	 private:
		// The features that are part of this track
		std::vector<orthosfm::Feature> m_features;

		// Store the current 3d point in homogenous coordinates in the format x,y,z,w
		Eigen::Vector4d m_point;

		// Store if this track has an assigned point. Initially set to false.
		bool m_hasPoint = false;
	};

	// Print an overview
	void printTrackOverview(const std::vector<Track>& tracks);

}

#endif //ORTHO_SFM_TRACK_H