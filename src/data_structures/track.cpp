/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include <utility>

#include <data_structures/track.h>

void orthosfm::Track::add(Feature feature) {
    m_features.push_back(feature);
}

bool orthosfm::Track::contains(const Feature& feature) const {
    for(auto m_feature : m_features) {
        if(m_feature.viewID == feature.viewID && m_feature.localFeatureID == feature.localFeatureID) {
            return true;
        }
    }

    return false;
}

const orthosfm::Feature &orthosfm::Track::get(int i) const{
    return m_features[i];
}

const unsigned int orthosfm::Track::size() const {
    return m_features.size();
}

std::ostream &orthosfm::operator<<(std::ostream &os, const orthosfm::Track &track) {
    os << "Track ";
    for(int i=0; i<track.m_features.size(); i++) {
        os << "[view: " << track.m_features[i].viewID << "; feature: " << track.m_features[i].localFeatureID << "]";
        if(i < track.m_features.size() - 1) {
            os << " <--> ";
        }
    }


    return os;
}

bool orthosfm::Track::containsCamera(int id) const {
    for(int i=0; i<m_features.size(); i++) {
        if(m_features[i].viewID == id) {
            return true;
        }
    }
    return false;
}

const Eigen::Vector4d &orthosfm::Track::getPoint() const {
    return m_point;
}

Eigen::Vector4d &orthosfm::Track::getPoint() {
    return m_point;
}

void orthosfm::Track::setPoint(Eigen::Vector4d point) {
    m_point = std::move(point);
    m_hasPoint = true;
}

const bool orthosfm::Track::hasPoint() const{
    return m_hasPoint;
}

void orthosfm::Track::invalidatePoint() {
    m_hasPoint = false;
}

orthosfm::Feature &orthosfm::Track::get(int i) {
    return m_features[i];;
}

const Eigen::Vector2f orthosfm::Track::getFeaturePosition(int i) const{
    return Eigen::Vector2f(m_features[i].x, m_features[i].y);
}

bool orthosfm::Track::operator==(const Track &other) {
    if(this->m_features.size() != other.m_features.size()) {
        return false;
    }
    else {
        // Loop over all features
        for(int i=0; i<m_features.size(); i++) {
            if(m_features[i].globalFeatureID != other.m_features[i].globalFeatureID) {
                return false;
            }
        }
        return true;
    }
}

void orthosfm::printTrackOverview(const std::vector<Track> &tracks) {
    // Count how many tracks of which length we have
    std::vector<int> countedLengths(100);
    unsigned int highestLength = 0;
    for(int i=0; i<tracks.size(); i++) {
        if(tracks[i].size() < 100) {
            countedLengths[tracks[i].size()] += 1;
            if (tracks[i].size() > highestLength) {
                highestLength = tracks[i].size();
            }
        }
    }

    // Print
    std::cout << std::endl << "========================================" << std::endl;
    for(int i=2; i<=highestLength; i++) {
        std::cout << "- Found\t" << countedLengths[i] << " \ttracks with length " << i << std::endl;
    }
    std::cout << "========================================" << std::endl << std::endl;
}


