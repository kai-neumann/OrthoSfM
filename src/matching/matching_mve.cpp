/*
 * Copyright (c) 2022 Kai Alexander Neumann
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license. See the LICENSE.txt file for details.
*/

#include "matching_mve.h"

// MVE dependencies
#include "../mve/mve/scene.h"
#include "../mve/sfm/bundler_common.h"
#include "../mve/sfm/bundler_tracks.h"
#include "../mve/util/system.h"
#include "../mve/util/timer.h"
#include "../mve/sfm/bundler_features.h"
#include "../mve/sfm/feature_set.h"
#include "../mve/sfm/bundler_matching.h"
#include "../mve/mve/image.h"
#include "../mve/mve/image_base.h"
#include "../mve/mve/image_tools.h"
#include "../mve/mve/image_io.h"
#include "../mve/mve/image_exif.h"

#include "boost/filesystem.hpp"

struct AppSettings
{
    std::string scene_path;
    std::string original_name = "original";
    std::string undistorted_name = "undistorted";
    std::string exif_name = "exif";
    std::string prebundle_file = "prebundle.sfm";
    std::string survey_file;
    std::string log_file;
    int max_image_size = 6000000;
    bool lowres_matching = true;
    bool normalize_scene = false;
    bool skip_sfm = false;
    bool always_full_ba = false;
    bool fixed_intrinsics = false;
    //bool shared_intrinsics = false;
    bool intrinsics_from_views = false;
    int video_matching = 0;
    float track_error_thres_factor = 10.0f;
    float new_track_error_thres = 0.01f;
    int initial_pair_1 = -1;
    int initial_pair_2 = -1;
    int min_views_per_track = 3;
    bool cascade_hashing = true;
    bool verbose_ba = false;
};

#define RAND_SEED_MATCHING 0
#define RAND_SEED_SFM 0
#define THUMBNAIL_SIZE 50

mve::ByteImage::Ptr
load_8bit_image (std::string const& fname, std::string* exif)
{
    std::string lcfname(util::string::lowercase(fname));
    std::string ext4 = util::string::right(lcfname, 4);
    std::string ext5 = util::string::right(lcfname, 5);
    try
    {
        if (ext4 == ".jpg" || ext5 == ".jpeg")
            return mve::image::load_jpg_file(fname, exif);
        else if (ext4 == ".png" ||  ext4 == ".ppm"
                 || ext4 == ".tif" || ext5 == ".tiff")
            return mve::image::load_file(fname);
    }
    catch (...)
    { }

    return mve::ByteImage::Ptr();
}

/* ---------------------------------------------------------------- */

mve::RawImage::Ptr
load_16bit_image (std::string const& fname)
{
    std::string lcfname(util::string::lowercase(fname));
    std::string ext4 = util::string::right(lcfname, 4);
    std::string ext5 = util::string::right(lcfname, 5);
    try
    {
        if (ext4 == ".tif" || ext5 == ".tiff")
            return mve::image::load_tiff_16_file(fname);
        else if (ext4 == ".ppm")
            return mve::image::load_ppm_16_file(fname);
    }
    catch (...)
    { }

    return mve::RawImage::Ptr();
}

/* ---------------------------------------------------------------- */

mve::FloatImage::Ptr
load_float_image (std::string const& fname)
{
    std::string lcfname(util::string::lowercase(fname));
    std::string ext4 = util::string::right(lcfname, 4);
    std::string ext5 = util::string::right(lcfname, 5);
    try
    {
        if (ext4 == ".tif" || ext5 == ".tiff")
            return mve::image::load_tiff_float_file(fname);
        else if (ext4 == ".pfm")
            return mve::image::load_pfm_file(fname);
    }
    catch (...)
    { }

    return mve::FloatImage::Ptr();
}

/* ---------------------------------------------------------------- */

mve::ImageBase::Ptr
load_any_image (std::string const& fname, std::string* exif)
{
    mve::ByteImage::Ptr img_8 = load_8bit_image(fname, exif);
    if (img_8 != nullptr)
        return img_8;

    mve::RawImage::Ptr img_16 = load_16bit_image(fname);
    if (img_16 != nullptr)
        return img_16;

    mve::FloatImage::Ptr img_float = load_float_image(fname);
    if (img_float != nullptr)
        return img_float;

#pragma omp critical
    std::cerr << "Skipping file " << fname
              << ", cannot load image." << std::endl;
    return mve::ImageBase::Ptr();
}

template <class T>
typename mve::Image<T>::Ptr
downscale_image (typename mve::Image<T>::Ptr img, int downscaleFactor)
{
    for(int i=1; i<downscaleFactor; i++) {
        img = mve::image::rescale_half_size<T>(img);
    }
    return img;
}

/* ---------------------------------------------------------------- */

mve::ImageBase::Ptr
downscale_image (mve::ImageBase::Ptr image, int downscaleFactor)
{
    switch (image->get_type())
    {
        case mve::IMAGE_TYPE_FLOAT:
            return downscale_image<float>(std::dynamic_pointer_cast
                                                   <mve::FloatImage>(image), downscaleFactor);
        case mve::IMAGE_TYPE_UINT8:
            return downscale_image<uint8_t>(std::dynamic_pointer_cast
                                                     <mve::ByteImage>(image), downscaleFactor);
        case mve::IMAGE_TYPE_UINT16:
            return downscale_image<uint16_t>(std::dynamic_pointer_cast
                                                      <mve::RawImage>(image), downscaleFactor);
        default:
            break;
    }
    return mve::ImageBase::Ptr();
}

bool
has_jpeg_extension (std::string const& filename)
{
    std::string lcfname(util::string::lowercase(filename));
    return util::string::right(lcfname, 4) == ".jpg"
           || util::string::right(lcfname, 5) == ".jpeg";
}

template <typename T>
void
find_min_max_percentile (typename mve::Image<T>::ConstPtr image,
                         T* vmin, T* vmax)
{
    typename mve::Image<T>::Ptr copy = mve::Image<T>::create(*image);
    std::sort(copy->begin(), copy->end());
    *vmin = copy->at(copy->get_value_amount() / 10);
    *vmax = copy->at(9 * copy->get_value_amount() / 10);
}

mve::ByteImage::Ptr
create_thumbnail (mve::ImageBase::ConstPtr img)
{
    mve::ByteImage::Ptr image;
    switch (img->get_type())
    {
        case mve::IMAGE_TYPE_UINT8:
            image = mve::image::create_thumbnail<uint8_t>
                    (std::dynamic_pointer_cast<mve::ByteImage const>(img),
                     THUMBNAIL_SIZE, THUMBNAIL_SIZE);
            break;

        case mve::IMAGE_TYPE_UINT16:
        {
            mve::RawImage::Ptr temp = mve::image::create_thumbnail<uint16_t>
                    (std::dynamic_pointer_cast<mve::RawImage const>(img),
                     THUMBNAIL_SIZE, THUMBNAIL_SIZE);
            uint16_t vmin, vmax;
            find_min_max_percentile(temp, &vmin, &vmax);
            image = mve::image::raw_to_byte_image(temp, vmin, vmax);
            break;
        }

        case mve::IMAGE_TYPE_FLOAT:
        {
            mve::FloatImage::Ptr temp = mve::image::create_thumbnail<float>
                    (std::dynamic_pointer_cast<mve::FloatImage const>(img),
                     THUMBNAIL_SIZE, THUMBNAIL_SIZE);
            float vmin, vmax;
            find_min_max_percentile(temp, &vmin, &vmax);
            image = mve::image::float_to_byte_image(temp, vmin, vmax);
            break;
        }

        default:
            return mve::ByteImage::Ptr();
    }

    return image;
}

void
add_exif_to_view (mve::View::Ptr view, std::string const& exif)
{
    if (exif.empty())
        return;

    mve::ByteImage::Ptr exif_image = mve::ByteImage::create(exif.size(), 1, 1);
    std::copy(exif.begin(), exif.end(), exif_image->begin());
    view->set_blob(exif_image, "exif");
}

std::vector<orthosfm::Track> orthosfm::calculateTracksUsingMVE(const std::vector<orthosfm::View> &views, const std::string& projectFolder, int downscaleFactor) {
    std::cout << "Calculating matches using MVE" << std::endl;

    // Create setting instance
    AppSettings conf;
    // Set the scene path
    conf.scene_path = projectFolder + "/mve";
    std::string viewPath = conf.scene_path + "/views";

    // Create the scene folder
    if(!boost::filesystem::is_directory(conf.scene_path)) {
        boost::filesystem::create_directory(conf.scene_path);
    }
    if(!boost::filesystem::is_directory(viewPath)) {
        boost::filesystem::create_directory(viewPath);
    }

    double imageWidth = 0;

    // Create MVE Views from our own view data structure
    #pragma omp parallel for ordered schedule(dynamic,1)
    for(int i=0; i<views.size(); i++)
    {
        std::string fname = views[i].getImageName();
        std::string afname = views[i].getImagePath();

        std::string exif;
        mve::ImageBase::Ptr image = load_any_image(afname, &exif);
        if (image == nullptr)
            continue;

        /* Create view, set headers, add image. */
        mve::View::Ptr view = mve::View::create();
        view->set_id((int)views[i].getID());
        view->set_name(views[i].getImageName());

        /* Rescale and add original image. */
        int orig_width = image->width();
        image = downscale_image(image, downscaleFactor);
        imageWidth = image->width(); // Safe for later
        if (orig_width == image->width() && has_jpeg_extension(afname))
            view->set_image_ref(afname, "original");
        else
            view->set_image(image, "original");

        /* Add thumbnail for byte images. */
        mve::ByteImage::Ptr thumb = create_thumbnail(image);
        if (thumb != nullptr)
            view->set_image(thumb, "thumbnail");

        /* Add EXIF data to view if available. */
        add_exif_to_view(view, exif);

        /* Save view to disc. */
        std::string mve_fname = "view_" + util::string::get_filled(views[i].getID(), 4) + ".mve";
        #pragma omp critical
        std::cout << "Importing image: " << fname
                  << ", writing MVE view: " << mve_fname << "..." << std::endl;
        view->save_view_as(viewPath + "/" + mve_fname);

        // Free the view
        view->cache_cleanup();
        view->clear();
        view.reset();
    }

    /* Load scene. */
    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(conf.scene_path);
    }
    catch (std::exception &e) {
        std::cerr << "Error loading scene: " << e.what() << std::endl;
        return {};
    }

    /* Try to load the pairwise matching from the prebundle. */
    sfm::bundler::ViewportList viewports;
    sfm::bundler::PairwiseMatching pairwise_matching;

    std::cout << "Starting feature matching." << std::endl;
    util::system::rand_seed(RAND_SEED_MATCHING);
    /* Feature computation for the scene. */
    sfm::bundler::Features::Options feature_opts;
    feature_opts.image_embedding = conf.original_name;
    feature_opts.max_image_size = conf.max_image_size;
    feature_opts.feature_options.feature_types = sfm::FeatureSet::FEATURE_ALL;

    std::cout << "Computing image features..." << std::endl;
    {
        util::WallTimer timer;
        sfm::bundler::Features bundler_features(feature_opts);
        bundler_features.compute(scene, &viewports);

        std::cout << "Computing features took " << timer.get_elapsed() << " ms." << std::endl;
    }

    // Filter by color
    /*for (int viewID = 0; viewID < viewports.size(); viewID++) {
        // Create a list of all features that should be removed
        std::vector<int> indices_to_remove;

        // Loop over all features
        for (int featureID = 0; featureID < viewports.at(viewID).features.colors.size(); featureID++) {
            //std::cout << "Checking View " << viewID << "; Feature " << featureID << " ..." << std::endl;

            // Check for color
            if ((int)viewports.at(viewID).features.colors[featureID].maximum() < 16) {
                indices_to_remove.push_back(featureID);
            }

        }

        std::cout << "Found " << indices_to_remove.size() << " features to remove" << std::endl;

        std::sort(indices_to_remove.begin(), indices_to_remove.end());

        for (int i = 0; i < indices_to_remove.size(); i++) {
            std::cout << "Removing " << indices_to_remove[i] << std::endl;
        }

        for (int i = indices_to_remove.size() - 1; i > 0; i--) {

            viewports.at(viewID).features.colors.erase(viewports->at(viewID).features.colors.begin() + indices_to_remove[i]);
            viewports.at(viewID).features.positions.erase(viewports->at(viewID).features.positions.begin() + indices_to_remove[i]);
            viewports.at(viewID).features.sift_descriptors.erase(viewports->at(viewID).features.sift_descriptors.begin() + indices_to_remove[i]);
            viewports.at(viewID).features.surf_descriptors.erase(viewports->at(viewID).features.surf_descriptors.begin() + indices_to_remove[i]);
        }

        // Apply filtered
        //viewports->at(viewID).features = filtered;

        //std::cout << "Filtered " << filteredCount << " features" << std::endl;

        // Do a check
        for (int featureID = 0; featureID < viewports.at(viewID).features.colors.size(); featureID++) {
            // Check for color
            if ((int)viewports.at(viewID).features.colors[featureID].maximum() < 16) {
                std::cout << "View " << viewID << "; Feature " << featureID << " passed the threshold" << std::endl;
            }

        }

    }*/

    /* Exhaustive matching between all pairs of views. */
    sfm::bundler::Matching::Options matching_opts;
    //matching_opts.ransac_opts.max_iterations = 1000;
    matching_opts.ransac_opts.threshold = 0.0015; // Default 0.0015
    matching_opts.ransac_opts.verbose_output = false;
    matching_opts.use_lowres_matching = conf.lowres_matching;


    //matching_opts.match_num_previous_frames = conf.video_matching;
    //matching_opts.match_num_previous_frames = 2;

    matching_opts.min_matching_inliers = 30;
    matching_opts.min_feature_matches = 50;

    matching_opts.matcher_type = conf.cascade_hashing
                                 ? sfm::bundler::Matching::MATCHER_CASCADE_HASHING
                                 : sfm::bundler::Matching::MATCHER_EXHAUSTIVE;

    std::cout << "Performing feature matching..." << std::endl;
    {
        util::WallTimer timer;
        sfm::bundler::Matching bundler_matching(matching_opts);
        bundler_matching.init(&viewports);
        bundler_matching.compute(&pairwise_matching);
        std::cout << "Matching took " << timer.get_elapsed() << " ms." << std::endl;
    }

    if (pairwise_matching.empty()) {
        std::cerr << "Error: No matching image pairs. Exiting." << std::endl;
        return {};
    }


    /* Drop descriptors and embeddings to save memory. */
    scene->cache_cleanup();
    for (std::size_t i = 0; i < viewports.size(); ++i)
        viewports[i].features.clear_descriptors();

    /* Check if there are some matching images. */
    if (pairwise_matching.empty())
    {
        std::cerr << "No matching image pairs. Exiting." << std::endl;
        return {};
    }

    /* Start incremental SfM. */
    util::system::rand_seed(RAND_SEED_SFM);

    /* Compute connected feature components, i.e. feature tracks. */
    sfm::bundler::TrackList tracks;
    {
        sfm::bundler::Tracks::Options tracks_options;
        tracks_options.verbose_output = true;

        sfm::bundler::Tracks bundler_tracks(tracks_options);
        std::cout << "Computing feature tracks..." << std::endl;
        bundler_tracks.compute(pairwise_matching, &viewports, &tracks);
        std::cout << "Created a total of " << tracks.size()
                  << " tracks." << std::endl;
    }

    // Convert to tracks
    //std::cout << "Image width" << imageWidth << std::endl;
    std::vector<Track> outputTracks;
    for (int trackID = 0; trackID < tracks.size(); trackID++) {
        Track t;
        for (int featureID = 0; featureID < tracks[trackID].features.size(); featureID++) {
            math::Vec2f pos = viewports[tracks[trackID].features[featureID].view_id].features.positions[tracks[trackID].features[featureID].feature_id];

            t.add(Feature(tracks[trackID].features[featureID].view_id, tracks[trackID].features[featureID].feature_id,
                          32768*tracks[trackID].features[featureID].view_id + tracks[trackID].features[featureID].feature_id,
                          imageWidth*(pos[0]+0.5), imageWidth*(pos[1]+0.5)));
        }
        outputTracks.push_back(t);
    }

    // Free remaining data structures
    viewports.clear();
    scene.reset();

    return outputTracks;
}
