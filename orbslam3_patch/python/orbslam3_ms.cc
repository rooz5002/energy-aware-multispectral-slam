// Python binding of the multispectral ORB-SLAM3 build (RGB-D mode, no viewer).
//
// One call per frame: the controller's values are written to ORB_SLAM3::MSControl, the frame is tracked,
// and the per-modality statistics that the controller needs for the next frame are returned.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <string>

#include <MSControl.h>
#include <ORBextractor.h>
#include <System.h>

namespace py = pybind11;
using U8 = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;
using U16 = py::array_t<uint16_t, py::array::c_style | py::array::forcecast>;

class Slam
{
public:
    Slam(const std::string &vocabulary, const std::string &settings)
        : mSystem(vocabulary, settings, ORB_SLAM3::System::RGBD, false) {}

    // grey: HxW uint8 (RGB stream converted to grey); depth: HxW uint16 in depth-map-factor units;
    // thermal: HxW uint8 with the same size/intrinsics as grey, or None.
    py::dict track(U8 grey, U16 depth, py::object thermal, double timestamp, int nFeaturesRGB, int nFeaturesThermal,
                   int minKeyFrameInterval, bool localBA, int loopClosureMode, int localBAEvery)
    {
        if (grey.ndim() != 2 || depth.ndim() != 2)
            throw std::invalid_argument("grey and depth must be 2-D arrays");
        cv::Mat imGrey(grey.shape(0), grey.shape(1), CV_8UC1, (void *)grey.data());
        cv::Mat imDepth(depth.shape(0), depth.shape(1), CV_16UC1, (void *)depth.data());

        ORB_SLAM3::MSControl &ctrl = ORB_SLAM3::MSControl::Get();
        U8 thermalArray;
        ctrl.thermal = cv::Mat();
        if (!thermal.is_none())
        {
            thermalArray = thermal.cast<U8>();
            if (thermalArray.ndim() != 2 || thermalArray.shape(0) != grey.shape(0) || thermalArray.shape(1) != grey.shape(1))
                throw std::invalid_argument("thermal must be 2-D with the same shape as grey");
            ctrl.thermal = cv::Mat(thermalArray.shape(0), thermalArray.shape(1), CV_8UC1, (void *)thermalArray.data());
        }
        ctrl.nFeaturesRGB = nFeaturesRGB;
        ctrl.nFeaturesThermal = nFeaturesThermal;
        ctrl.minKeyFrameInterval = minKeyFrameInterval;
        ctrl.localBA = localBA;
        ctrl.localBAEvery = localBAEvery;
        ctrl.loopClosureMode = loopClosureMode;

        Sophus::SE3f Tcw;
        ORB_SLAM3::MSFrameStats s;
        {
            py::gil_scoped_release release;
            Tcw = mSystem.TrackRGBD(imGrey, imDepth, timestamp);
            s = mSystem.GetMSFrameStats();
        }
        ctrl.thermal = cv::Mat(); // the numpy buffer is only valid during this call

        py::array_t<float> pose({4, 4});
        Eigen::Matrix4f T = Tcw.matrix();
        auto p = pose.mutable_unchecked<2>();
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                p(r, c) = T(r, c);

        py::dict out;
        out["Tcw"] = pose;
        out["state"] = s.trackingState;
        out["n_keys_rgb"] = s.nKeys[ORB_SLAM3::MS_RGB];
        out["n_keys_thermal"] = s.nKeys[ORB_SLAM3::MS_THERMAL];
        out["n_inliers_rgb"] = s.nInliers[ORB_SLAM3::MS_RGB];
        out["n_inliers_thermal"] = s.nInliers[ORB_SLAM3::MS_THERMAL];
        out["keyframe"] = s.keyFrame;
        out["n_maps"] = s.nMaps;
        out["map_id"] = s.mapId;
        out["n_keyframes"] = s.nKeyFramesInMap;
        out["applied_n_features_rgb"] = s.appliedFeaturesRGB;
        out["applied_n_features_thermal"] = s.appliedFeaturesThermal;
        out["applied_min_keyframe_interval"] = s.appliedMinKeyFrameInterval;
        out["applied_local_ba"] = s.appliedLocalBA;
        out["applied_local_ba_every"] = s.appliedLocalBAEvery;
        out["applied_loop_closure_mode"] = s.appliedLoopClosureMode;
        return out;
    }

    void shutdown()
    {
        py::gil_scoped_release release;
        mSystem.Shutdown();
    }
    void saveTrajectoryTUM(const std::string &path) { mSystem.SaveTrajectoryTUM(path); }
    void saveTrajectoryAllMaps(const std::string &path) { mSystem.SaveTrajectoryAllMapsMS(path); }
    void saveKeyFrameTrajectoryTUM(const std::string &path) { mSystem.SaveKeyFrameTrajectoryTUM(path); }

private:
    ORB_SLAM3::System mSystem;
};

// Keypoints detected by ORB-SLAM3's own extractor on one 8-bit image (used for image statistics and for
// defining degradation severity); returns an (N, 2) array of pixel coordinates.
static py::array_t<float> orbKeypoints(U8 image, int nFeatures, float scaleFactor, int nLevels, int iniThFAST, int minThFAST)
{
    if (image.ndim() != 2)
        throw std::invalid_argument("image must be a 2-D uint8 array");
    cv::Mat im(image.shape(0), image.shape(1), CV_8UC1, (void *)image.data());
    ORB_SLAM3::ORBextractor extractor(nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);
    std::vector<cv::KeyPoint> keys;
    cv::Mat descriptors;
    std::vector<int> lapping = {0, 0};
    extractor(im, cv::Mat(), keys, descriptors, lapping);
    py::array_t<float> out({(py::ssize_t)keys.size(), (py::ssize_t)2});
    auto o = out.mutable_unchecked<2>();
    for (size_t i = 0; i < keys.size(); i++)
    {
        o(i, 0) = keys[i].pt.x;
        o(i, 1) = keys[i].pt.y;
    }
    return out;
}

PYBIND11_MODULE(orbslam3_ms, m)
{
    m.def("orb_keypoints", &orbKeypoints, py::arg("image"), py::arg("n_features") = 1200, py::arg("scale_factor") = 1.2f,
          py::arg("n_levels") = 8, py::arg("ini_th_fast") = 20, py::arg("min_th_fast") = 7);
    m.doc() = "Multispectral ORB-SLAM3 (RGB-D + thermal feature pooling, run-time control hooks)";
    m.attr("STATE_NOT_INITIALIZED") = 1;
    m.attr("STATE_OK") = 2;
    m.attr("STATE_RECENTLY_LOST") = 3;
    m.attr("STATE_LOST") = 4;
    m.attr("LOOP_CLOSURE_MODES") = py::dict(py::arg("full") = 0, py::arg("reduced") = 1, py::arg("off") = 2);
    py::class_<Slam>(m, "System")
        .def(py::init<const std::string &, const std::string &>(), py::arg("vocabulary"), py::arg("settings"))
        .def("track", &Slam::track, py::arg("grey"), py::arg("depth"), py::arg("thermal"), py::arg("timestamp"),
             py::arg("n_features_rgb"), py::arg("n_features_thermal"), py::arg("min_keyframe_interval") = 0,
             py::arg("local_ba") = true, py::arg("loop_closure_mode") = 0,
             py::arg("local_ba_every") = 1)
        .def("shutdown", &Slam::shutdown)
        .def("save_trajectory_tum", &Slam::saveTrajectoryTUM)
        .def("save_trajectory_all_maps", &Slam::saveTrajectoryAllMaps)
        .def("save_keyframe_trajectory_tum", &Slam::saveKeyFrameTrajectoryTUM);
}
