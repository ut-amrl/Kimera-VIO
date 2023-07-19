/**
 * @file   KimeraVIOEvaluation.cpp
 * @brief  Evaluate Kimera VIO for ObVi-SLAM
 * @author Taijing Chen
 */

#include <future>
#include <memory>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"
#include "kimera-vio/dataprovider/KittiDataProvider.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"

DEFINE_int32(dataset_type, 0, "Type of parser to use:\n "
                              "0: Euroc \n 1: Kitti (not supported).");
DEFINE_string(
    params_folder_path,
    "../params/Euroc",
    "Path to the folder containing the yaml files with the VIO parameters.");

int main(int argc, char* argv[]) {

  Eigen::Affine3d T 
      = Eigen::Translation3d(-0.11, 0.06, 0.68) 
        * Eigen::Quaterniond(0.491, -0.491, 0.509, -0.509);
  std::cout << T.rotation() << std::endl;
  std::cout << T.translation() << std::endl;
  exit(0);

  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse VIO parameters from gflags.
  VIO::VioParams vio_params(FLAGS_params_folder_path);

  // Build dataset parser.
  VIO::DataProviderInterface::Ptr dataset_parser = nullptr;
  switch (FLAGS_dataset_type) {
    case 0: {
      switch (vio_params.frontend_type_) {
        case VIO::FrontendType::kMonoImu: {
          dataset_parser =
              VIO::make_unique<VIO::MonoEurocDataProvider>(vio_params);
        } break;
        case VIO::FrontendType::kStereoImu: {
          dataset_parser = VIO::make_unique<VIO::EurocDataProvider>(vio_params);
        } break;
        default: {
          LOG(FATAL) << "Unrecognized Frontend type: "
                     << VIO::to_underlying(vio_params.frontend_type_)
                     << ". 0: Mono, 1: Stereo.";
        }
      }
    } break;
    case 1: {
      dataset_parser = VIO::make_unique<VIO::KittiDataProvider>();
    } break;
    default: {
      LOG(FATAL) << "Unrecognized dataset type: " << FLAGS_dataset_type << "."
                 << " 0: EuRoC, 1: Kitti.";
    }
  }
  CHECK(dataset_parser);

  VIO::Pipeline::Ptr vio_pipeline;

  switch (vio_params.frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline = VIO::make_unique<VIO::MonoImuPipeline>(vio_params);
    } break;
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline = VIO::make_unique<VIO::StereoImuPipeline>(vio_params);
    } break;
    default: {
      LOG(FATAL) << "Unrecognized Frontend type: "
                 << VIO::to_underlying(vio_params.frontend_type_)
                 << ". 0: Mono, 1: Stereo.";
    } break;
  }
}