/**
* @Function: Ambiguity Resolution
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include "gici/gnss/gnss_types.h"
#include "gici/optimizer/estimator_types.hpp"
#include "gici/optimizer/graph.hpp"

namespace gici {

// Ambiguity resolution options
struct AmbiguityResolutionOptions {
  // Usage of satellite systems
  // Currently we do not support GLONASS ambiguity resolution
  std::vector<char> system_exclude = {'R'};

  // Percentage of narrow lane ambiguity fixation to consider as fixed solution
  double min_percentage_fixation_nl = 0.8;

  // Percentage of wide lane ambiguity fixation to consider as successed
  double min_percentage_fixation_wl = 0.8;

  // Percentage of ultra wide lane ambiguity fixation to consider as successed
  double min_percentage_fixation_uwl = 0.8;

  // Ambiguity fixation ratio for LAMBDA
  double ratio = 2.0;

  // Max normalized phase residual to reject fixed ambiguity
  double norm_phase_residual_reject_thres = 3.0;

  // Number of consistant fixed ambiguity to consider it as stable
  size_t min_consistant_fix_as_stable = 5;
};

// Ambiguity resolution (AR)
class AmbiguityResolution {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Combination types
  enum class LaneType {
    NL, WL, UWL
  };

  // Ambiguity 
  struct Spec {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    std::string prn;
    double value;  // in meter
    double wavelength;
    double elevation;
    std::shared_ptr<ParameterBlock> parameter_block;
    Graph::ResidualBlockSpec residual_block;
    std::vector<BackendId> parameter_block_ids_connected;
    bool is_reference = false;
  };

  // Between Satellite Difference (BSD) ambiguity pair
  struct BsdPair {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BsdPair() { }
    BsdPair(std::vector<Spec>& ambiguities, size_t i, size_t i_ref) : 
      spec_id(i), spec_id_ref(i_ref) {
      value = ambiguities[i].value - ambiguities[i_ref].value;
      wavelength = ambiguities[i].wavelength;
      elevation = ambiguities[i].elevation;
    }

    // update values after Spec change
    void update(std::vector<Spec>& ambiguities) {
      value = ambiguities[spec_id].value - ambiguities[spec_id_ref].value;
    }

    size_t spec_id;
    size_t spec_id_ref;
    double value;   // in meter
    double wavelength;
    double elevation; // elevation of the spec_id satellite
    bool is_fixed = false;
    bool is_base_frequency;
  };

  // Ultra-widelane (UWL), Widelane (WL) and Narrowlane (NL) ambiguity pair
  struct LanePair {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LanePair() { }
    // For UWL and WL
    LanePair(std::vector<BsdPair>& ambiguity_pairs, size_t i_higher, 
      size_t i_lower) : bsd_pair_id_higher(i_higher), 
      bsd_pair_id_lower(i_lower) {
      wavelength = 1.0 / (1.0 / ambiguity_pairs[i_higher].wavelength - 
                   1.0 / ambiguity_pairs[i_lower].wavelength);
      value = (ambiguity_pairs[i_higher].value / 
               ambiguity_pairs[i_higher].wavelength - 
               ambiguity_pairs[i_lower].value / 
               ambiguity_pairs[i_lower].wavelength) * 
               wavelength;
      elevation = ambiguity_pairs[i_higher].elevation;
    }
    // For NL (stand-alone frequency)
    LanePair(std::vector<BsdPair>& ambiguity_pairs, size_t i) :
      bsd_pair_id_lower(0), bsd_pair_id_higher(i) {
      wavelength = ambiguity_pairs[i].wavelength;
      value = ambiguity_pairs[i].value;
      elevation = ambiguity_pairs[i].elevation;
    }

    // Distinguish NL, WL and UWL
    LaneType laneType() {
      if (wavelength < 0.3) return LaneType::NL;
      else if (wavelength < 2.0) return LaneType::WL;
      else return LaneType::UWL;
    }

    // update values after Spec change
    void update(std::vector<BsdPair>& ambiguity_pairs) {
      if (bsd_pair_id_lower == 0 && laneType() == LaneType::NL) {
        value = ambiguity_pairs[bsd_pair_id_higher].value;
      }
      else {
        value = (ambiguity_pairs[bsd_pair_id_higher].value / 
                 ambiguity_pairs[bsd_pair_id_higher].wavelength - 
                 ambiguity_pairs[bsd_pair_id_lower].value / 
                 ambiguity_pairs[bsd_pair_id_lower].wavelength) * 
                 wavelength;
      }
    }

    size_t bsd_pair_id_lower;   // with lower frequency
    size_t bsd_pair_id_higher;  // with higher frequency
    double value;  // in meter
    double wavelength;
    double elevation;
    bool is_fixed = false;
    int num_consistant = 0;
    ceres::ResidualBlockId residual_id = nullptr;
  };

  // Other parameters
  struct Parameter {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    Eigen::VectorXd value;
    size_t size;
    size_t covariance_start_index;
  };

  AmbiguityResolution(const AmbiguityResolutionOptions options,
                      const std::shared_ptr<Graph>& graph_ptr);
  ~AmbiguityResolution();

  // Solve ambiguity at given epoch
  // It returns true only when enough number (see "min_num_fixation") of 
  // narrow lane ambiguities are fixed
  bool solve(const BackendId& epoch_id, 
             const std::vector<BackendId>& ambiguity_ids,
             const std::pair<GnssMeasurement, GnssMeasurement>& measurements);

private:
  // Apply Between-Satellite-Difference (BSD) and lane combination
  void formSatellitePair();

  // Solve ambiguities
  bool solveLanes(std::vector<LaneType> types);

  // Search match on the last pairs
  bool findMatch(LanePair& lane_pair,
                std::vector<LanePair>& matches,
                std::vector<double>& coefficients);

  // Check if it is the first epoch
  bool isFirstEpoch() { return ambiguities_.size() < 2; }

  // Check whether we use the system for ambiguity resolution
  bool useSystem(const char system);

  // Getters
  std::vector<Spec>& curAmbs() { return ambiguities_.back(); }
  std::vector<BsdPair>& curAmbPairs() { return ambiguity_pairs_.back(); }
  std::vector<LanePair>& curAmbLanePairs() { return ambiguity_lane_pairs_.back(); }
  std::vector<Spec>& lastAmbs() { 
    CHECK(ambiguities_.size() >= 2);
    return ambiguities_[ambiguities_.size() - 2]; 
  }
  std::vector<BsdPair>& lastAmbPairs() { 
    CHECK(ambiguity_pairs_.size() >= 2);
    return ambiguity_pairs_[ambiguity_pairs_.size() - 2]; 
  }
  std::vector<LanePair>& lastAmbLanePairs() { 
    CHECK(ambiguity_lane_pairs_.size() >= 2);
    return ambiguity_lane_pairs_[ambiguity_lane_pairs_.size() - 2]; 
  }

private:
  // Graph in RTK estimator class
  std::shared_ptr<Graph> graph_ptr_;

  // loss function
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // Ambiguity handles
  std::deque<std::vector<Spec>> ambiguities_;
  std::deque<std::vector<BsdPair>> ambiguity_pairs_;
  std::deque<std::vector<LanePair>> ambiguity_lane_pairs_;
  std::vector<Parameter> other_parameters_;

  Eigen::MatrixXd ambiguity_covariance_;
  Eigen::MatrixXd other_parameters_covariance_;
  Eigen::MatrixXd ambiguity_others_covariance_;

  // Options
  AmbiguityResolutionOptions options_;

};

// Common functions -------------------------------------------------------------
// Cycle slip detection
// We apply Loss of Lock Indicator (LLI) detection, Geometry-Free (GF) detection 
// and Melbourne-Wubbena (MW) detection in default. If relative position is given, 
// we apply relative position assisted single frequency cycle slip detection.
void cycleSlipDetection(GnssMeasurement& measurement_pre, 
                        GnssMeasurement& measurement_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d position_cur = Eigen::Vector3d::Zero());

// Cycle slip detection after single difference
void cycleSlipDetectionSD(GnssMeasurement& measurement_rov_pre, 
                        GnssMeasurement& measurement_ref_pre, 
                        GnssMeasurement& measurement_rov_cur,
                        GnssMeasurement& measurement_ref_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d position_cur = Eigen::Vector3d::Zero());

// Cycle slip detection by Loss of Lock Indicator (LLI)
void cycleSlipDetectionLLI(GnssMeasurement& measurement_pre, 
                           GnssMeasurement& measurement_cur);

// Cycle slip detection by Melbourne-Wubbena (MW) combination
void cycleSlipDetectionMW(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold);
                        
// Cycle slip detection by Geometry-Free (GF) combination
void cycleSlipDetectionGF(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold);

// Cycle slip detection by relative position
void cycleSlipDetectionPosition(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          const Eigen::Vector3d position_pre,
                          const Eigen::Vector3d position_cur,
                          double threshold);

// Cycle slip detection by time gap for single frequency receiver
void cycleSlipDetectionTimeGap(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double max_time_gap);

}