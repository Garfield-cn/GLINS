/**
* @Function: Ambiguity Resolution
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/ambiguity.h"

#include "gici/gnss/gnss_common.h"
#include "gici/gnss/phaserange_error_sd.h"
#include "gici/gnss/ambiguity_error.h"

namespace gici {

// The default constructor
AmbiguityResolution::AmbiguityResolution(
    const AmbiguityResolutionOptions options,
    const std::shared_ptr<Graph>& graph_ptr) :
  options_(options), graph_ptr_(graph_ptr),
  cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
  huber_loss_function_ptr_(new ceres::HuberLoss(1))
{
  ambiguities_.push_back(std::vector<Spec>());
  ambiguity_pairs_.push_back(std::vector<BsdPair>());
  ambiguity_lane_pairs_.push_back(std::vector<LanePair>());
}

// The default destructor
AmbiguityResolution::~AmbiguityResolution()
{}

// Solve ambiguity at given epoch
bool AmbiguityResolution::solve(const BackendId& epoch_id, 
             const std::vector<BackendId>& ambiguity_ids,
             const std::pair<GnssMeasurement, GnssMeasurement>& measurements)
{
  // Prepare data -----------------------------------------------------------
  // Shift storage
  ambiguities_.push_back(std::vector<Spec>());
  ambiguity_pairs_.push_back(std::vector<BsdPair>());
  ambiguity_lane_pairs_.push_back(std::vector<LanePair>());
  if (ambiguities_.size() > 2) {
    ambiguities_.pop_front();
    ambiguity_pairs_.pop_front();
    ambiguity_lane_pairs_.pop_front();
  }
  other_parameters_.clear();

  // Get parameters
  for (size_t i = 0; i < ambiguity_ids.size(); i++) {
    uint64_t id = ambiguity_ids[i].asInteger();
    Spec ambiguity;
    ambiguity.id = ambiguity_ids[i];
    ambiguity.prn = ambiguity.id.gPrn();
    CHECK(graph_ptr_->parameterBlockExists(id));

    // parameter and residual block
    ambiguity.parameter_block = graph_ptr_->parameterBlockPtr(id);
    ambiguity.value = *ambiguity.parameter_block->parameters();
    Graph::ResidualBlockCollection residuals = graph_ptr_->residuals(id);
    CHECK(residuals.size() > 0);
    // Ideally, for a non-reference satellite ambiguity parameter block, it only 
    // conresponds to one phaserange error block
    if (residuals.size() == 1) 
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeError ||
          residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeErrorSD ||
          residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeErrorDD) {
        ambiguity.residual_block = residuals[r];
        break;
      }
    }

    // wavelength
    char system = ambiguity_ids[i].gSystem();
    std::string prn = ambiguity_ids[i].gPrn();
    int phase_id = ambiguity_ids[i].gPhaseId();
    for (auto obs : measurements.first.satellites.at(prn).observations) {
      auto& observation = obs.second;
      if (gnss_common::getPhaseID(
          system, obs.first, observation.wavelength) == phase_id) {
        ambiguity.wavelength = observation.wavelength;
      }
    }

    // elevation angle for reference satellite selection
    const Eigen::Vector3d& position = measurements.first.position;
    const Eigen::Vector3d& sat_position = 
      measurements.first.satellites.at(prn).sat_position;
    ambiguity.elevation = gnss_common::satelliteElevation(sat_position, position);

    // Collect all other parameter blocks connected to the phase residual block
    Graph::ParameterBlockCollection parameters = 
      graph_ptr_->parameters(ambiguity.residual_block.residual_block_id);
    for (size_t p = 0; p < parameters.size(); p++) {
      ambiguity.parameter_block_ids_connected.push_back(BackendId(parameters[p].first));
      if (parameters[p].second->typeInfo() == "AmbiguityParameterBlock") continue;
      bool found = false;
      for (size_t j = 0; j < other_parameters_.size(); j++) {
        if (other_parameters_[j].id.asInteger() == parameters[p].first) {
          found = true; break;
        }
      }
      if (!found) {
        Parameter other;
        other.id = BackendId(parameters[p].first);
        other.size = parameters[p].second->dimension();
        other.value = Eigen::Map<Eigen::VectorXd>(
          parameters[p].second->parameters(), other.size);
        if (other_parameters_.size() == 0) {
          other.covariance_start_index = 0;
        }
        else {
          Parameter& last_other = other_parameters_.back();
          other.covariance_start_index = 
            last_other.covariance_start_index + last_other.size;
        }
        other_parameters_.push_back(other);
      }
    }

    curAmbs().push_back(ambiguity);
  }

  // Get covariance
  std::vector<uint64_t> parameter_block_ids;
  for (size_t i = 0; i < curAmbs().size(); i++) {
    parameter_block_ids.push_back(curAmbs()[i].id.asInteger());
  }
  size_t others_size = 0;
  for (size_t i = 0; i < other_parameters_.size(); i++) {
    parameter_block_ids.push_back(other_parameters_[i].id.asInteger());
    others_size += other_parameters_[i].size;
  }
  Eigen::MatrixXd covariance;
  graph_ptr_->computeCovariance(parameter_block_ids, covariance);
  ambiguity_covariance_ = covariance.topLeftCorner(curAmbs().size(), curAmbs().size());
  other_parameters_covariance_ = covariance.bottomRightCorner(others_size, others_size);
  ambiguity_others_covariance_ = covariance.topRightCorner(curAmbs().size(), others_size);

  // Sovle ambiguity -----------------------------------------------------
  // Apply Between-Satellite-Difference (BSD) and lane combination
  formSatellitePair();

  // Solve ambiguities
  bool ret_uwl_and_wl = false;
  if (!solveLanes({LaneType::UWL})) {
    // If UWL fix failed (maybe because of less satellite), we type to solve them
    // together with WL
    ret_uwl_and_wl = solveLanes({LaneType::UWL, LaneType::WL});

    // if still failed, we try only with WL
    if (!ret_uwl_and_wl) {
      ret_uwl_and_wl = solveLanes({LaneType::WL});
    }
  }
  else {
    ret_uwl_and_wl = solveLanes({LaneType::WL});
  }
  // If UWL and WL fixation successed, we fix narrow lane
  if (ret_uwl_and_wl) {
    if (!solveLanes({LaneType::NL})) return false;
  }
  // Or we check if there are no UWL or WL combinations, i.e. single frequency case
  else {
    int num_non_nl = 0;
    for (size_t i = 0; i < curAmbLanePairs().size(); i++) {
      if (curAmbLanePairs()[i].laneType() != LaneType::NL) {
        num_non_nl++;
      }
    }
    if (num_non_nl == 0) {
      if (!solveLanes({LaneType::NL})) return false;
    }
    else return false;
  }

  // Check ambiguity stability, if one ambiguity was fixed to the same value for given
  // epochs (see "min_consistant_fix_as_stable"), we consider it as stable. And then we 
  // constraint it on RTK estimator tightly.
  if (isFirstEpoch()) return true;
  for (size_t i = 0; i < curAmbLanePairs().size(); i++) {
    auto& lane_pair = curAmbLanePairs()[i];
    if (!lane_pair.is_fixed) continue;

    std::vector<LanePair> matches;
    std::vector<double> coefficients;
    if (!findMatch(lane_pair, matches, coefficients)) {
      lane_pair.num_consistant = 0;
      continue;
    }

    // compare ambiguity
    double matched_ambiguity = 0.0;
    for (size_t m = 0; m < matches.size(); m++) {
      matched_ambiguity += matches[m].value * coefficients[m];
    }
    double dcycle = (matched_ambiguity - lane_pair.value) / lane_pair.wavelength;
    // fixed to the same cycle
    if (fabs(dcycle) < 0.25) {
      if (matches.size() == 1) {
        lane_pair.num_consistant = matches[0].num_consistant + 1;
      }
      else {
        // we propagate the smaller one
        int last_num_consistant = matches[0].num_consistant > 
          matches[1].num_consistant ? matches[1].num_consistant : 
          matches[0].num_consistant;
        lane_pair.num_consistant = last_num_consistant + 1;
      }
    }
    // not the same, we think maybe both the last fixation and the current are not 
    // accurate. we delete the last fixation, and keep the current for further observing.
    else {
      for (size_t m = 0; m < matches.size(); m++) {
        graph_ptr_->removeResidualBlock(matches[m].residual_id);
      }
      lane_pair.num_consistant = 0;
    }

    // stable enough
    if (lane_pair.num_consistant >= options_.min_consistant_fix_as_stable) {
      CHECK(lane_pair.residual_id != nullptr);
      Graph::ResidualBlockSpec residual_block_spec = 
        graph_ptr_->residualBlockIdToResidualBlockSpecMap().at(lane_pair.residual_id);
      auto& base_ptr = residual_block_spec.error_interface_ptr;
      if (lane_pair.laneType() != LaneType::NL) {
        std::shared_ptr<AmbiguityError4Coef> ambiguity_error = 
          std::dynamic_pointer_cast<AmbiguityError4Coef>(base_ptr);
        CHECK(ambiguity_error != nullptr);
        ambiguity_error->setInformation(1e6);  // 0.001 cycle
      }
      else {
        std::shared_ptr<AmbiguityError2Coef> ambiguity_error = 
          std::dynamic_pointer_cast<AmbiguityError2Coef>(base_ptr);
        CHECK(ambiguity_error != nullptr);
        ambiguity_error->setInformation(1e6);  // 0.001 cycle
      }
    } 
  }

  return true;
}

// Apply Between-Satellite-Difference (BSD) and lane combination
void AmbiguityResolution::formSatellitePair()
{
  // Prepare data
  std::map<char, int> system_to_num_phases;
  std::multimap<char, double> system_to_wavelengths;
  std::map<std::string, int> prn_to_number_phases; 
  std::multimap<std::string, double> prn_to_wavelengths;
  std::multimap<std::string, Spec> prn_to_specs;
  std::map<uint64_t, size_t> id_to_spec_id;
  for (size_t i = 0; i < curAmbs().size(); i++) {
    Spec& ambiguity = curAmbs()[i];
    std::string prn = ambiguity.id.gPrn();

    auto it = prn_to_number_phases.find(prn);
    if (it == prn_to_number_phases.end()) {
      prn_to_number_phases.insert(std::make_pair(prn, 1));
    }
    else it->second++;
    prn_to_specs.insert(std::make_pair(prn, ambiguity));
    prn_to_wavelengths.insert(std::make_pair(prn, ambiguity.wavelength));
    id_to_spec_id.insert(std::make_pair(ambiguity.id.asInteger(), i));
  }
  for (size_t i = 0; i < GnssSystems.size(); i++) {
    char system = GnssSystems[i];

    system_to_num_phases.insert(std::make_pair(system, 0));
    for (auto it : prn_to_number_phases) {
      if (it.first[0] != system) continue;
      if (system_to_num_phases.at(system) < it.second) {
        system_to_num_phases.at(system) = it.second;
      }
    }

    for (auto it : prn_to_wavelengths) {
      if (it.first[0] != system) continue;
      if (system_to_wavelengths.find(system) == system_to_wavelengths.end()) {
        system_to_wavelengths.insert(std::make_pair(system, it.second));
      }
      bool found = false;
      for (auto it_wave = system_to_wavelengths.lower_bound(system); 
          it_wave != system_to_wavelengths.upper_bound(system); it_wave++) {
        if (it_wave->second == it.second) {
          found = true; break;
        }
      }
      if (!found) system_to_wavelengths.insert(std::make_pair(system, it.second));
    }
  }

  // Find reference satellites for each system and frequencies
  std::map<char, std::string> system_to_reference_prn;
  for (size_t i = 0; i < GnssSystems.size(); i++) {
    char system = GnssSystems[i];

    // check if we have a BDS-3 satellite
    bool only_use_bds3 = false;
    if (system == 'C') {
      for (size_t j = 0; j < curAmbs().size(); j++) {
        if (curAmbs()[j].prn[0] != 'C') continue;
        if (!gnss_common::isBds1(curAmbs()[j].prn) && 
            !gnss_common::isBds2(curAmbs()[j].prn)) {
          only_use_bds3 = true; break;
        }
      }
    }

    // find satellite with maximum elevation angle
    double max_elevation = 0.0;
    for (size_t j = 0; j < curAmbs().size(); j++) {
      Spec& ambiguity = curAmbs()[j];
      if (ambiguity.id.gSystem() != system) continue;

      // we try not to use BDS-1 or BDS-2 satellite
      if (only_use_bds3 && (gnss_common::isBds1(ambiguity.prn) || 
          gnss_common::isBds2(ambiguity.prn))) continue;

      // we only select satellites with max phase number
      if (prn_to_number_phases.at(ambiguity.id.gPrn()) != 
          system_to_num_phases.at(system)) continue;

      if (max_elevation < ambiguity.elevation) {
        system_to_reference_prn[system] = ambiguity.id.gPrn();
        max_elevation = ambiguity.elevation;
      }
    }
  }

  // Form BSD pair
  for (size_t i = 0; i < curAmbs().size(); i++) {
    Spec& ambiguity = curAmbs()[i];
    char system = ambiguity.id.gSystem();
    std::string prn = ambiguity.id.gPrn();
    std::string prn_ref = system_to_reference_prn.at(system);

    // do not use BDS-1 and BDS-2
    if (gnss_common::isBds1(ambiguity.prn) || gnss_common::isBds2(ambiguity.prn)) continue;

    if (!useSystem(system)) continue;
    if (prn == prn_ref) {
      ambiguity.is_reference = true;
      continue;
    }

    for (auto it = prn_to_specs.lower_bound(prn_ref); 
         it != prn_to_specs.upper_bound(prn_ref); it++) {
      Spec& ambiguity_ref = it->second;
      if (ambiguity_ref.id.gPhaseId() == ambiguity.id.gPhaseId()) {
        BsdPair ambiguity_pair(curAmbs(), i, id_to_spec_id.at(ambiguity_ref.id.asInteger()));
        curAmbPairs().push_back(ambiguity_pair);
        break;
      }
    }
  }

  // Widelane combination ------------------------------------------------
  // We set the phase with highest frequency as base, and sequentially apply dual-freqency 
  // widelane combination between two adjacent frequecies from the lowest frequecy to base.
  // Sort frequencies for each system
  std::map<char, double> system_to_base_wavelength;
  for (size_t i = 0; i < GnssSystems.size(); i++) {
    char system = GnssSystems[i];
    std::vector<double> wavelengths;
    auto it = system_to_wavelengths.lower_bound(system);
    if (it == system_to_wavelengths.end()) continue;
    for (; it != system_to_wavelengths.upper_bound(system); it++) {
      wavelengths.push_back(it->second);
    }
    if (wavelengths.size() == 0) continue;
    system_to_wavelengths.erase(system);
    std::sort(wavelengths.begin(), wavelengths.end());
    for (std::vector<double>::reverse_iterator wavelength = wavelengths.rbegin(); 
         wavelength != wavelengths.rend(); wavelength++) {
      system_to_wavelengths.insert(std::make_pair(system, *wavelength));
    }
    system_to_base_wavelength.insert(std::make_pair(system, wavelengths.front()));
  }

  // Form combinations
  for (size_t i = 0; i < curAmbPairs().size(); i++) {
    char system = curAmbs()[curAmbPairs()[i].spec_id].id.gSystem();
    std::string prn = curAmbs()[curAmbPairs()[i].spec_id].id.gPrn();

    // base frequency observation for NL AR
    if (curAmbPairs()[i].wavelength == system_to_base_wavelength.at(system)) {
      LanePair nl_pair(curAmbPairs(), i);
      curAmbLanePairs().push_back(nl_pair);
      curAmbPairs()[i].is_base_frequency = true;
    }

    // Widelanes
    double wavelength = curAmbPairs()[i].wavelength;
    // select the nearest frequency to combine
    double desired_wavelength_to_combine;
    for (auto it = system_to_wavelengths.lower_bound(system); 
         it != system_to_wavelengths.upper_bound(system); it++) {
      if (it->second == wavelength) {
        desired_wavelength_to_combine = (++it)->second;
        break;
      }
    } 
    for (size_t j = 0; j < curAmbPairs().size(); j++) {
      char system_j = curAmbs()[curAmbPairs()[j].spec_id].id.gSystem();
      if (system_j != system) continue;

      std::string prn_j = curAmbs()[curAmbPairs()[j].spec_id].id.gPrn();
      if (prn_j != prn) continue;
      
      double wavelength_j = curAmbPairs()[j].wavelength;
      if (wavelength_j != desired_wavelength_to_combine) continue;

      LanePair wl_pair(curAmbPairs(), j, i);
      curAmbLanePairs().push_back(wl_pair);
    }
  }
}

// Solve ambiguities
bool AmbiguityResolution::solveLanes(std::vector<LaneType> types)
{
  // Get combinations
  std::vector<LanePair> lane_pairs;
  for (size_t i = 0; i < curAmbLanePairs().size(); i++) {
    for (size_t j = 0; j < types.size(); j++) {
      if (curAmbLanePairs()[i].laneType() == types[j]) {
        lane_pairs.push_back(curAmbLanePairs()[i]);
      }
    }
  }

  // Get convergence judge
  double min_percentage_fixation = 0.0;
  for (size_t i = 0; i < types.size(); i++) {
    double percentage;
    if (types[i] == LaneType::UWL) {
      percentage = options_.min_percentage_fixation_uwl;
    }
    else if (types[i] == LaneType::WL) {
      percentage = options_.min_percentage_fixation_wl;
    }
    else if (types[i] == LaneType::NL) {
      percentage = options_.min_percentage_fixation_nl;
    }
    // get biggest one
    if (percentage > min_percentage_fixation) {
      min_percentage_fixation = percentage;
    }
  }

  // Sort them by elevation to apply partial AR latter
  std::sort(lane_pairs.begin(), lane_pairs.end(), [](LanePair lhs, LanePair rhs) 
    -> bool { return lhs.elevation > rhs.elevation; });

  // Get float ambiguities and its covariance
  Eigen::VectorXd float_ambiguities = Eigen::VectorXd::Zero(lane_pairs.size());
  Eigen::MatrixXd float_covariance = 
    Eigen::MatrixXd::Zero(lane_pairs.size(), lane_pairs.size());
  Eigen::MatrixXd differential_jacobian = 
    Eigen::MatrixXd::Zero(lane_pairs.size(), curAmbs().size());
  for (size_t i = 0; i < lane_pairs.size(); i++) {
    float_ambiguities(i) = lane_pairs[i].value / lane_pairs[i].wavelength;
    size_t id_higher_raw = curAmbPairs()[lane_pairs[i].bsd_pair_id_higher].spec_id;
    size_t id_higher_ref = curAmbPairs()[lane_pairs[i].bsd_pair_id_higher].spec_id_ref;

    if (lane_pairs[i].laneType() != LaneType::NL) {
      size_t id_lower_raw = curAmbPairs()[lane_pairs[i].bsd_pair_id_lower].spec_id;
      size_t id_lower_ref = curAmbPairs()[lane_pairs[i].bsd_pair_id_lower].spec_id_ref;
      double wave_higher = curAmbPairs()[lane_pairs[i].bsd_pair_id_higher].wavelength;
      double wave_lower = curAmbPairs()[lane_pairs[i].bsd_pair_id_lower].wavelength;
      differential_jacobian(i, id_higher_raw) = 1.0 / wave_higher;
      differential_jacobian(i, id_higher_ref) = -1.0 / wave_higher;
      differential_jacobian(i, id_lower_raw) = -1.0 / wave_lower;
      differential_jacobian(i, id_lower_ref) = 1.0 / wave_lower;
    }
    else {
      double wave = lane_pairs[i].wavelength;
      differential_jacobian(i, id_higher_raw) = 1.0 / wave;
      differential_jacobian(i, id_higher_ref) = -1.0 / wave;
    }
  }
  float_covariance = differential_jacobian * 
      ambiguity_covariance_ * differential_jacobian.transpose();

  // Sovle ambiguity by LAMBDA
  Eigen::VectorXd fixed_ambiguities;
  int num_active = float_ambiguities.size();
  int min_num_fixation = static_cast<int>(
    min_percentage_fixation * static_cast<double>(num_active));
  if (min_num_fixation < 5) min_num_fixation = 5;
  Eigen::VectorXd active_float_ambiguities = float_ambiguities;
  Eigen::MatrixXd active_float_covariance = float_covariance;
  // Try full AR and then partial AR, until it successed or active number 
  // of ambiguities reaches the minimum number of fixation judgement.
  while (num_active >= min_num_fixation) {
    if (gnss_common::solveAmbiguityLambda(active_float_ambiguities, 
        active_float_covariance, options_.ratio, fixed_ambiguities)) break;
    // reduce subsets
    --num_active;
    active_float_ambiguities = float_ambiguities.topRows(num_active);
    active_float_covariance = float_covariance.topLeftCorner(num_active, num_active);
  }
  if (num_active < min_num_fixation) return false;

  // Contraint the fixed ambiguities to temporary parameters and check 
  // phaserange residual. We test the residuals until no outlier is 
  // detected or the valid number of ambiguities reaches the minimum 
  // number of fixation judgement.
  int num_valid = num_active;
  std::vector<bool> is_reject;
  for (int i = 0; i < num_active; i++) is_reject.push_back(false);
  Eigen::VectorXd full_parameters_store;
  Eigen::MatrixXd full_covariance_store;
  while (num_valid >= min_num_fixation) {
    // parameters
    size_t ambiguity_size = ambiguity_covariance_.cols();
    size_t others_size = other_parameters_covariance_.cols();
    size_t full_size = ambiguity_size + others_size;
    Eigen::VectorXd full_parameters; full_parameters.resize(full_size);
    Eigen::MatrixXd full_covariance; full_covariance.resize(full_size, full_size);
    for (size_t i = 0; i < ambiguity_size; i++) {
      full_parameters(i) = curAmbs()[i].value;
    }
    for (size_t i = 0; i < other_parameters_.size(); i++) {
      size_t start = other_parameters_[i].covariance_start_index + ambiguity_size;
      size_t size = other_parameters_[i].size;
      full_parameters.middleRows(start, size) = other_parameters_[i].value;
    }
    full_covariance.topLeftCorner(
      ambiguity_size, ambiguity_size) = ambiguity_covariance_;
    full_covariance.bottomRightCorner(
      others_size, others_size) = other_parameters_covariance_;
    full_covariance.topRightCorner(
      ambiguity_size, others_size) = ambiguity_others_covariance_;
    full_covariance.bottomLeftCorner(
      others_size, ambiguity_size) = ambiguity_others_covariance_.transpose();

    // ambiguity measurements
    CHECK(fixed_ambiguities.size() == num_active);
    Eigen::MatrixXd jacobian; jacobian.resize(num_active, full_size);
    jacobian.leftCols(ambiguity_size) = differential_jacobian.topRows(num_active);
    jacobian.rightCols(others_size).setZero();
    Eigen::MatrixXd fix_ambiguity_covariance = 
      Eigen::MatrixXd::Identity(num_active, num_active) * 1e-4; // 0.01 cycles
    for (size_t i = 0; i < is_reject.size(); i++) {
      // we do not believe this ambiguity anymore
      if (is_reject[i]) fix_ambiguity_covariance(i, i) = 1e6;
    }

    // apply Kalman upadte to constraint the parameters
    Eigen::MatrixXd kalman_gain = full_covariance * jacobian.transpose() * (jacobian * 
      full_covariance * jacobian.transpose() + fix_ambiguity_covariance).inverse();
    full_parameters = full_parameters + kalman_gain * 
      (fixed_ambiguities - jacobian * full_parameters);
    full_covariance = (Eigen::MatrixXd::Identity(full_size, full_size) - 
      kalman_gain * jacobian) * full_covariance;
    full_covariance = (full_covariance + full_covariance.transpose()) / 2.0;

    // Store them
    full_parameters_store = full_parameters;
    full_covariance_store = full_covariance;
    
    // check phaserange residuals
    std::vector<double> normalized_residuals; 
    normalized_residuals.resize(ambiguity_size);
    for (size_t i = 0; i < curAmbs().size(); i++) {
      Spec& ambiguity = curAmbs()[i];
      // get parameters
      size_t num_parameter_block = ambiguity.parameter_block_ids_connected.size();

      // we did not put residual storages on base satellites side
      if (num_parameter_block == 0) continue;

      double **parameters_ptr = new double*[num_parameter_block];
      for (size_t p = 0; p < num_parameter_block; p++) {
        BackendId& id = ambiguity.parameter_block_ids_connected[p];
        if (id.type() == IdType::gAmbiguity) {
          for (size_t k = 0; k < curAmbs().size(); k++) {
            if (id == curAmbs()[k].id) {
              parameters_ptr[p] = new double[1]; 
              parameters_ptr[p][0] = full_parameters(k);
              break;
            }
          }
        }
        else {
          for (size_t k = 0; k < other_parameters_.size(); k++) {
            if (other_parameters_[k].id == id) {
              size_t size = other_parameters_[k].size;
              size_t start = ambiguity_size + other_parameters_[k].covariance_start_index;
              parameters_ptr[p] = new double[other_parameters_[k].size];
              memcpy(parameters_ptr[p], 
                    full_parameters.segment(start, size).data(), 
                    sizeof(double) * size);
              break;
            }
          }
        }
      }
      double residual = 0.0;
      ambiguity.residual_block.error_interface_ptr->EvaluateWithMinimalJacobians(
        parameters_ptr, &residual, nullptr, nullptr);
      normalized_residuals[i] = residual;

      for (size_t p = 0; p < num_parameter_block; p++) delete parameters_ptr[p];
      delete[] parameters_ptr;
    }

    // find outlier
    double normalized_residuals_median = vk::getMedian(normalized_residuals);
    for (size_t i = 0; i < normalized_residuals.size(); i++) {
      normalized_residuals[i] -= normalized_residuals_median;
    }
    bool found = false;
    std::vector<size_t> outlier_indexes;
    for (size_t i = 0; i < normalized_residuals.size(); i++) {
      // outlier detected
      if (fabs(normalized_residuals[i]) > options_.norm_phase_residual_reject_thres) {
        found = true;
        outlier_indexes.push_back(i);
        LOG(INFO) << "Outlier detected, residual = " << normalized_residuals[i];
      }
    }
    // no outlier found, break this process
    if (!found) break;

    // outlier found
    // Note that we reject only one ambiguity in each iteration, we iterate multiple
    // outliers here to in case the outlier does not have corresponding fixed ambiguity.
    int num_valid_store = num_valid;
    for (auto outlier_index : outlier_indexes) {
      if (num_valid_store > num_valid) break;

      Spec& ambiguity = curAmbs()[outlier_index];
      if (ambiguity.is_reference) {
        // it is a reference satellite, we reject all the ambiguity fixations of
        // this constellation
        // TODO: this is a big loss, maybe some better strategies can be applied here
        char system = ambiguity.id.gSystem();
        for (int i = 0; i < num_active; i++) {
          auto& pair = curAmbPairs()[lane_pairs[i].bsd_pair_id_higher];
          auto& spec = curAmbs()[pair.spec_id];
          if (spec.id.gSystem() == system) {
            is_reject[i] = true;
            num_valid--;
          }
        }
      }
      else {
        // find the corresponding BSD ambiguity pair
        BsdPair ambiguity_pair;
        for (size_t i = 0; i < curAmbPairs().size(); i++) {
          if (curAmbPairs()[i].spec_id == outlier_index) {
            ambiguity_pair = curAmbPairs()[i];
            break;
          }
        }
        if (ambiguity_pair.is_base_frequency) {
          // it is a base frequency, we reject all the ambiguity fixations of
          // this satellite
          std::string prn = ambiguity.id.gPrn();
          for (int i = 0; i < num_active; i++) {
            auto& pair = curAmbPairs()[lane_pairs[i].bsd_pair_id_higher];
            auto& spec = curAmbs()[pair.spec_id];
            if (spec.id.gPrn() == prn) {
              is_reject[i] = true;
              num_valid--;
            }
          }
        }
        else {
          // it is neither a reference satellite, nor a base frequency
          // we just reject itself
          std::string prn = ambiguity.id.gPrn();
          int phase_id = ambiguity.id.gPhaseId();
          found = false;
          for (int i = 0; i < num_active; i++) {
            auto& pair = curAmbPairs()[lane_pairs[i].bsd_pair_id_higher];
            auto& spec = curAmbs()[pair.spec_id];
            if (spec.id.gPrn() == prn && spec.id.gPhaseId() == phase_id) {
              is_reject[i] = true;
              num_valid--;
              found = true;
            }
          }
          if (!found) {
            // does not corresponds to any fixed ambiguities
            continue;
          }
        }
      }
    }
    // no correspondence, we think it equialents to no outliers
    if (num_valid_store == num_valid) {
      break;
    }
  }
  if (num_valid < min_num_fixation) return false;

  // Constraint ambiguities on global parameters
  auto& ambiguities = curAmbs();
  auto& ambiguity_pairs = curAmbPairs();
  auto& ambiguity_lane_pairs = curAmbLanePairs();
  for (size_t i = 0; i < ambiguities.size(); i++) {
    ambiguities[i].value = full_parameters_store(i);
  }
  for (size_t i = 0; i < ambiguity_pairs.size(); i++) {
    ambiguity_pairs[i].update(ambiguities);
  }
  for (size_t i = 0; i < ambiguity_lane_pairs.size(); i++) {
    ambiguity_lane_pairs[i].update(ambiguity_pairs);
  }
  // set fix flag
  for (int i = 0; i < num_active; i++) {
    if (is_reject[i]) continue;
    auto& fixed_lane_pair = lane_pairs[i];
    for (size_t j = 0; j < ambiguity_lane_pairs.size(); j++) {
      if (ambiguity_lane_pairs[j].bsd_pair_id_higher == 
          fixed_lane_pair.bsd_pair_id_higher && 
          ambiguity_lane_pairs[j].bsd_pair_id_lower == 
          fixed_lane_pair.bsd_pair_id_lower) {
        ambiguity_lane_pairs[j].is_fixed = true;
        if (ambiguity_lane_pairs[j].laneType() == LaneType::NL) {
          size_t id = ambiguity_lane_pairs[j].bsd_pair_id_higher;
          ambiguity_pairs[id].is_fixed = true;
        }
        break;
      }
    }
  }
  size_t ambiguity_size = ambiguity_covariance_.cols();
  size_t others_size = other_parameters_covariance_.cols();
  for (size_t i = 0; i < other_parameters_.size(); i++) {
    size_t start = ambiguity_size + other_parameters_[i].covariance_start_index;
    size_t size = other_parameters_[i].size;
    other_parameters_[i].value = full_parameters_store.middleRows(start, size);
  }
  ambiguity_covariance_ = 
    full_covariance_store.topLeftCorner(ambiguity_size, ambiguity_size);
  other_parameters_covariance_ = 
    full_covariance_store.bottomRightCorner(others_size, others_size);
  ambiguity_others_covariance_ = 
    full_covariance_store.topRightCorner(ambiguity_size, others_size);

  // Add a loosely constraint on RTK estimator, because we do not fully believe they
  // are accurate enough yet.
  const double information_loosely = 1e-2;   // 10 cycle
  for (size_t i = 0; i < ambiguity_lane_pairs.size(); i++) {
    if (!ambiguity_lane_pairs[i].is_fixed) continue;
    std::vector<double> coefficients;
    size_t id_higher_raw = 
      curAmbPairs()[ambiguity_lane_pairs[i].bsd_pair_id_higher].spec_id;
    size_t id_higher_ref = 
      curAmbPairs()[ambiguity_lane_pairs[i].bsd_pair_id_higher].spec_id_ref;

    if (ambiguity_lane_pairs[i].laneType() != LaneType::NL) {
      size_t id_lower_raw = 
        curAmbPairs()[ambiguity_lane_pairs[i].bsd_pair_id_lower].spec_id;
      size_t id_lower_ref = 
        curAmbPairs()[ambiguity_lane_pairs[i].bsd_pair_id_lower].spec_id_ref;
      double wave_higher = 
        curAmbPairs()[ambiguity_lane_pairs[i].bsd_pair_id_higher].wavelength;
      double wave_lower = 
        curAmbPairs()[ambiguity_lane_pairs[i].bsd_pair_id_lower].wavelength;
      double wave = ambiguity_lane_pairs[i].wavelength;
      coefficients.push_back(1.0 / wave_higher);
      coefficients.push_back(-1.0 / wave_higher);
      coefficients.push_back(-1.0 / wave_lower);
      coefficients.push_back(1.0 / wave_lower);
      // we use a small information, and a flat robust cost function
      std::shared_ptr<AmbiguityError4Coef> ambiguity_error = 
        std::make_shared<AmbiguityError4Coef>(
        ambiguity_lane_pairs[i].value / wave, information_loosely, coefficients);
      ambiguity_lane_pairs[i].residual_id = graph_ptr_->addResidualBlock(
        ambiguity_error,
        cauchy_loss_function_ptr_ ? cauchy_loss_function_ptr_.get() : nullptr,
        graph_ptr_->parameterBlockPtr(curAmbs()[id_higher_raw].id.asInteger()),
        graph_ptr_->parameterBlockPtr(curAmbs()[id_higher_ref].id.asInteger()),
        graph_ptr_->parameterBlockPtr(curAmbs()[id_lower_raw].id.asInteger()),
        graph_ptr_->parameterBlockPtr(curAmbs()[id_lower_ref].id.asInteger()));
    }
    else {
      double wave = ambiguity_lane_pairs[i].wavelength;
      coefficients.push_back(1.0 / wave);
      coefficients.push_back(-1.0 / wave);

      std::shared_ptr<AmbiguityError2Coef> ambiguity_error = 
        std::make_shared<AmbiguityError2Coef>(
        ambiguity_lane_pairs[i].value / wave, information_loosely, coefficients);
      ambiguity_lane_pairs[i].residual_id = graph_ptr_->addResidualBlock(
        ambiguity_error,
        cauchy_loss_function_ptr_ ? cauchy_loss_function_ptr_.get() : nullptr,
        graph_ptr_->parameterBlockPtr(curAmbs()[id_higher_raw].id.asInteger()),
        graph_ptr_->parameterBlockPtr(curAmbs()[id_higher_ref].id.asInteger()));
    }
  }

  return true;
}

// Search match on the last pairs
bool AmbiguityResolution::findMatch(
    LanePair& lane_pair,
    std::vector<LanePair>& matches,
    std::vector<double>& coefficients)
{
  if (!lane_pair.is_fixed) return false;

  matches.clear();
  coefficients.clear();

  auto& pair_higher = curAmbPairs()[lane_pair.bsd_pair_id_higher];
  auto& pair_lower = curAmbPairs()[lane_pair.bsd_pair_id_lower];
  auto& spec_higher_raw = curAmbs()[pair_higher.spec_id];
  auto& spec_higher_ref = curAmbs()[pair_higher.spec_id_ref];
  auto& spec_lower_raw = curAmbs()[pair_lower.spec_id];
  auto& spec_lower_ref = curAmbs()[pair_lower.spec_id_ref];
  char system = spec_higher_raw.id.gSystem();
  std::string prn_raw = spec_higher_raw.id.gPrn();
  std::string prn_ref = spec_higher_ref.id.gPrn();
  double wavelength = lane_pair.wavelength;
  double phase_id_higher = spec_higher_raw.id.gPhaseId();
  double phase_id_lower = spec_lower_raw.id.gPhaseId();
  auto& last_lane_pairs = lastAmbLanePairs();

  for (size_t i = 0; i < last_lane_pairs.size(); i++) {
    if (last_lane_pairs[i].wavelength != wavelength) continue;
    if (!last_lane_pairs[i].is_fixed) continue;

    auto& last_pair_higher = lastAmbPairs()[last_lane_pairs[i].bsd_pair_id_higher];
    auto& last_pair_lower = lastAmbPairs()[last_lane_pairs[i].bsd_pair_id_lower];
    auto& last_spec_higher_raw = lastAmbs()[last_pair_higher.spec_id];
    auto& last_spec_higher_ref = lastAmbs()[last_pair_higher.spec_id_ref];
    auto& last_spec_lower_raw = lastAmbs()[last_pair_lower.spec_id];
    auto& last_spec_lower_ref = lastAmbs()[last_pair_lower.spec_id_ref];
    char last_system = last_spec_higher_raw.id.gSystem();
    std::string last_prn_raw = last_spec_higher_raw.id.gPrn();
    std::string last_prn_ref = last_spec_higher_ref.id.gPrn();
    double last_phase_id_higher = last_spec_higher_raw.id.gPhaseId();
    double last_phase_id_lower = last_spec_lower_raw.id.gPhaseId();

    if (last_system != system) continue;

    if (lane_pair.laneType() != LaneType::NL) {
      if (last_phase_id_higher != phase_id_higher || 
          last_phase_id_lower != phase_id_lower) {
        continue;
      }
    }
    else {
      if (last_phase_id_higher != phase_id_higher) {
        continue;
      }
    }

    if (last_prn_raw == prn_raw && last_prn_ref == prn_ref) {
      matches.push_back(last_lane_pairs[i]);
      coefficients.push_back(1.0);
      return true;
    }

    // reference satellite changed
    if (prn_ref != last_prn_ref) {
      // last reference satellite is current raw satellite,
      // we find another satellite
      if (prn_raw == last_prn_ref && prn_ref == last_prn_raw) {
        matches.push_back(last_lane_pairs[i]);
        coefficients.push_back(-1.0);
        return true;
      }
      // not that case
      // we find all candidates with the satellite PRNs
      else {
        if (prn_raw == last_prn_raw) {
          matches.push_back(last_lane_pairs[i]);
          coefficients.push_back(1.0);
        }
        if (prn_ref == last_prn_raw) {
          matches.push_back(last_lane_pairs[i]);
          coefficients.push_back(-1.0);
        }
      }
    }
  }

  // Check candidates
  CHECK(matches.size() == coefficients.size());
  CHECK(matches.size() < 3);
  // we did not find the pair
  if (matches.size() < 2) return false;
  else return true;
}

// Check whether we use the system for ambiguity resolution
bool AmbiguityResolution::useSystem(const char system)
{
  auto it = std::find(options_.system_exclude.begin(), 
    options_.system_exclude.end(), system);
  if (it == options_.system_exclude.end()) return true;
  else return false;
}

// ---------------------------------------------------------
// Cycle slip detection
void cycleSlipDetection(GnssMeasurement& measurement_pre, 
                        GnssMeasurement& measurement_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre,
                        const Eigen::Vector3d position_cur)
{
  // Detect by LLI
  cycleSlipDetectionLLI(measurement_pre, measurement_cur);

  // Detect by MW
  cycleSlipDetectionMW(measurement_pre, measurement_cur, options.mw_slip_thres);

  // Detect by GF
  cycleSlipDetectionGF(measurement_pre, measurement_cur, options.gf_slip_thres);

  // Detect by relative position
  if (!(checkZero(position_pre) && checkZero(position_cur))) {
    // TODO
    // cycleSlipDetectionPosition(measurement_pre, measurement_cur,
    //     position_pre, position_cur, option_tools.?);
  }

  // Detect by time gap for single frequency
  cycleSlipDetectionTimeGap(measurement_pre, measurement_cur, options.period * 1.5);
}

// Cycle slip detection after single difference
void cycleSlipDetectionSD(GnssMeasurement& measurement_rov_pre, 
                        GnssMeasurement& measurement_ref_pre, 
                        GnssMeasurement& measurement_rov_cur,
                        GnssMeasurement& measurement_ref_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre,
                        const Eigen::Vector3d position_cur)
{
  // Detect by LLI
  cycleSlipDetectionLLI(measurement_rov_pre, measurement_rov_cur);
  cycleSlipDetectionLLI(measurement_ref_pre, measurement_ref_cur);

  // Apply single difference 
  GnssMeasurementSDIndexPairs pairs_pre = 
    gnss_common::formPhaserangeSDPair(measurement_rov_pre, measurement_ref_pre);
  GnssMeasurement measurement_sd_pre;
  measurement_sd_pre.timestamp = measurement_rov_pre.timestamp;
  for (size_t i = 0; i < pairs_pre.size(); i++) {
    Observation& observation_rov = measurement_rov_pre.getObs(pairs_pre[i].rov);
    Observation& observation_ref = measurement_ref_pre.getObs(pairs_pre[i].ref);
    Satellite& satellite_rov = measurement_rov_pre.getSat(pairs_pre[i].rov);
    double dpseudorange = observation_rov.pseudorange - observation_ref.pseudorange;
    double dphaserange = observation_rov.phaserange - observation_ref.phaserange;
    
    Observation observation_sd;
    observation_sd.pseudorange = dpseudorange;
    observation_sd.phaserange = dphaserange;
    observation_sd.wavelength = observation_rov.wavelength;
    observation_sd.slip = false;

    // force insert here
    std::string prn = pairs_pre[i].rov.prn;
    int code_type = pairs_pre[i].rov.code_type;
    measurement_sd_pre.satellites[prn].prn = satellite_rov.prn;
    measurement_sd_pre.satellites[prn].sat_type = satellite_rov.sat_type;
    measurement_sd_pre.satellites[prn].sat_position = satellite_rov.sat_position;
    measurement_sd_pre.satellites[prn].sat_clock = satellite_rov.sat_clock;
    measurement_sd_pre.satellites[prn].observations[code_type] = observation_sd;
    measurement_sd_pre.position.setZero();
  }

  GnssMeasurementSDIndexPairs pairs_cur = 
    gnss_common::formPhaserangeSDPair(measurement_rov_cur, measurement_ref_cur);
  GnssMeasurement measurement_sd_cur;
  measurement_sd_cur.timestamp = measurement_rov_cur.timestamp;
  for (size_t i = 0; i < pairs_cur.size(); i++) {
    Observation& observation_rov = measurement_rov_cur.getObs(pairs_cur[i].rov);
    Observation& observation_ref = measurement_ref_cur.getObs(pairs_cur[i].ref);
    Satellite& satellite_rov = measurement_rov_cur.getSat(pairs_cur[i].rov);
    double dpseudorange = observation_rov.pseudorange - observation_ref.pseudorange;
    double dphaserange = observation_rov.phaserange - observation_ref.phaserange;
    
    Observation observation_sd;
    observation_sd.pseudorange = dpseudorange;
    observation_sd.phaserange = dphaserange;
    observation_sd.wavelength = observation_rov.wavelength;
    observation_sd.slip = false;

    std::string prn = pairs_cur[i].rov.prn;
    int code_type = pairs_cur[i].rov.code_type;
    measurement_sd_cur.satellites[prn].prn = satellite_rov.prn;
    measurement_sd_cur.satellites[prn].sat_type = satellite_rov.sat_type;
    measurement_sd_cur.satellites[prn].sat_position = satellite_rov.sat_position;
    measurement_sd_cur.satellites[prn].sat_clock = satellite_rov.sat_clock;
    measurement_sd_cur.satellites[prn].observations[code_type] = observation_sd;
    measurement_sd_cur.position.setZero();
  }

  // Detect by GF
  cycleSlipDetectionGF(measurement_sd_pre, measurement_sd_cur, options.gf_sd_slip_thres);

  // Detect by time gap for single frequency
  cycleSlipDetectionTimeGap(measurement_sd_pre, measurement_sd_cur, options.period * 1.5);

  // Put slip flags
  for (size_t i = 0; i < pairs_cur.size(); i++) {
    Observation& observation_sd = measurement_sd_cur.getObs(pairs_cur[i].rov);
    Observation& observation_rov = measurement_rov_cur.getObs(pairs_cur[i].rov);
    Observation& observation_ref = measurement_ref_cur.getObs(pairs_cur[i].ref);
    observation_rov.slip |= observation_sd.slip;
    observation_ref.slip |= observation_sd.slip;
  }

  // Detect by relative position
  if (!(checkZero(position_pre) && checkZero(position_cur))) {
    // TODO
    // cycleSlipDetectionPosition(measurement_rov_pre, measurement_rov_cur,
    //     position_pre, position_cur, option_tools.?);
  }
}

// Cycle slip detection by Loss of Lock Indicator (LLI)
void cycleSlipDetectionLLI(GnssMeasurement& measurement_pre, 
                           GnssMeasurement& measurement_cur)
{
  for (auto& sat : measurement_cur.satellites) {
    for (auto& obs : sat.second.observations) {
      Observation& observation = obs.second;
      if (observation.LLI & 1) {
        observation.slip = true;
        continue;
      }

      // detect slip by parity unknown flag transition in LLI
      uint8_t LLI_cur = observation.LLI;
      auto it_sat = measurement_pre.satellites.find(sat.first);
      if (it_sat == measurement_pre.satellites.end()) continue;
      auto it_obs = it_sat->second.observations.find(obs.first);
      if (it_obs == it_sat->second.observations.end()) continue;
      uint8_t LLI_pre = it_obs->second.LLI;
      if (((LLI_pre & 2) && !(LLI_cur & 2)) || (!(LLI_pre & 2) && (LLI_cur & 2))) {
        observation.slip = true;
      }
    }
  }
}

// Cycle slip detection by Melbourne-Wubbena (MW) combination
void cycleSlipDetectionMW(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold)
{
  GnssMeasurementSDIndexPairs pairs = 
    gnss_common::formPhaserangeSDPair(measurement_pre, measurement_cur);

  // Find valid frequencies for each satellite
  std::vector<std::vector<int>> pair_indexes;
  std::string last_prn = "";
  for (size_t i = 0; i < pairs.size(); i++) {
    std::string prn = pairs[i].rov.prn;
    if (prn != last_prn) {
      std::vector<int> indexes; indexes.push_back(i);
      pair_indexes.push_back(indexes);
      last_prn = prn;
    }
    else {
      pair_indexes[pair_indexes.size() - 1].push_back(i);
    }
  }

  // Form MW combinations and detect
  for (size_t i = 0; i < pair_indexes.size(); i++) {
    if (pair_indexes[i].size() < 2) continue;

    for (size_t j = 1; j < pair_indexes[i].size(); j++) {
      GnssMeasurementIndex index_pre_0 = pairs[pair_indexes[i][0]].rov;
      GnssMeasurementIndex index_pre_1 = pairs[pair_indexes[i][j]].rov;
      GnssMeasurementIndex index_cur_0 = pairs[pair_indexes[i][0]].ref;
      GnssMeasurementIndex index_cur_1 = pairs[pair_indexes[i][j]].ref;
      Observation& observation_pre_0 = measurement_pre.getObs(index_pre_0);
      Observation& observation_pre_1 = measurement_pre.getObs(index_pre_1);
      Observation& observation_cur_0 = measurement_cur.getObs(index_cur_0);
      Observation& observation_cur_1 = measurement_cur.getObs(index_cur_1);

      double mw_pre = gnss_common::combinationMW(observation_pre_0, observation_pre_1);
      double mw_cur = gnss_common::combinationMW(observation_cur_0, observation_cur_1);

      if (fabs(mw_pre - mw_cur) > threshold) {
        observation_cur_0.slip = true;
        observation_cur_1.slip = true;
      }
    }
  }
}

// Cycle slip detection by Geometry-Free (GF) combination
void cycleSlipDetectionGF(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold)
{
  GnssMeasurementSDIndexPairs pairs = 
    gnss_common::formPhaserangeSDPair(measurement_pre, measurement_cur);

  // Find valid frequencies for each satellite
  std::vector<std::vector<int>> pair_indexes;
  std::string last_prn = "";
  for (size_t i = 0; i < pairs.size(); i++) {
    std::string prn = pairs[i].rov.prn;
    if (prn != last_prn) {
      std::vector<int> indexes; indexes.push_back(i);
      pair_indexes.push_back(indexes);
      last_prn = prn;
    }
    else {
      pair_indexes[pair_indexes.size() - 1].push_back(i);
    }
  }

  // Form MW combinations and detect
  for (size_t i = 0; i < pair_indexes.size(); i++) {
    if (pair_indexes[i].size() < 2) continue;

    for (size_t j = 1; j < pair_indexes[i].size(); j++) {
      GnssMeasurementIndex index_pre_0 = pairs[pair_indexes[i][0]].rov;
      GnssMeasurementIndex index_pre_1 = pairs[pair_indexes[i][j]].rov;
      GnssMeasurementIndex index_cur_0 = pairs[pair_indexes[i][0]].ref;
      GnssMeasurementIndex index_cur_1 = pairs[pair_indexes[i][j]].ref;
      Observation& observation_pre_0 = measurement_pre.getObs(index_pre_0);
      Observation& observation_pre_1 = measurement_pre.getObs(index_pre_1);
      Observation& observation_cur_0 = measurement_cur.getObs(index_cur_0);
      Observation& observation_cur_1 = measurement_cur.getObs(index_cur_1);

      double gf_pre = gnss_common::combinationGF(observation_pre_0, observation_pre_1);
      double gf_cur = gnss_common::combinationGF(observation_cur_0, observation_cur_1);

      if (fabs(gf_pre - gf_cur) > threshold) {
        observation_cur_0.slip = true;
        observation_cur_1.slip = true;
      }
    }
  }
}

// Cycle slip detection by relative position
void cycleSlipDetectionPosition(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          const Eigen::Vector3d position_pre,
                          const Eigen::Vector3d position_cur,
                          double threshold)
{
  // TODO: Detect by relative position
  // how to fix clock jump?
  return;
}

// Cycle slip detection by time gap for single frequency receiver
void cycleSlipDetectionTimeGap(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double max_time_gap)
{
#ifndef NDEBUG
  return; // disable in debug mode
#endif

  if (measurement_cur.timestamp - measurement_pre.timestamp < max_time_gap) {
    return;
  }

  // Check if single frequency
  std::vector<bool> is_single_frequency;
  for (auto sat : measurement_cur.satellites) {
    auto& satellite = sat.second;
    int num_phases = 0;
    for (auto obs : satellite.observations) {
      if (gnss_common::checkObservationValid(measurement_cur, 
          GnssMeasurementIndex(sat.first, obs.first), 
          ObservationType::Phaserange)) {
        num_phases++;
      }
    }
    if (num_phases > 1) is_single_frequency.push_back(false);
    else is_single_frequency.push_back(true);
  }

  // set slip flag
  size_t index = 0;
  for (auto& sat : measurement_cur.satellites) {
    if (is_single_frequency[index]) {
      auto& satellite = sat.second;
      for (auto& obs : satellite.observations) {
        obs.second.slip = true;
      }
    }
    index++;
  }
}

}