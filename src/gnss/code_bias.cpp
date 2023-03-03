/**
* @Function: Code bias handler
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/gnss/code_bias.h"

#include <vector>
#include <algorithm>
#include <Eigen/Core>

#include "gici/gnss/gnss_common.h"

namespace gici {

// Set default base frequencies if not setted
void CodeBias::setDefaultBase()
{
  mutex_.lock();
  if (bases_.find('G') == bases_.end()) {
    bases_.insert(std::make_pair('G', std::make_pair(CODE_L1W, CODE_L2W)));
  }
  if (bases_.find('R') == bases_.end()) {
    bases_.insert(std::make_pair('R', std::make_pair(CODE_L1P, CODE_L2P)));
  }
  if (bases_.find('E') == bases_.end()) {
    bases_.insert(std::make_pair('E', std::make_pair(CODE_L1C, CODE_L5Q)));
  }
  if (bases_.find('C') == bases_.end()) {
    bases_.insert(std::make_pair('C', std::make_pair(CODE_L6I, CODE_NONE)));
  }
  mutex_.unlock();
}

// Set Differential Code Bias (DCB)
void CodeBias::setDcb(const std::string prn, 
    const int code, const int code_base, 
    const double value)
{
  CHECK(code != code_base);
  CHECK(prn.size() == 3);

  // check if exist
  bool found = false;
  auto it_dcb = dcbs_.lower_bound(prn);
  for ( ;it_dcb != dcbs_.upper_bound(prn); it_dcb++) {
    if (it_dcb->second.code1 == code_base && 
        it_dcb->second.code2 == code) {
      found = true;
      break;
    }
  }
  
  // Add to handle
  if (found) {
    it_dcb->second.value = value;
  }
  else {
    Dcb dcb;
    dcb.code1 = code_base;
    dcb.code2 = code;
    dcb.value = value;
    dcbs_.insert(std::make_pair(prn, dcb));
  }
}

// Set Time Group Delay (TGD) or Inter-System Corrections (ISC)
void CodeBias::setTgdIsc(const std::string prn, 
  const TgdIscType type, const double value)
{
  // check if exist
  bool found = false;
  auto it_tgd = tgds_.lower_bound(prn);
  for ( ;it_tgd != tgds_.upper_bound(prn); it_tgd++) {
    if (it_tgd->second.type == type) {
      found = true;
      break;
    }
  }
  
  // Add to handle
  if (found) {
    it_tgd->second.value = value * CLIGHT;
  }
  else {
    TgdIsc tgd;
    tgd.type = type;
    tgd.value = value * CLIGHT;
    tgds_.insert(std::make_pair(prn, tgd));
  }
}

// Set zero-difference code bias
void CodeBias::setZdcb(const std::string prn, 
  const int code, const double value)
{
  CHECK(prn.size() == 3);

  // check if exist
  bool found = false;
  auto it_zdcb = zdcbs_.lower_bound(prn);
  for ( ;it_zdcb != zdcbs_.upper_bound(prn); it_zdcb++) {
    if (it_zdcb->second.code == code) {
      found = true;
      break;
    }
  }
  
  // Add to handle
  if (found) {
    it_zdcb->second.value = value;
  }
  else {
    Zdcb zdcb;
    zdcb.code = code;
    zdcb.value = value;
    zdcbs_.insert(std::make_pair(prn, zdcb));
  }
}

// Arrange DCBs, TGDs, and ISCs to base frequencies
void CodeBias::arrangeToBases()
{
  mutex_.lock();
  
  // Clear
  biases_.clear();
  biases_with_tgds_.clear();

  // Use ZDCBs or DCBs
  if (zdcbs_.size() != 0) {
    arrangeZdcb();
  }
  else {
    arrangeDcb();
  }

  // try to add some default DCBs
  trySetDefaultCodes(biases_);
  trySetDefaultCodes(biases_with_tgds_);

  // Try to add TGDs and ISCs, if redundant, do not use it
  arrangeTgd();

  // try to add some default DCBs
  trySetDefaultCodes(biases_with_tgds_);

  mutex_.unlock();

  biases_initialized_ = true;
}

// Get code bias correction
double CodeBias::getCodeBias(const std::string prn, 
  const int code, const bool use_tgd)
{
  // Initialization
  if (!biases_initialized_) {
    arrangeToBases();
  }

  // Get code bias
  mutex_.lock();
  if (!use_tgd) 
  {
    if (biases_.find(prn) != biases_.end()) {
      std::unordered_map<int, double> biases_sat = biases_.at(prn);
      if (biases_sat.find(code) != biases_sat.end()) {
        double bias = biases_sat.at(code);
        // we need check if it is zero (invalid) outside
        if (bias == 0.0) bias = 1e-4;
        mutex_.unlock();
        return bias;
      }
    }
  }
  else
  {
    if (biases_with_tgds_.find(prn) != biases_with_tgds_.end()) {
      std::unordered_map<int, double> biases_sat = biases_with_tgds_.at(prn);
      if (biases_sat.find(code) != biases_sat.end()) {
        double bias = biases_sat.at(code);
        // we need check if it is zero (invalid) outside
        if (bias == 0.0) bias = 1e-4;
        mutex_.unlock();
        return bias;
      }
    }
  }
  mutex_.unlock();

  return 0.0;
}

// Arrange DCBs to base frequencies
void CodeBias::arrangeDcb()
{
  // get all PRNs
  std::vector<std::string> prns;
  for (auto dcb : dcbs_) {
    if (prns.size() == 0 || prns.back() != dcb.first) {
      prns.push_back(dcb.first);
    }
  }

  // fill DCBs of every satellites
  for (auto prn : prns) {
    std::vector<int> codes;
    std::vector<Dcb> dcbs;
    for (auto dcb = dcbs_.lower_bound(prn); 
        dcb != dcbs_.upper_bound(prn); dcb++) {
      if (std::find(codes.begin(), codes.end(), dcb->second.code1) 
          == codes.end()) { 
        codes.push_back(dcb->second.code1);
      } 
      if (std::find(codes.begin(), codes.end(), dcb->second.code2) 
          == codes.end()) { 
        codes.push_back(dcb->second.code2);
      } 
      dcbs.push_back(dcb->second);
    }
    if (codes.size() == 0) continue;

    // apply a least-square to convert DCB to code biases
    Eigen::VectorXd x = Eigen::VectorXd::Zero(codes.size());
    Eigen::VectorXd z = Eigen::VectorXd::Zero(dcbs.size() + 1);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dcbs.size() + 1, codes.size());
    for (size_t i = 0; i < dcbs.size(); i++) {
      z(i) = dcbs[i].value;
      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == dcbs[i].code1) H(i, j) = -1.0;
        if (codes[j] == dcbs[i].code2) H(i, j) = 1.0;
      }
    }

    // set the base frequency as zero
    std::pair<int, int> base = bases_.at(prn[0]);
    z(dcbs.size()) = 0.0;
    // single frequency base
    if (base.second == CODE_NONE) {
      bool found = false;
      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == base.first) {
          H(dcbs.size(), j) = 1.0; found = true;
        }
      }
      if (!found) {
        LOG(INFO) << "Input DCBs for " << prn << " does not contain base code " 
          << gnss_common::codeTypeToRinexType(prn[0], base.first) << "!";
        return;
      }
    }
    // ionosphere-free combination base
    else {
      double f1 = gnss_common::codeToFrequency(prn[0], base.first);
      double f2 = gnss_common::codeToFrequency(prn[0], base.second);
      double c1 = square(f1) / (square(f1) - square(f2));
      double c2 = -square(f2) / (square(f1) - square(f2));
      bool found_first = false, found_second = false;;
      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == base.first) {
          H(dcbs.size(), j) = c1; found_first = true;
        }
        else if (codes[j] == base.second) {
          H(dcbs.size(), j) = c2; found_second = true;
        }
      }
      if (!found_first) {
        LOG(INFO) << "Input DCBs for " << prn << " does not contain base code " 
          << gnss_common::codeTypeToRinexType(prn[0], base.first) << "!";
        return;
      }
      if (!found_second) {
        LOG(INFO) << "Input DCBs for " << prn << " does not contain base code " 
          << gnss_common::codeTypeToRinexType(prn[0], base.second) << "!";
        return;
      }
    }

    // Check rank
    if (checkZero((H.transpose() * H).determinant())) {
      LOG(INFO) << "Input DCBs are not closed for " << prn 
                 << "! H = " << std::endl << H;
      return;
    }

    // solve
    x = (H.transpose() * H).inverse() * H.transpose() * z;
    
    // fill bias handle
    for (size_t i = 0; i < codes.size(); i++) {
      // TODO: segment fault here
      if (biases_.find(prn) == biases_.end()) {
        biases_.insert(std::make_pair(prn, std::unordered_map<int, double>()));
      }
      biases_.at(prn).insert(std::make_pair(codes[i], x(i)));

      if (biases_with_tgds_.find(prn) == biases_with_tgds_.end()) {
        biases_with_tgds_.insert(
          std::make_pair(prn, std::unordered_map<int, double>()));
      }
      biases_with_tgds_.at(prn).insert(std::make_pair(codes[i], x(i)));
    }
  }
}

// Arrange TGDs and ISCs to base frequencies
void CodeBias::arrangeTgd()
{
  // get all PRNs
  std::vector<std::string> prns;
  for (auto tgd : tgds_) {
    if (prns.size() == 0 || prns.back() != tgd.first) {
      prns.push_back(tgd.first);
    }
  }

  // fill TGDs of every satellites
  for (auto prn : prns) {
    // get DCB-derived biases
    std::unordered_map<int, double> code_to_bias;
    if (biases_with_tgds_.find(prn) != biases_with_tgds_.end()) {
      code_to_bias = biases_with_tgds_.at(prn);
    }
    std::vector<int> codes;
    for (auto it : code_to_bias) {
      codes.push_back(it.first);
    }
    size_t old_size = codes.size();
    
    // select unredundant TGDs
    std::vector<TgdIsc> tgds;
    for (auto tgd = tgds_.lower_bound(prn); 
        tgd != tgds_.upper_bound(prn); tgd++) {
      int code1, code2;
      bool has_new = false;
      if (tgd->second.type == TgdIscType::None) continue;
      else if (tgd->second.type == TgdIscType::GpsTgd) {
        code1 = CODE_L1W;
        code2 = CODE_L2W;
      }
      else if (tgd->second.type == TgdIscType::GlonassTgd) {
        code1 = CODE_L1P;
        code2 = CODE_L2P;
      }
      else if (tgd->second.type == TgdIscType::GalileoBgdE1E5a) {
        code1 = CODE_L1C;
        code2 = CODE_L5Q;
      }
      else if (tgd->second.type == TgdIscType::GalileoBgdE1E5b) {
        code1 = CODE_L1C;
        code2 = CODE_L7Q;
      }
      else if (tgd->second.type == TgdIscType::BdsTgdB1B3) {
        code1 = CODE_L2I;
        code2 = CODE_L6I;
      }
      else if (tgd->second.type == TgdIscType::BdsTgdB2B3) {
        code1 = CODE_L7I;
        code2 = CODE_L6I;
      }
      if (!checkExist(prn, code1, biases_with_tgds_) && 
          std::find(codes.begin(), codes.end(), code1) == codes.end()) {
        codes.push_back(code1); has_new = true;
      }
      if (!checkExist(prn, code2, biases_with_tgds_) && 
          std::find(codes.begin(), codes.end(), code2) == codes.end()) {
        codes.push_back(code2); has_new = true;
      }
      if (has_new) {
        tgds.push_back(tgd->second);
      }
    }
    if (codes.size() == 0) continue;

    // apply a least-square to convert tgd to code biases
    Eigen::VectorXd x = Eigen::VectorXd::Zero(codes.size());
    Eigen::VectorXd z = Eigen::VectorXd::Zero(old_size + tgds.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(old_size + tgds.size(), codes.size());
    size_t i = 0;
    for (auto it : code_to_bias) {
      z(i) = it.second;
      H(i, i) = 1.0;
      i++;
    }
    for (auto tgd : tgds) {
      z(i) = tgd.value;
      int code1, code2;
      double c1, c2;
      if (tgd.type == TgdIscType::GpsTgd) {
        code1 = CODE_L1W;
        code2 = CODE_L2W;
        double f1 = gnss_common::codeToFrequency(prn[0], CODE_L1W);
        double f2 = gnss_common::codeToFrequency(prn[0], CODE_L2W);
        double gamma = square(f1 / f2);
        c1 = -1.0 / (1.0 - gamma);
        c2 = 1.0 / (1.0 - gamma);
      }
      else if (tgd.type == TgdIscType::GlonassTgd) {
        code1 = CODE_L1P;
        code2 = CODE_L2P;
        c1 = 1.0;
        c2 = -1.0;
      }
      else if (tgd.type == TgdIscType::GalileoBgdE1E5a) {
        code1 = CODE_L1C;
        code2 = CODE_L5Q;
        double f1 = gnss_common::codeToFrequency(prn[0], CODE_L1C);
        double f2 = gnss_common::codeToFrequency(prn[0], CODE_L5Q);
        double gamma = square(f1 / f2);
        c1 = -1.0 / (1.0 - gamma);
        c2 = 1.0 / (1.0 - gamma);
      }
      else if (tgd.type == TgdIscType::GalileoBgdE1E5b) {
        code1 = CODE_L1C;
        code2 = CODE_L7Q;
        double f1 = gnss_common::codeToFrequency(prn[0], CODE_L1C);
        double f2 = gnss_common::codeToFrequency(prn[0], CODE_L7Q);
        double gamma = square(f1 / f2);
        c1 = -1.0 / (1.0 - gamma);
        c2 = 1.0 / (1.0 - gamma);
      }
      else if (tgd.type == TgdIscType::BdsTgdB1B3) {
        code1 = CODE_L2I;
        code2 = CODE_L6I;
        c1 = -1.0;
        c2 = 1.0;
      }
      else if (tgd.type == TgdIscType::BdsTgdB2B3) {
        code1 = CODE_L7I;
        code2 = CODE_L6I;
        c1 = -1.0;
        c2 = 1.0;
      }

      for (size_t j = 0; j < codes.size(); j++) {
        if (codes[j] == code1) H(i, j) = c1;
        if (codes[j] == code2) H(i, j) = c2;
      }
      i++;
    }

    // if no DCB-derived code biases, we add some constrains
    if (old_size == 0) {
      // set the base frequency as zero
      z.conservativeResize(z.rows() + 1);
      H.conservativeResize(H.rows() + 1, Eigen::NoChange);
      std::pair<int, int> base = bases_.at(prn[0]);
      z(z.rows() - 1) = 0.0;
      H.bottomRows(1).setZero();
      // single frequency base
      if (base.second == CODE_NONE) {
        bool found = false;
        for (size_t j = 0; j < codes.size(); j++) {
          if (codes[j] == base.first) {
            H(z.rows() - 1, j) = 1.0; found = true;
          }
          else H(z.rows() - 1, j) = 0.0;
        }
        if (!found) {
          LOG(INFO) << "Input TGDs does not contain base code " 
            << gnss_common::codeTypeToRinexType(prn[0], base.first) << "!";
          return;
        }
      }
      // ionosphere-free combination base
      else {
        double f1 = gnss_common::codeToFrequency(prn[0], base.first);
        double f2 = gnss_common::codeToFrequency(prn[0], base.second);
        double c1 = square(f1) / (square(f1) - square(f2));
        double c2 = -square(f2) / (square(f1) - square(f2));
        bool found_first = false, found_second = false;
        for (size_t j = 0; j < codes.size(); j++) {
          if (codes[j] == base.first) {
            H(z.rows() - 1, j) = c1; found_first = true;
          }
          else if (codes[j] == base.second) {
            H(z.rows() - 1, j) = c2; found_second = true;
          }
        }
        if (!found_first) return;
        if (!found_second) return;
      }
    }

    // Check rank
    if (checkZero((H.transpose() * H).determinant())) {
      LOG(INFO) << "Input TGDs are not closed for " << prn 
                 << "! H = " << std::endl << H;
      return;
    }

    // solve
    x = (H.transpose() * H).inverse() * H.transpose() * z;
    
    // fill bias handle
    for (size_t i = 0; i < codes.size(); i++) {
      if (biases_with_tgds_.find(prn) == biases_with_tgds_.end()) {
        biases_with_tgds_.insert(std::make_pair(prn, std::unordered_map<int, double>()));
      }
      std::unordered_map<int, double>& code_to_bias = biases_with_tgds_.at(prn);
      if (code_to_bias.find(codes[i]) == code_to_bias.end()) {
        code_to_bias.insert(std::make_pair(codes[i], x(i)));
      }
    }
  }
}

// Arrange ZDCBs to base frequencies
void CodeBias::arrangeZdcb()
{
  // Put ZDCBs to code biases
  for (auto zdcb : zdcbs_) {
    std::string prn = zdcb.first;
    int code = zdcb.second.code;
    double value = zdcb.second.value;
    if (biases_.find(prn) == biases_.end()) {
      biases_.insert(std::make_pair(prn, std::unordered_map<int, double>()));
    }
    biases_.at(prn).insert(std::make_pair(code, value));
  }
  
  // try to add some default DCBs
  trySetDefaultCodes(biases_);

  // Check if base frequency is zero
  for (auto it_i = biases_.begin(); it_i != biases_.end();) {
    std::string prn = it_i->first;
    std::unordered_map<int, double> code_to_bias = it_i->second;
    std::pair<int, int> base = bases_.at(prn[0]);
    auto it_lhs = code_to_bias.find(base.first);
    auto it_rhs = code_to_bias.find(base.second);
    if (it_lhs == code_to_bias.end() || 
       (it_rhs == code_to_bias.end() && base.second != CODE_NONE)) {
      LOG(INFO) << "Cannot find base code for " << prn << ". "
        << "Input ZDCBs should contain both the two base codes!";
      it_i = biases_.erase(it_i);
      continue;
    }

    double bias_lhs = it_lhs->second;
    double bias_rhs = (base.second == CODE_NONE) ? 0.0 : it_rhs->second;
    double f1 = gnss_common::codeToFrequency(prn[0], base.first);
    double f2 = gnss_common::codeToFrequency(prn[0], base.second);
    double c1 = square(f1) / (square(f1) - square(f2));
    double c2 = -square(f2) / (square(f1) - square(f2));
    double cmb = c1 * bias_lhs + c2 * bias_rhs;
    if ((base.second == CODE_NONE && !checkZero(bias_lhs, 0.1)) || 
        (base.second != CODE_NONE && !checkZero(cmb, 0.1))) {
      double out = (base.second == CODE_NONE) ? bias_lhs : cmb;
      LOG(INFO) << "Base code or the combination of base codes for " << prn
        << " is not zero (" << out << ")!";
      it_i = biases_.erase(it_i);
      continue;
    }

    it_i++;
  }

  // Copy
  biases_with_tgds_ = biases_;
}

// Check if a code exists in biases_
bool CodeBias::checkExist(const std::string prn, const int code, BiasMap& biases)
{
  if (biases.find(prn) == biases.end()) return false;
  std::unordered_map<int, double>& code_to_bias = biases.at(prn);
  if (code_to_bias.find(code) != code_to_bias.end()) return true;
  return false;
}
  
// Try to set two code biases as the same
void CodeBias::tryBindCodes(const std::string prn, 
  const int code1, const int code2, BiasMap& biases)
{
  if (biases.find(prn) == biases.end()) return;
  std::unordered_map<int, double>& code_to_bias = biases.at(prn);
  bool found1 = false, found2 = false;
  if (code_to_bias.find(code1) != code_to_bias.end()) found1 = true;
  if (code_to_bias.find(code2) != code_to_bias.end()) found2 = true;
  // already has DCB
  if (found1 && found2) return;
  // not exist
  if (!found1 && !found2) return;
  if (found1 && !found2) {
    code_to_bias.insert(std::make_pair(code2, code_to_bias.at(code1)));
  }
  else if (!found1 && found2) {
    code_to_bias.insert(std::make_pair(code1, code_to_bias.at(code2)));
  }
}

// Try to set some default code relations
void CodeBias::trySetDefaultCodes(BiasMap& biases)
{
  std::vector<std::string> prns;
  for (auto bias : biases) {
    if (prns.size() == 0 || prns.back() != bias.first) {
      prns.push_back(bias.first);
    }
  }
  for (auto prn : prns) 
  {
    std::unordered_map<int, double>& code_to_bias = biases.at(prn);
    if (prn[0] == 'G') {
      tryBindCodes(prn, CODE_L1W, CODE_L1P, biases);
      tryBindCodes(prn, CODE_L2W, CODE_L2P, biases);
    }
  }
}

}