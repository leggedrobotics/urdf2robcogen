//
// Created by rgrandia on 24.09.19.
//

#include "urdf2robcogen/generateDtDsl.hpp"

#include "urdf2robcogen/utils.hpp"

#include <fstream>

std::string list_frames(const std::vector<std::string>& linkAndFrameNames) {
  std::ostringstream out;

  // All frames, 4 per line
  int k = 0;
  out << "\t";
  for (const auto& name : linkAndFrameNames) {
    if (k > 3) {
      out << std::endl;
      out << "\t";
      k = 0;
    }
    out << "fr_" << name;
    if (name != linkAndFrameNames.back()) {
      out << ", ";
    }
    k++;
  }
  return out.str();
}

std::string list_transforms(const std::string& rootName, const std::vector<std::string>& linkAndFrameNames) {
  std::ostringstream out;

  // request all transforms from and to the base
  for (const auto& name : linkAndFrameNames) {
    if (name == rootName) {
      continue;
    }
    out << "\tbase=fr_" << rootName << ", name=fr_" << name << std::endl;
    out << "\tbase=fr_" << name << ", target=fr_" << rootName << std::endl;
  }

  return out.str();
}

std::string list_jacobians(const std::string& rootName, const std::vector<std::string>& linkAndFrameNames) {
  // Note: Don't request jacobians from frames of the same link!
  // For now, generate Jacobians to all links and frames from the base
  std::ostringstream out;

  for (const auto& name : linkAndFrameNames) {
    if (name == rootName) {
      continue;
    }
    out << "\tbase=fr_" << rootName << ", target=fr_" << name << std::endl;
  }

  return out.str();
}

void generateDtDsl(const std::string& robotName, const std::string& rootName, const std::vector<std::string>& robotLinkNames,
                   const std::vector<std::string>& robotFixedFrameNames) {
  std::vector<std::string> linkAndFrameNames;
  linkAndFrameNames.insert(linkAndFrameNames.end(), robotLinkNames.begin(), robotLinkNames.end());
  linkAndFrameNames.insert(linkAndFrameNames.end(), robotFixedFrameNames.begin(), robotFixedFrameNames.end());

  std::string dtdsl_name = robotName + ".dtdsl";
  std::ofstream dtdsl_file(dtdsl_name.c_str());

  print_timeStamp(dtdsl_file);

  dtdsl_file << "Robot " << robotName << std::endl;
  dtdsl_file << "Frames {" << std::endl;
  dtdsl_file << list_frames(linkAndFrameNames) << std::endl;
  dtdsl_file << "}" << std::endl;

  dtdsl_file << "Transforms {" << std::endl;
  dtdsl_file << list_transforms(rootName, linkAndFrameNames) << std::endl;
  dtdsl_file << "}" << std::endl;

  dtdsl_file << "Jacobians {" << std::endl;
  dtdsl_file << list_jacobians(rootName, linkAndFrameNames) << std::endl;
  dtdsl_file << "}" << std::endl;

  dtdsl_file.close();
}