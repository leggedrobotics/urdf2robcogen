//
// Created by rgrandia on 24.09.19.
//

#include <string>

#include <tinyxml.h>
#include <urdf/model.h>

/**
 * Creates a kindsl and dtdsl file based on the provided urdf.
 *
 * If the root link is called "world", a fixed based system is generated, otherwise a floating base system is generated.
 *
 * The following conversions are applied to comply with the RobCoGen conventions
 * - links that are attached with 'fixed' joints are converted to frames on the first moveable parent link
 * - moveable joints are reoriented to move along their z-axis
 *
 */
void urdf2RobCoGen(const std::string& robotName, urdf::Model urdf, TiXmlDocument urdfXml, const std::string& outputFolder, bool verbose);
