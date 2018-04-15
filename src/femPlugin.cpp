// Copyright (c) 2018, Danilo Ferreira. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <femPlugin.h>
#include <femObjectData.h>
#include <femMesh.h>
#include <femObject.h>
#include <femSolver.h>
#include <femDebug.h>
#include <ftNumber.h>
#include <ftSolver.h>
#include <ftConvert.h>

#include <maya/MGlobal.h>
#include <maya/MAnimControl.h>
#include <maya/MFnPlugin.h>
#include <maya/MCommonSystemUtils.h>
#include <maya/MTime.h>

#include <nglib.h>

#include <openvdb/openvdb.h>

FEMPlugin::FEMPlugin() {}
FEMPlugin::~FEMPlugin() {}

MStatus FEMPlugin::initialize(MObject & object, const MString & author, const MString & version) {
    nglib::Initialize();
    openvdb::initialize();

    MStatus status;
    MFnPlugin plugin(object, author.asChar(), version.asChar());

    status = plugin.registerData(FEMObjectData::typeName, FEMObjectData::id,
        FEMObjectData::creator);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FEMMesh::typeName, FEMMesh::id,
        FEMMesh::creator, FEMMesh::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FEMObject::typeName, FEMObject::id,
        FEMObject::creator, FEMObject::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FEMSolver::typeName, FEMSolver::id,
        FEMSolver::creator, FEMSolver::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FEMDebug::typeName, FEMDebug::id,
        FEMDebug::creator, FEMDebug::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FTNumber::typeName, FTNumber::id,
        FTNumber::creator, FTNumber::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FTSolver::typeName, FTSolver::id,
        FTSolver::creator, FTSolver::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerNode(FTConvert::typeName, FTConvert::id,
        FTConvert::creator, FTConvert::initialize);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.registerUI("femCreateUI", "femDeleteUI");
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = MGlobal::executeCommand("workspace -fileRule \"femCache\" \"cache/FEM\"");
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MCommonSystemUtils::makeDirectory(getCacheDirectory());
}
MStatus FEMPlugin::uninitialize(MObject & object) {
    openvdb::uninitialize();

    MStatus status;
    MFnPlugin plugin(object);

    status = plugin.deregisterNode(FEMMesh::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(FEMObject::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(FEMSolver::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(FEMDebug::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterData(FEMObjectData::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MGlobal::executeCommand("workspace -removeFileRuleEntry \"femCache\"");
}

double FEMPlugin::getStartTime() {
    return MAnimControl::minTime().value();
}
double FEMPlugin::getEndTime() {
    return MAnimControl::maxTime().value();
}
double FEMPlugin::getFramerate() {
    return MTime(1.0, MTime::kSeconds).as(MTime::uiUnit());
}
MString FEMPlugin::getCacheName() {
    return MString("defaultCache");
}
MString FEMPlugin::getCacheDirectory() {
    MString fileRule, cacheDirectory;

    MGlobal::executeCommand("workspace -fileRuleEntry \"femCache\"", fileRule);
    MGlobal::executeCommand("workspace -expandName \"" + fileRule + "\"", cacheDirectory);

    return cacheDirectory;
}

MStatus initializePlugin(MObject object) {
    return FEMPlugin::initialize(object, FEM_PLUGIN_AUTHOR, FEM_PLUGIN_VERSION);
}
MStatus uninitializePlugin(MObject object) {
    return FEMPlugin::uninitialize(object);
}