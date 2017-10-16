// Copyright (c) 2017, Danilo Peixoto. All rights reserved.
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

#include "femObjectData.h"
#include "femMesh.h"
#include "femObject.h"
#include "femSolver.h"

#include <maya/MFnPlugin.h>

MStatus initializePlugin(MObject object) {
    MStatus status;
    MFnPlugin plugin(object, "Danilo Peixoto", "1.0.0");

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

    status = plugin.registerUI("femCreateUI", "femDeleteUI");
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}

MStatus uninitializePlugin(MObject object) {
    MStatus status;
    MFnPlugin plugin(object);

    status = plugin.deregisterNode(FEMMesh::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(FEMObject::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterNode(FEMSolver::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plugin.deregisterData(FEMObjectData::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}