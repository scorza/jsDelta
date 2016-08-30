#include "jsDelta.h"
#include "jsDeltaCmd.h"
#include "common.h"

const char* JSDeltaCmd::kName = "jsDeltaCmd";
const char* JSDeltaCmd::kNameFlagShort = "-n";
const char* JSDeltaCmd::kNameFlagLong = "-name";
const char* JSDeltaCmd::kSmoothIterationsFlagShort = "-si";
const char* JSDeltaCmd::kSmoothIterationsFlagLong = "-smoothIterations";

JSDeltaCmd::JSDeltaCmd() {}

void* JSDeltaCmd::creator() { return new JSDeltaCmd; }

MSyntax JSDeltaCmd::newSyntax() {
	MSyntax syntax;

	syntax.addFlag(kNameFlagShort, kNameFlagLong, MSyntax::kString);
	syntax.addFlag(kSmoothIterationsFlagShort, kSmoothIterationsFlagLong, MSyntax::kString);
	syntax.setObjectType(MSyntax::kSelectionList, 1);
	syntax.useSelectionAsDefault(true);

	syntax.enableEdit(false);
	syntax.enableQuery(false);

	return syntax;
}

MStatus JSDeltaCmd::doIt(const MArgList& argList) {
	MStatus status;

	/// syntax() comes packaged with all MPxCommand, points to pointer in 
	/// registerCommand() in pluginMain, which points to newSyntax() below
	MArgDatabase argData(syntax(), argList, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MSelectionList mSel;
	argData.getObjects(mSel);
	mSel.getDagPath(0, pathDeformMesh_);
	status = getShapeNode(pathDeformMesh_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Get command arguments
	if (argData.isFlagSet("-n")) 
	{
		nameArg_ = argData.flagArgumentString("-n", 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	else { nameArg_ = "jsDelta#"; }

	if (argData.isFlagSet("-si"))
	{
		smoothIterations_ = argData.flagArgumentInt("-si", 0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	else { smoothIterations_ = 20; }

	// Set command to execute on dgMod_
	MString command = "deformer -type jsDelta -n \"" + nameArg_ + "\" ";
	command += pathDeformMesh_.partialPathName().asChar();
	status = dgMod_.commandToExecute(command);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return redoIt();
}

MStatus JSDeltaCmd::redoIt() {
	MStatus status;

	status = dgMod_.doIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	getLastDeltaNode(deltaNode_);
	
	status = createRefMesh(pathRefMesh_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = getShapeNode(pathRefMesh_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = createSmoothMesh(pathSmoothMesh_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = getShapeNode(pathSmoothMesh_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Get adjacent verts
	MItMeshVertex iterMeshVert(pathRefMesh_, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	adjVerts_.resize(iterMeshVert.count());
	for (; !iterMeshVert.isDone(); iterMeshVert.next())
	{
		MIntArray vertList;
		iterMeshVert.getConnectedVertices(vertList);
		adjVerts_[iterMeshVert.index()] = vertList;
	}

	// Smooth smooth mesh, get tanget matrices, calculate deltas, delete smooth mesh
	MFnMesh fnSmoothMesh(pathSmoothMesh_);
	fnSmoothMesh.getPoints(initPoints_, MSpace::kWorld);

	status = SmoothPoints(smoothIterations_, adjVerts_, initPoints_, smoothPoints_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = fnSmoothMesh.setPoints(smoothPoints_, MSpace::kWorld);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// Get tangent matrices
	tangentMatrices_.resize(smoothPoints_.length());
	for (unsigned int i = 0; i < smoothPoints_.length(); ++i)
	{
		tangentMatrices_[i] = GetTangentMatrix(smoothPoints_[i],
											   smoothPoints_[adjVerts_[i][0]],
											   smoothPoints_[adjVerts_[i][1]]);
	}
	
	
	MFnMesh fnMesh(pathDeformMesh_);
	fnMesh.getPoints(initPoints_, MSpace::kWorld);

	
	// Calculate deltas
	status = CalculateDeltas(initPoints_, smoothPoints_, tangentMatrices_, deltas_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// Delete smoothed mesh
	if (smoothGeos_.length())
	{
		MDGModifier dgModDelete;
		for (unsigned int i = 0; i < smoothGeos_.length(); ++i)
		{
			status = dgModDelete.commandToExecute("delete " + smoothGeos_[i]);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		status = dgModDelete.doIt();
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Get plugs and connect to store initial data
	MPlug plugIterations(deltaNode_, JSDelta::aSmoothIterations);
	MPlug plugRefMesh(deltaNode_, JSDelta::aRefMesh);
	MPlug plugAdjVerts(deltaNode_, JSDelta::aAdjacentVerts);
	MPlug plugDeltas(deltaNode_, JSDelta::aDeltas);
	MPlug plugIsConnected(deltaNode_, JSDelta::aIsConnected);

	MDGModifier dgModConnect;

	// Store smooth iterations value
	status = plugIterations.setInt(smoothIterations_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Store any per-component data: adjVerts
	MItGeometry iterGeo(pathDeformMesh_);
	for (; !iterGeo.isDone(); iterGeo.next())
	{
		// Store adjVerts per component
		int idx = iterGeo.index();
		MPlug plugAdjVertsElement = plugAdjVerts.elementByLogicalIndex(idx);
		MFnIntArrayData fnIntArray;
		MObject oIntArray = fnIntArray.create(adjVerts_[idx], &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = dgModConnect.newPlugValue(plugAdjVertsElement, oIntArray);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	// Store deltas (not per component bc MVectorArray)
	MFnVectorArrayData fnVectorArray;
	MObject oVectorArray = fnVectorArray.create(deltas_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = dgModConnect.newPlugValue(plugDeltas, oVectorArray);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Connect ref mesh
	MFnDagNode fnDagRef(pathRefMesh_);
	MPlug plugOutMesh = fnDagRef.findPlug("outMesh", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = dgModConnect.connect(plugOutMesh, plugRefMesh);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Set isConnected value so deformer init continues on the node
	status = plugIsConnected.setValue(true);

	status = dgModConnect.doIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus JSDeltaCmd::undoIt() {
	MStatus status;
	
	status = dgMod_.undoIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	/// status = dgModRef_.undoIt();
	/// CHECK_MSTATUS_AND_RETURN_IT(status);

	if (refGeos_.length())
	{
		MDGModifier dgModDelete;
		for (unsigned int i = 0; i < refGeos_.length(); ++i)
		{
			status = dgModDelete.commandToExecute("delete " + refGeos_[i]);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		status = dgModDelete.doIt();
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	return MS::kSuccess;
}

bool JSDeltaCmd::isUndoable() const {
	return true;
}

MStatus JSDeltaCmd::getShapeNode(MDagPath& path) {
	MStatus status;

	if (path.node().hasFn(MFn::kMesh))
	{
		return MS::kSuccess;
	}

	unsigned int numShapes;
	status = path.numberOfShapesDirectlyBelow(numShapes);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	for (unsigned int i = 0; i < numShapes; i++) {
		status = path.extendToShapeDirectlyBelow(i);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		if (!path.hasFn(MFn::kMesh)) {
			path.pop();
			continue;
		}

		MFnDagNode fnDag(path, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		if (!fnDag.isIntermediateObject()) {
			return MS::kSuccess;
		}
		path.pop();
	}
	return MS::kFailure;
}

MStatus JSDeltaCmd::getLastDeltaNode(MObject& oDeltaNode) {
	MStatus status;

	MObject oDeformMesh = pathDeformMesh_.node();
	MItDependencyGraph iterDG(oDeformMesh, MFn::kGeometryFilt,
		MItDependencyGraph::kUpstream,
		MItDependencyGraph::kDepthFirst,
		MItDependencyGraph::kNodeLevel, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	while (!iterDG.isDone()) {
		oDeltaNode = iterDG.currentItem();
		MFnDependencyNode fnDepNode(oDeltaNode, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		if (fnDepNode.typeId() == JSDelta::id) {
			return MS::kSuccess;
		}
		iterDG.next();
	}
	return MS::kFailure;
}

MStatus JSDeltaCmd::createRefMesh(MDagPath& path) {
	MStatus status;

	MStringArray duplicateReturn;
	MFnDependencyNode fnDepDelta(deltaNode_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDagNode fnDagDeform(pathDeformMesh_);

	MGlobal::executeCommand("duplicate -rr -n " + fnDagDeform.name() + "_jsDeltaBase " + fnDagDeform.partialPathName(), duplicateReturn);

	refGeos_.append(duplicateReturn[0]);
	MSelectionList mSel;
	status = MGlobal::getSelectionListByName(duplicateReturn[0], mSel);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = mSel.getDagPath(0, path);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnDagNode fnDagRef(path, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug plugVis = fnDagRef.findPlug("visibility", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = plugVis.setBool(false);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus JSDeltaCmd::createSmoothMesh(MDagPath& path) {
	MStatus status;

	MStringArray duplicateReturn;
	MFnDependencyNode fnDepDelta(deltaNode_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDagNode fnDagDeform(pathDeformMesh_);

	MGlobal::executeCommand("duplicate -rr -n " + fnDagDeform.name() + "_jsDeltaSmooth " + fnDagDeform.partialPathName(), duplicateReturn);

	smoothGeos_.append(duplicateReturn[0]);
	MSelectionList mSel;
	status = MGlobal::getSelectionListByName(duplicateReturn[0], mSel);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = mSel.getDagPath(0, path);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnDagNode fnDagRef(path, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug plugVis = fnDagRef.findPlug("visibility", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = plugVis.setBool(false);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus JSDeltaCmd::getDagPathFromName(MString& name, MDagPath& path) {
	MStatus status;

	MSelectionList mSel;
	status = MGlobal::getSelectionListByName(name, mSel);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = mSel.getDagPath(0, path);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}