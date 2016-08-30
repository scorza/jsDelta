#pragma once

/* C++ INCLUDES */
#include <chrono>
#include <map>
#include <math.h>
#include <queue>
#include <set>
#include <stdio.h>
#include <string>
#include <time.h>
#include <utility>
#include <vector>
#include <unordered_map>

/* MFN SETS */
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnFloatArrayData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnVectorArrayData.h>

/* MFN ATTRS */
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>

/* ITERATORS */
#include <maya/MItDag.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshVertex.h>

/* PROXY CLASSES */
#include <maya/MPxCommand.h>
#include <maya/MPxDeformerNode.h>

/* GENERAL */
#include <maya/MArgList.h>
#include <maya/MArgDatabase.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDGMessage.h>
#include <maya/MDGModifier.h>
#include <maya/MFloatPointArray.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MNodeMessage.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MSyntax.h>
#include <maya/MVector.h>

/* TBB INCLUDES */
#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>

/* MAYA 2016 GPU CLASSES */
#if MAYA_API_VERSION >= 201600
#include <maya/MPxGPUDeformer.h>
#include <maya/MGPUDeformerRegistry.h>
#include <maya/MOpenCLInfo.h>
#include <clew/clew_cl.h>
#endif

/* STANDALONE FUNCTIONS */
inline MMatrix GetTangentMatrix(const MPoint &pt, const MPoint &ptx, const MPoint &ptz)
{
	MVector vec = MVector(pt);
	MVector vecx = MVector(ptx);
	MVector vecz = MVector(ptz);
	vecx = vecx - vec;
	vecz = vecz - vec;
	vecx.normalize();
	vecz.normalize();
	MVector vecy = vecx ^ vecz;
	vecy.normalize();
	vecz = vecx ^ vecy;
	vecz.normalize();
	double result[4][4] = { { vecx.x, vecx.y, vecx.z, 0 },
							{ vecy.x, vecy.y, vecy.z, 0 },
							{ vecz.x, vecz.y, vecz.z, 0 },
							{ pt.x,   pt.y,   pt.z,   1 } };
	MMatrix resultMatrix = result;

	return resultMatrix;
}


MStatus SmoothPoints(const unsigned int iterations,
					 const std::vector<MIntArray> &adjVerts,
					 MPointArray &originalPoints,
					 MPointArray &targetPoints);

MStatus CalculateDeltas(const MPointArray &originalPoints,
						const MPointArray &smoothedPoints,
						const std::vector<MMatrix> &tangentMatrices,
						MVectorArray &deltas);
