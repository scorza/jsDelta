#include "jsDelta.h"
#include "common.h"

using std::chrono::microseconds;
using std::chrono::duration_cast;
typedef std::chrono::high_resolution_clock Clock;


MTypeId JSDelta::id(0x2efff);
const char* JSDelta::kName = "jsDelta";

MObject JSDelta::aAutoKickGPU;
MObject JSDelta::aSmoothIterations;
MObject JSDelta::aRefMesh;
MObject JSDelta::aAdjacentVerts;
MObject JSDelta::aDeltas;
MObject JSDelta::aDeltasBuffer;
MObject JSDelta::aIsConnected;

MFloatArray JSDelta::weights_; // set as static public member so can be accessed from GPU override class
MVectorArray JSDelta::deltas_; // set as static public member so can be accessed from GPU override class


JSDelta::JSDelta()	:initialized_(false) {}
JSDelta::~JSDelta() {}

void* JSDelta::creator() { 
	return new JSDelta(); 
}

MStatus JSDelta::initialize() {
	MStatus status;

	MFnNumericAttribute fnNumAttr;
	MFnTypedAttribute fnTypAttr;

	aAutoKickGPU = fnNumAttr.create("autoKickGPU", "autoKickGPU", MFnNumericData::kBoolean, false);
	fnNumAttr.setKeyable(true);
	fnNumAttr.setStorable(true);
	addAttribute(aAutoKickGPU);
	attributeAffects(aAutoKickGPU, outputGeom);
	
	aSmoothIterations = fnNumAttr.create("smoothIterations", "smoothIterations", MFnNumericData::kInt, 20);
	fnNumAttr.setKeyable(true);
	fnNumAttr.setStorable(true);
	fnNumAttr.setMin(1);
	addAttribute(aSmoothIterations);
	attributeAffects(aSmoothIterations, outputGeom);

	aRefMesh = fnTypAttr.create("referenceMesh", "referenceMesh", MFnData::kMesh);
	fnTypAttr.setHidden(true);
	addAttribute(aRefMesh);
	//attributeAffects(aRefMesh, outputGeom);

	aAdjacentVerts = fnTypAttr.create("adjacentVerts", "adjacentVerts", MFnNumericData::kIntArray);
	fnTypAttr.setArray(true);
	fnTypAttr.setHidden(true);
	addAttribute(aAdjacentVerts);
	//attributeAffects(aAdjacentVerts, outputGeom);

	aDeltas = fnTypAttr.create("deltas", "deltas", MFnNumericData::kVectorArray);
	fnTypAttr.setHidden(true);
	addAttribute(aDeltas);
	attributeAffects(aDeltas, outputGeom);

	aDeltasBuffer = fnTypAttr.create("deltasBuffer", "deltasBuffer", MFnNumericData::kVectorArray);
	fnTypAttr.setHidden(true);
	addAttribute(aDeltasBuffer);
	attributeAffects(aDeltasBuffer, outputGeom);

	aIsConnected = fnNumAttr.create("isConnected", "isConnected", MFnNumericData::kBoolean, false);
	fnNumAttr.setHidden(true);
	addAttribute(aIsConnected);
	
	MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer jsDelta weights;");

	return MS::kSuccess;
}

MStatus JSDelta::deform(MDataBlock& data, MItGeometry& iterGeo, 
						const MMatrix& toWorldMatrix, unsigned int indexGeo) {
	MStatus status;
	// Get simple values from datablock
	float env = data.inputValue(envelope).asFloat();
	unsigned int smoothIterations = data.inputValue(aSmoothIterations).asInt();
	bool kick = data.outputValue(aAutoKickGPU).asBool();
	bool isConnected = data.outputValue(aIsConnected).asBool();

	// Get weight values to store as static member variable
	weights_.setLength(iterGeo.count());
	for (iterGeo.reset(); !iterGeo.isDone(); iterGeo.next())
	{
		float weight = weightValue(data, indexGeo, iterGeo.index());
		weights_[iterGeo.index()] = weight;
	}
	iterGeo.reset();
	

	// INITIALIZE IF BRANCH ////////
	if (initialized_ == false) 
	{
		cout << "INIT branch called" << endl;
		if (!isConnected)
		{
			return MS::kSuccess;
		}
		// Check if previousNumIterations_ has a value
		if (previousNumIterations_ == NULL)
		{
			previousNumIterations_ = smoothIterations;
		}

		// Get deltas_ MVectorArray from datablock using MFnVectorArrayData
		deltas_.setLength(iterGeo.count());
		MDataHandle hDeltas = data.inputValue(aDeltas);
		MObject oVectorArray = hDeltas.data();
		MFnVectorArrayData fnVectorArray(oVectorArray, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		deltas_ = fnVectorArray.array(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		//// Fill buffer attr for GPU
		//MDataHandle hDeltasBuffer = data.outputValue(aDeltasBuffer);
		//hDeltasBuffer.set(oVectorArray);

		// Pull adjMap from datablock once
		MArrayDataHandle hArrayAdjVerts = data.inputArrayValue(aAdjacentVerts);
		MIntArray adjVerts;
		adjMap_.resize(iterGeo.count());
		unsigned int i = 0;
		for (iterGeo.reset(); !iterGeo.isDone(); ++i, iterGeo.next())
		{
			status = hArrayAdjVerts.jumpToElement(i);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			MDataHandle hAdjVerts = hArrayAdjVerts.inputValue();
			MObject oIntArray = hAdjVerts.data();
			MFnIntArrayData fnIntArray(oIntArray, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			adjMap_[i] = fnIntArray.array();
		}

		// Get adjacent verts from output geo /////////
		MObject oRefMesh = data.inputValue(aRefMesh).asMesh();
		MFnMesh fnMesh(oRefMesh, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		fnMesh.getPoints(refPoints_);
		numPoints_ = refPoints_.length();

		// Set target MPointArray length ////////////
		finalPoints_.setLength(numPoints_);
		smoothRefPoints_.setLength(numPoints_);
		smoothDeformPoints_.setLength(numPoints_);
		
		// Kick GPU override settings
		status = kickGPU(kick);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		initialized_ = true;
		return MS::kSuccess;

	}


	// Resmooth only if iterations value has changed
	if (smoothIterations != previousNumIterations_)
	{
		cout << "RESMOOTH branch called" << endl;

		smoothRefPoints_.clear();
		smoothRefPoints_.setLength(numPoints_);
		refTangentMatrices_.clear();
		refTangentMatrices_.resize(numPoints_);
		deltas_.clear();
		deltas_.setLength(numPoints_);

		cout << "resmooth called" << endl;
		MPointArray refPointsCopy = refPoints_;
		
		// OK not entirely sure why this works and getSmoothPoints fails...
		SmoothPoints(smoothIterations, adjMap_, refPointsCopy, smoothRefPoints_);

		for (unsigned int i = 0; i < numPoints_; ++i)
		{
			refTangentMatrices_[i] = GetTangentMatrix(smoothRefPoints_[i],
				smoothRefPoints_[adjMap_[i][0]],
				smoothRefPoints_[adjMap_[i][1]]);
		}
		
		// OK not entirely sure why this works and CalculateDeltas fails...
		calculateDeltas(refPoints_, smoothRefPoints_);

		//// Tried using static attr as buffer for GPU because not pulling from deltas_...
		//MDataHandle hDeltasBuffer = data.outputValue(aDeltasBuffer);
		//MFnVectorArrayData fnVectorArray;
		//MObject oDeltas = fnVectorArray.create(deltas_, &status);
		//CHECK_MSTATUS_AND_RETURN_IT(status);
		//hDeltasBuffer.set(oDeltas);

		previousNumIterations_ = smoothIterations;

		// Kick GPU override settings // CAUSES CRASH HERE
		//status = kickGPU(kick);
		//CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}


	// MAIN DEFORM /////
	auto t0 = Clock::now(); ////////////////////////////////////////////////////////////////
	MPointArray pts;
	MPointArray currentPts;
	iterGeo.allPositions(pts);
	iterGeo.allPositions(currentPts);
	status = getSmoothPoints(pts, smoothDeformPoints_, smoothIterations);
	CHECK_MSTATUS_AND_RETURN_IT(status);
		
	MPointArray *pts_PTR = &currentPts;
	MPointArray *smooth_PTR = &smoothDeformPoints_;
	MPointArray *final_PTR = &finalPoints_;
	MVectorArray *delta_PTR = &deltas_;

	iterGeo.setAllPositions(currentPts);

	Tangent_TBB kernel(pts_PTR, smooth_PTR, final_PTR, delta_PTR, adjMap_, weights_, env);
	tbb::parallel_for(tbb::blocked_range<size_t>(0, numPoints_, 2000), kernel);
		
	//// Basic algorithm, here for backup and posterity
	//for (unsigned int i = 0; i < numPoints_; ++i) {
	//	MMatrix tangentMatrix = GetTangentMatrix(smoothDeformPoints_[i],
	//											 smoothDeformPoints_[adjMap_[i][0]],
	//											 smoothDeformPoints_[adjMap_[i][1]]);
	//	MVector delta = deltas_[i];
	//	MPoint currentPt = currentPts[i];
	//	MPoint finalPt = smoothDeformPoints_[i];

	//	delta *= tangentMatrix;
	//	finalPt += delta;

	//	finalPt = finalPt + ((finalPt - currentPt) *  env);

	//	finalPoints_[i] = finalPt;
	//}
		
	auto t1 = Clock::now(); ////////////////////////////////////////////////////////////////

	iterGeo.setAllPositions(finalPoints_);

		
	/////////////////////////////// CALCULATE BENCHMARK /////////////////////////////////////////////
	long long duration = duration_cast<microseconds>(t1 - t0).count();
	char* operation = "CALCULATE WHOLE DEFORM ";
	durationTotal_ += duration;

	if (clockCounter_ >= 100)
	{
		benchmark(durationTotal_ / double(100) , operation);
		clockCounter_ = 0;
		durationTotal_ = 0;
	}
	clockCounter_++;

	
	return MS::kSuccess;
}


MStatus JSDelta::calculateDeltas(MPointArray& originalPoints, MPointArray& smoothedPoints){
	MStatus status;
	for (unsigned int i = 0; i < originalPoints.length(); i++) {
		MMatrix tangentMatrix = refTangentMatrices_[i];

		MVector originalVec = originalPoints[i];
		MVector smoothedVec = smoothedPoints[i];

		MVector delta = originalVec - smoothedVec;
		delta *= tangentMatrix.inverse();
		if (deltas_.length() < numPoints_) {
			deltas_.append(delta);
		}
		else {
			deltas_[i] = delta;
		}
	}
	return MS::kSuccess;
}

MStatus JSDelta::benchmark(long long durationTotal, const char* operation)
{
	MStatus status;

	cout << endl << endl << endl << "/////////////////////////" << endl;
	cout << "benchmark for " << operation << " took " << durationTotal_/double(100) << " microseconds" << endl;
	cout << "___________________________________________" << endl;
	

	
	return MS::kSuccess;
}

MStatus JSDelta::getSmoothPoints(MPointArray& unsmoothPoints, MPointArray& targetArray, unsigned int& smoothIterations)
{
	MStatus status;

	MPointArray *pts_PTR = &unsmoothPoints;
	MPointArray *newPts_PTR = &targetArray;
	for (unsigned int k = 0; k < smoothIterations; k++)
	{
		Smooth_TBB kernel(pts_PTR, newPts_PTR, smoothIterations, adjMap_);
		tbb::parallel_for(tbb::blocked_range<size_t>(0, numPoints_, 2000), kernel);
			
		if (k != smoothIterations - 1) /* ______________________ this makes sense right? */
		{
			std::swap(pts_PTR, newPts_PTR);
		}
	}
		
	return MS::kSuccess;
}

MStatus JSDelta::kickGPU(bool kickOrNot)
{
	MStatus status;
	
	if (kickOrNot)
	{
		MGlobal::executeCommand("turnOffOpenCLEvaluatorActive");
		MGlobal::executeCommand("turnOnOpenCLEvaluatorActive");
	}

	return MS::kSuccess;
}

/* TBB FUNCTIONS */
Smooth_TBB::Smooth_TBB(MPointArray *unsmoothPoints,	MPointArray *targetPoints, unsigned int smoothIterations,const std::vector<MIntArray> &adjMap):adjMap_tbb(adjMap)
{
	source_ = unsmoothPoints;
	target_ = targetPoints;
	iterations_ = smoothIterations;

}

void Smooth_TBB::operator() (const tbb::blocked_range<size_t> &r) const
{
	/// "declares a new pointer (pts_PTR) to the address of the value pointed to by the x-value of the first MPoint in the MPointArray (source_)
	double* pts_PTR = &(*source_)[0].x; 
	double* newPts_PTR = &(*target_)[0].x; 
	double x, y, z;
	unsigned int adjLength;
	float adjInverse;
	signed int adjIndex;
	int idx = 0;

		for (int i = r.begin(); i < r.end(); i++)
		{
			x = 0;
			y = 0;
			z = 0;
			idx = i * 4;
			adjLength = adjMap_tbb[i].length();
			adjInverse = 1 / float(adjLength);

			for (unsigned int j = 0; j < adjLength; j++)
			{
				adjIndex = adjMap_tbb[i][j];
				adjIndex *= 4;

				x += pts_PTR[adjIndex + 0];
				y += pts_PTR[adjIndex + 1];
				z += pts_PTR[adjIndex + 2];

			}
			
			newPts_PTR[i * 4 + 0] = x * adjInverse;
			newPts_PTR[i * 4 + 1] = y * adjInverse;
			newPts_PTR[i * 4 + 2] = z * adjInverse;
			newPts_PTR[i * 4 + 3] = 1.0;
			
		}
}

Tangent_TBB::Tangent_TBB(MPointArray *pts,
						MPointArray *smoothPts, 
						MPointArray *finalPts,
						MVectorArray *deltas,
						const std::vector<MIntArray> &adjMap,
						const MFloatArray &weights,
						float& envelope)
	:adjMap_tbb(adjMap), weights_tbb(weights),  envelope_tbb(envelope)
{
	pts_ = pts;
	smoothPts_ = smoothPts;
	finalPts_ = finalPts;
	deltas_ = deltas;
}

void Tangent_TBB::operator() (const tbb::blocked_range<size_t> &r) const
{
	double *finalPts_PTR = &(*finalPts_)[0].x;
	double *smoothPts_PTR = &(*smoothPts_)[0].x;
	double *pts_PTR = &(*pts_)[0].x;
	double *deltas_PTR = &(*deltas_)[0].x;

	int idx = 0;
	int adjIndexX, adjIndexZ;
	double magx, magz;
	float weight;
	/// double result[4][4];
	MMatrix resultMatrix;
	MVector vec, vecx, vecy, vecz, dvec, pt;
	
	double dx_mtx;
	double dy_mtx;
	double dz_mtx;
	double dw_mtx;
	
	double& vx = vec[0];
	double& vy = vec[1];
	double& vz = vec[2];

	double& vxx = vecx[0];
	double& vxy = vecx[1];
	double& vxz = vecx[2];

	double& vyx = vecy[0];
	double& vyy = vecy[1];
	double& vyz = vecy[2];

	double& vzx = vecz[0];
	double& vzy = vecz[1];
	double& vzz = vecz[2];

	double& dx = dvec[0];
	double& dy = dvec[1];
	double& dz = dvec[2];

	double& ptx = pt[0];
	double& pty = pt[1];
	double& ptz = pt[2];

	for (int i = r.begin(); i < r.end(); i++)
	{	
		weight = weights_tbb[i];
		adjIndexX = adjMap_tbb[i][0];
		adjIndexZ = adjMap_tbb[i][1];
		adjIndexX *= 4;
		adjIndexZ *= 4;
		idx = i * 4;

		//NB: shift deltas_PTR by 4 instead of by 3 for a weird mc escher effect!

		vx = smoothPts_PTR[idx + 0];
		vy = smoothPts_PTR[idx + 1];
		vz = smoothPts_PTR[idx + 2];

		vxx = smoothPts_PTR[adjIndexX + 0] - vx;
		vxy = smoothPts_PTR[adjIndexX + 1] - vy;
		vxz = smoothPts_PTR[adjIndexX + 2] - vz;

		vzx = smoothPts_PTR[adjIndexZ + 0] - vx;
		vzy = smoothPts_PTR[adjIndexZ + 1] - vy;
		vzz = smoothPts_PTR[adjIndexZ + 2] - vz;

		dx = deltas_PTR[i * 3];
		dy = deltas_PTR[i * 3 + 1];
		dz = deltas_PTR[i * 3 + 2];

		ptx = pts_PTR[idx];
		pty = pts_PTR[idx + 1];
		ptz = pts_PTR[idx + 2];

		magx = 1 / sqrt(vxx*vxx + vxy*vxy + vxz*vxz);
		magz = 1 / sqrt(vzx*vzx + vzy*vzy + vzz*vzz);

		vxx *= magx;
		vxy *= magx;
		vxz *= magx;

		vzx *= magz;
		vzy *= magz;
		vzz *= magz;
		
		vecy = vecx ^ vecz;
		vecy.normalize();
		vecz = vecx ^ vecy;
		vecz.normalize();

		double result[4][4] = { { vxx, vxy, vxz, 0 },
								{ vyx, vyy, vyz, 0 },
								{ vzx, vzy, vzz, 0 },
								{ vx,  vy,  vz,  1 }};
		resultMatrix = result;

		dvec *= resultMatrix;

		vx += dx;
		vy += dy;
		vz += dz;

		vx = ptx + ((vx - ptx) * weight * envelope_tbb);
		vy = pty + ((vy - pty) * weight * envelope_tbb);
		vz = ptz + ((vz - ptz) * weight * envelope_tbb);


		finalPts_PTR[idx + 0] = vx;
		finalPts_PTR[idx + 1] = vy;
		finalPts_PTR[idx + 2] = vz;
		finalPts_PTR[idx + 3] = 1.0;

	}
}


#if MAYA_API_VERSION >= 201600
/* GPU FUNCTIONS */

MString JSDeltaGPU::pluginLoadPath; // Must be public static to be accessed inside pluginMain.cpp

JSDeltaGPU::JSDeltaGPU() : isInit(false) {}
JSDeltaGPU::~JSDeltaGPU() {}
void JSDeltaGPU::terminate()
{
	clAdjPerVertex.reset();
	clAdjOffsets.reset();
	clAdjVerts.reset();
	clDeltas.reset();
	MOpenCLInfo::releaseOpenCLKernel(kernelSmooth_);
	MOpenCLInfo::releaseOpenCLKernel(kernelDelta_);
	kernelSmooth_.reset();
	kernelDelta_.reset();
}
MGPUDeformerRegistrationInfo* JSDeltaGPU::getGPUDeformerInfo()
{
	static JSDeltaGPUInfo deltaInfo;
	return &deltaInfo;
}


MPxGPUDeformer::DeformerStatus JSDeltaGPU::evaluate(MDataBlock &data,
											const MEvaluationNode &evaluationNode,
											const MPlug &plug,
											unsigned int numElements,
											const MAutoCLMem inputBuffer,
											const MAutoCLEvent inputEvent,
											MAutoCLMem outputBuffer,
											MAutoCLEvent &outputEvent)
{
	cl_int err = CL_SUCCESS;
	MStatus status;
	MPxGPUDeformer::DeformerStatus defStatus(kDeformerSuccess);

	std::vector<MIntArray> adjMap;
	MVectorArray deltas;

	// Kernel + static value initialization check
	if (!isInit)
	{
		err = initializeKernel(data, plug, numElements);
		MOpenCLInfo::checkCLErrorStatus(err);
		isInit = true;
	}

	// Get simple values from datablock
	float env = data.inputValue(MPxDeformerNode::envelope).asFloat();
	unsigned int smoothIterations = data.inputValue(JSDelta::aSmoothIterations).asInt();
	

	// Flatten and store paint weights
	float *pWeights = new float[numElements];
	for (int i = 0; i < numElements; ++i)
	{
		pWeights[i] = JSDelta::weights_[i];
	}

	err = enqueueBuffer(clWeights, numElements * sizeof(float), (void*)pWeights);
	MOpenCLInfo::checkCLErrorStatus(err);
	delete[] pWeights;

	// create void pointers to clPing and clPong
	void *in = (void*)clPing.getReadOnlyRef();
	void *out = (void*)clPong.getReadOnlyRef();

	// Execute pingpong smooth in loop
	unsigned int parameterId = 0;
	for (unsigned int i = 0; i < smoothIterations; ++i)
	{
		if (i == 0)
		{

			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), in);
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)inputBuffer.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjPerVertex.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjOffsets.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjVerts.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_uint),(void*)&numElements);
			MOpenCLInfo::checkCLErrorStatus(err);

			err = clEnqueueNDRangeKernel(MOpenCLInfo::getOpenCLCommandQueue(),
										kernelSmooth_.get(),
										1,
										NULL,
										&pingGlobalWorkSize,
										&pingLocalWorkSize,
										1,
										inputEvent.getReadOnlyRef(),
										outputEvent.getReferenceForAssignment());
			MOpenCLInfo::checkCLErrorStatus(err);
		}
		else if (i > 0 && i < smoothIterations - 1)
		{
			parameterId = 0;
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), out);
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), in);
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjPerVertex.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjOffsets.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjVerts.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_uint), (void*)&numElements);
			MOpenCLInfo::checkCLErrorStatus(err);
			
			err = clEnqueueNDRangeKernel(MOpenCLInfo::getOpenCLCommandQueue(),
										kernelSmooth_.get(),
										1,
										NULL,
										&pingGlobalWorkSize,
										&pingLocalWorkSize,
										1,
										inputEvent.getReadOnlyRef(),
										outputEvent.getReferenceForAssignment());
			MOpenCLInfo::checkCLErrorStatus(err);
			std::swap(in, out);
		}
		else
		{
			parameterId = 0;
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), out);
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), in);
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjPerVertex.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjOffsets.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjVerts.getReadOnlyRef());
			err = clSetKernelArg(kernelSmooth_.get(), parameterId++, sizeof(cl_uint), (void*)&numElements);
			MOpenCLInfo::checkCLErrorStatus(err);

			err = clEnqueueNDRangeKernel(MOpenCLInfo::getOpenCLCommandQueue(),
										kernelSmooth_.get(),
										1,
										NULL,
										&pingGlobalWorkSize,
										&pingLocalWorkSize,
										1,
										inputEvent.getReadOnlyRef(),
										outputEvent.getReferenceForAssignment());
			MOpenCLInfo::checkCLErrorStatus(err);
		}
	}
	
	
	clFinish(MOpenCLInfo::getOpenCLCommandQueue());


	// Set arguments for delta kernel operation
	parameterId = 0;
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)outputBuffer.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)inputBuffer.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), out);
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjPerVertex.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjOffsets.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)clAdjVerts.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)clDeltas.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_mem), (void*)clWeights.getReadOnlyRef());
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_float),(void*)&env);
	err = clSetKernelArg(kernelDelta_.get(), parameterId++, sizeof(cl_uint), (void*)&numElements);
	MOpenCLInfo::checkCLErrorStatus(err);


	// Figure out good work group size for delta kernel
	size_t localWorkSize = 256;
	size_t globalWorkSize;
	size_t workGroupSize;
	size_t returnSize;
	err = clGetKernelWorkGroupInfo(kernelDelta_.get(),
								   MOpenCLInfo::getOpenCLDeviceId(),
								   CL_KERNEL_WORK_GROUP_SIZE,
								   sizeof(size_t),
								   &workGroupSize,
								   &returnSize);
	MOpenCLInfo::checkCLErrorStatus(err);

	if (returnSize > 0) { localWorkSize = workGroupSize; }
	globalWorkSize = (localWorkSize - numElements % localWorkSize) + numElements; // globalWorkSize must be multiple of localWorkSize
	unsigned int numInputEvents = 0;
	if (inputEvent.get()) { numInputEvents = 1;	}

	// Execute delta kernel
	err = clEnqueueNDRangeKernel(MOpenCLInfo::getOpenCLCommandQueue(), // cl_command_queue, in maya wrapper
								kernelDelta_.get(), // cl_kernel return of our MAutoCLKernel kernelDelta_
								1, // work dimension
								NULL, // "work offset" must be 0 in all current versions(?)
								&globalWorkSize,
								&localWorkSize,
								numInputEvents,
								numInputEvents ? inputEvent.getReadOnlyRef() : 0, // Use inputEvent if any
								outputEvent.getReferenceForAssignment());	// Associate this kernel execution with outputEvent
	MOpenCLInfo::checkCLErrorStatus(err);

	return defStatus;
}

cl_int JSDeltaGPU::initializeKernel(MDataBlock &data, const MPlug &plug, const unsigned int numElements)
{
	cl_int err = CL_SUCCESS;
	MStatus status;

	// Load OpenCL kernels
	if (!kernelDelta_.get())
	{
		// == openCL kernel file isn't loaded, so load from disk
		MString openCLKernelFile(pluginLoadPath);
		openCLKernelFile += "/jsDelta.cl";
		kernelDelta_ = MOpenCLInfo::getOpenCLKernel(openCLKernelFile, "delta");
		if (kernelDelta_.isNull())
		{
			cerr << "Could not compile openCL kernel " << openCLKernelFile.asChar() << endl;
			return MPxGPUDeformer::kDeformerFailure;
		}
	}

	if (!kernelSmooth_.get())
	{
		// == openCL kernel file isn't loaded, so load from disk
		MString openCLKernelFile(pluginLoadPath);
		openCLKernelFile += "/jsDelta.cl";
		kernelSmooth_ = MOpenCLInfo::getOpenCLKernel(openCLKernelFile, "smooth");
		if (kernelSmooth_.isNull())
		{
			cerr << "Could not compile openCL kernel " << openCLKernelFile.asChar() << endl;
			return MPxGPUDeformer::kDeformerFailure;
		}
	}

	std::vector<MIntArray> adjMap;
	MVectorArray deltas;
	MFloatArray weightVals;

	// Fill adjMap from datablock
	MArrayDataHandle hArrayAdjVerts = data.inputArrayValue(JSDelta::aAdjacentVerts);
	adjMap.resize(numElements);
	for (unsigned int i = 0; i < numElements; ++i)
	{
		status = hArrayAdjVerts.jumpToElement(i);
		CHECK_MSTATUS(status);
		MDataHandle hAdjVerts = hArrayAdjVerts.inputValue();
		MObject oIntArray = hAdjVerts.data();
		MFnIntArrayData fnIntArray(oIntArray, &status);
		CHECK_MSTATUS(status);
		adjMap[i] = fnIntArray.array();
	}

	// Flatten and store adjMap + adjOffsets and adjPerVertex
	size_t arraySize = adjMap.size();
	int* pAdjPerVertex = new int[arraySize];
	int* pAdjOffsets = new int[arraySize];
	unsigned int totalAdjs = 0;
	for (size_t i = 0; i < adjMap.size(); ++i)
	{
		pAdjPerVertex[i] = (int)adjMap[i].length();
		pAdjOffsets[i] = totalAdjs;
		totalAdjs += pAdjPerVertex[i];
	}

	err = enqueueBuffer(clAdjPerVertex, arraySize * sizeof(int), (void*)pAdjPerVertex);
	MOpenCLInfo::checkCLErrorStatus(err);
	err = enqueueBuffer(clAdjOffsets, arraySize * sizeof(int), (void*)pAdjOffsets);
	MOpenCLInfo::checkCLErrorStatus(err);
	delete[] pAdjPerVertex;
	delete[] pAdjOffsets;

	int* pAdjMap = new int[totalAdjs];
	unsigned int iter = 0;
	for (size_t i = 0; i < adjMap.size(); ++i)
	{
		for (unsigned int j = 0; j < adjMap[i].length(); ++j)
		{
			pAdjMap[iter] = adjMap[i][j];
			iter++;
		}
	}
	err = enqueueBuffer(clAdjVerts, totalAdjs * sizeof(int), (void*)pAdjMap);
	MOpenCLInfo::checkCLErrorStatus(err);
	delete[] pAdjMap;

	//// Tried filling from buffer attr on datablock to allow change to smooth iterations on gpu
	//deltas.setLength(numElements);
	//MDataHandle hDeltas = data.inputValue(JSDelta::aDeltas);
	//MObject oDeltas = hDeltas.data();
	//MFnVectorArrayData fnVectorArray(oDeltas, &status);
	//CHECK_MSTATUS(status);
	//deltas = fnVectorArray.array(&status);
	//CHECK_MSTATUS(status);

	// Fill deltas from static member variable deltas_ (allows user to change smooth iters)
	deltas = JSDelta::deltas_;
	
	// Flatten and store deltas
	arraySize = deltas.length() * 3;
	float* pDeltas = new float[arraySize];
	iter = 0;
	for (size_t i = 0; i < deltas.length(); ++i)
	{
		pDeltas[iter++] = deltas[i].x;
		pDeltas[iter++] = deltas[i].y;
		pDeltas[iter++] = deltas[i].z;
	}

	err = enqueueBuffer(clDeltas, arraySize * sizeof(float), (void*)pDeltas);
	MOpenCLInfo::checkCLErrorStatus(err);
	delete[] pDeltas;

	// Figure out good work group size for kernelSmooth
	pingLocalWorkSize = 256;
	err = clGetKernelWorkGroupInfo(kernelSmooth_.get(),
		MOpenCLInfo::getOpenCLDeviceId(),
		CL_KERNEL_WORK_GROUP_SIZE,
		sizeof(size_t),
		&pingWorkGroupSize, // read up, lots of different strategies for choosing
		&pingReturnSize);
	MOpenCLInfo::checkCLErrorStatus(err);
	if (pingReturnSize > 0) { pingLocalWorkSize = pingWorkGroupSize; }
	pingGlobalWorkSize = (pingLocalWorkSize - numElements % pingLocalWorkSize) + numElements; // globalWorkSize must be multiple of localWorkSize

	// Set buffer length for clPing, clPong
	clPing.attach(clCreateBuffer(MOpenCLInfo::getOpenCLContext(), CL_MEM_READ_WRITE, sizeof(float) * numElements * 3, NULL, &err));
	clPong.attach(clCreateBuffer(MOpenCLInfo::getOpenCLContext(), CL_MEM_READ_WRITE, sizeof(float) * numElements * 3, NULL, &err));

	return err;
}

cl_int JSDeltaGPU::enqueueBuffer(MAutoCLMem &maclMem, size_t bufferSize, void* data)
{
	cl_int err = CL_SUCCESS;
	if (!maclMem.get())
	{
		// The buffer doesn't exist yet, create it and copy data over
		maclMem.attach(clCreateBuffer(MOpenCLInfo::getOpenCLContext(),
									  CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
									  bufferSize, data, &err));
	}
	else
	{
		// The buffer already exists, just copy data over
		err = clEnqueueWriteBuffer(MOpenCLInfo::getOpenCLCommandQueue(),
								   maclMem.get(), CL_TRUE, 0, 
								   bufferSize, data, 0, NULL, NULL);
	}
	return err;
}


/* GPU INFO FUNCTIONS */

JSDeltaGPUInfo::JSDeltaGPUInfo() {}
JSDeltaGPUInfo::~JSDeltaGPUInfo() {}
MPxGPUDeformer* JSDeltaGPUInfo::createGPUDeformer() 
{
	return new JSDeltaGPU;
}

bool JSDeltaGPUInfo::validateNode(MDataBlock &data,
									const MEvaluationNode &evaluationNode,
									const MPlug &plug,
									MStringArray *messages)
{
	return true;
}
#endif