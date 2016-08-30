#pragma once
#include "common.h"

/* MAIN DEFORM CLASS */
class JSDelta : public MPxDeformerNode 
{
public:
	JSDelta();
	virtual ~JSDelta();
	static void* creator();
	static MStatus initialize();

	virtual MStatus deform(MDataBlock& data, MItGeometry& iterGeo, 
		const MMatrix& toWorldMatrix, unsigned int indexGeo);
	MStatus getSmoothPoints(MPointArray& unsmoothPoints, MPointArray& targetArray, unsigned int& smoothIterations);
	MStatus calculateDeltas(MPointArray &originalPoints, MPointArray &smoothedPoints);
	MStatus kickGPU(bool kickOrNot);
	
	static MObject aAutoKickGPU;
	static MObject aRebind;
	static MObject aSmoothIterations;
	static MObject aRefMesh;
	static MObject aAdjacentVerts;
	static MObject aDeltas;
	static MObject aDeltasBuffer;
	static MObject aIsConnected;

	const static char* kName;
	static MTypeId id;
	
	bool initialized_;
	static MFloatArray weights_;
	static MVectorArray deltas_;
	
private:
	unsigned int numPoints_;
	MPointArray refPoints_;
	MPointArray smoothRefPoints_;
	MPointArray smoothDeformPoints_;
	MPointArray finalPoints_;
	unsigned int previousNumIterations_;
	
	
	std::vector<MMatrix> refTangentMatrices_;
	std::vector<MIntArray> adjMap_;


	// BENCHMARK STUFF
	int clockCounter_;
	int durationTotal_;
	MStatus benchmark(long long durationTotal, const char* operation);

};

/* TBB STRUCTS */
struct Smooth_TBB
{////// according to intel docs this has implicit constructor/destructor, right?
public:
	////// but we redeclare / overload the constructor with these arguments
	Smooth_TBB(MPointArray *unsmoothPoints,
				MPointArray *targetPoints,
				unsigned int smoothIterations, ///////// Should I be declaring int and let the compiler sort it out??
				const std::vector<MIntArray> &adjMap);
	
	void operator() (const tbb::blocked_range<size_t> &r) const;

private:
	MPointArray *source_;
	MPointArray *target_;
	unsigned int iterations_;
	const std::vector<MIntArray>& adjMap_tbb;
};

struct Tangent_TBB
{
public:
	Tangent_TBB(MPointArray *pts,
				MPointArray *smoothPts, 
				MPointArray *finalPts,
				MVectorArray *deltas,
				const std::vector<MIntArray> &adjMap,
				const MFloatArray &weights,
				float& envelope);

	void operator() (const tbb::blocked_range<size_t> &r) const;

private:
	MPointArray *pts_;
	MPointArray *smoothPts_;
	MPointArray *finalPts_;
	MVectorArray *deltas_;
	const std::vector<MIntArray>& adjMap_tbb;
	const MFloatArray &weights_tbb;
	float& envelope_tbb; /////// declaring a reference variable?

};

/* GPU CLASS OVERRIDE */
#if MAYA_API_VERSION >= 201600
class JSDeltaGPU : public MPxGPUDeformer
{
public:
	JSDeltaGPU();
	virtual ~JSDeltaGPU();
	virtual void terminate();
	static MGPUDeformerRegistrationInfo* getGPUDeformerInfo();
	virtual MPxGPUDeformer::DeformerStatus evaluate(MDataBlock &data,
													const MEvaluationNode &evaluationNode,
													const MPlug &plug,
													unsigned int numElements,
													const MAutoCLMem inputBuffer,
													const MAutoCLEvent inputEvent,
													MAutoCLMem outputBuffer,
													MAutoCLEvent &outputEvent);

	static MString pluginLoadPath; // Used to find the CL kernel

private:
	// Helper functions
	cl_int enqueueBuffer(MAutoCLMem &maclMem, size_t bufferSize, void* data);
	cl_int initializeKernel(MDataBlock &data, const MPlug &plug, const unsigned int numElements);

	bool isInit;

	// Storage for data on the GPU
	MAutoCLMem clSmoothPts;
	MAutoCLMem clAdjPerVertex;
	MAutoCLMem clAdjOffsets;
	MAutoCLMem clAdjVerts;
	MAutoCLMem clDeltas;
	MAutoCLMem clWeights;
	MAutoCLMem clPing;
	MAutoCLMem clPong;

	size_t pingLocalWorkSize;
	size_t pingGlobalWorkSize;
	size_t pingWorkGroupSize;
	size_t pingReturnSize;

	// Kernel: represents an openCL function loaded from disK
	MAutoCLKernel kernelDelta_;
	MAutoCLKernel kernelSmooth_;

};

class JSDeltaGPUInfo : public MGPUDeformerRegistrationInfo
{
public:
	JSDeltaGPUInfo();
	virtual ~JSDeltaGPUInfo();
	virtual MPxGPUDeformer* createGPUDeformer();

	virtual bool validateNode(MDataBlock &data,
							const MEvaluationNode &evaluationNode,
							const MPlug &plug,
							MStringArray *messages);


};
#endif