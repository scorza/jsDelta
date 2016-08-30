#include "common.h"

/* STANDALONE FUNCTIONS */

MStatus SmoothPoints(const unsigned int iterations,
					 const std::vector<MIntArray> &adjVerts,
					 MPointArray &originalPoints,
					 MPointArray &targetPoints)
{
	MStatus status;

	unsigned int numPoints = originalPoints.length();
	targetPoints.setLength(numPoints);

	double *pts_PTR = &originalPoints[0].x;
	double *newPts_PTR = &targetPoints[0].x;
	double adjx;
	double adjy;
	double adjz;

	for (unsigned int i = 0; i < iterations; ++i)
	{
		
		for (unsigned int j = 0; j < numPoints; ++j)
		{
			MPoint pt;
			unsigned int adjLength = adjVerts[j].length();
			float adjInverse = 1 / float(adjLength);
						
			for (unsigned int k = 0; k < adjLength; ++k)
			{
				unsigned int adjIndex = adjVerts[j][k];

				adjx = pts_PTR[adjIndex * 4 + 0];
				adjy = pts_PTR[adjIndex * 4 + 1];
				adjz = pts_PTR[adjIndex * 4 + 2];

				MVector adj(adjx, adjy, adjz);
				pt += adj;
			}
			pt = pt * adjInverse;
			newPts_PTR[j * 4 + 0] = pt.x;
			newPts_PTR[j * 4 + 1] = pt.y;
			newPts_PTR[j * 4 + 2] = pt.z;
			newPts_PTR[j * 4 + 3] = 1.0;
		}

		if (i != iterations - 1) /* ______________________ this makes sense right? */
		{
			std::swap(pts_PTR, newPts_PTR);
		}

	}

	return MS::kSuccess;
}


MStatus CalculateDeltas(const MPointArray &originalPoints,
						const MPointArray &smoothedPoints,
						const std::vector<MMatrix> &tangentMatrices,
						MVectorArray &deltas)
{
	MStatus status;

	deltas.setLength(originalPoints.length());

	for (unsigned int i = 0; i < originalPoints.length(); ++i)
	{
		MMatrix tangentMatrix = tangentMatrices[i];

		MVector originalVec = originalPoints[i];
		MVector smoothedVec = smoothedPoints[i];

		MVector delta = originalVec - smoothedVec;
		delta *= tangentMatrix.inverse();

		deltas[i] = delta;

	}
	return MS::kSuccess;
}
