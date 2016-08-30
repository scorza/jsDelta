__kernel void smooth(__global float* outPts,
					__global const float* inPts,
					__global const int* adjPerVertex,
					__global const int* adjOffsets,
					__global const int* adjVerts,
					const int numElements
					)
{
	unsigned int positionId = get_global_id(0);	// represents current vertex id

	if (positionId >= numElements) { return; }
	unsigned int offset = positionId * 3;

	float avgx = 0.0;
	float avgy = 0.0;
	float avgz = 0.0;

	float ptx = inPts[offset];
	float pty = inPts[offset + 1];
	float ptz = inPts[offset + 2];

	int adjOff = adjOffsets[positionId];
	int adjPer = adjPerVertex[positionId];
	float adjInv = 1 / (float)adjPer;

	for (unsigned int i = 0; i < adjPer; ++i)
	{
		int adjId = adjVerts[adjOff + i];
		float adjx = inPts[adjId * 3];
		float adjy = inPts[adjId * 3 + 1];
		float adjz = inPts[adjId * 3 + 2];

		avgx += adjx;
		avgy += adjy;
		avgz += adjz;
	}

	avgx *= adjInv;
	avgy *= adjInv;
	avgz *= adjInv;

	outPts[offset]	   = avgx;
	outPts[offset + 1] = avgy;
	outPts[offset + 2] = avgz;

}



__kernel void delta(__global float* finalPts,
					__global const float* currentPts,
					__global const float* smoothPts,
					__global const int* adjPerVertex,
					__global const int* adjOffsets,
					__global const int* adjVerts,
					__global const float* deltas,
					__global const float* weights,
					const float envelope,
					const int numElements
					)
{
	unsigned int positionId = get_global_id(0);
	if (positionId >= numElements) { return; }
	unsigned int posOffset = positionId * 3;

	float ptx = currentPts[posOffset];
	float pty = currentPts[posOffset + 1];
	float ptz = currentPts[posOffset + 2];

	float smx = smoothPts[posOffset];
	float smy = smoothPts[posOffset + 1];
	float smz = smoothPts[posOffset + 2];

	float dx = deltas[posOffset];
	float dy = deltas[posOffset + 1];
	float dz = deltas[posOffset + 2];

	// Get adjacent verts
	int adjOff = adjOffsets[positionId];
	int adjIdX = adjVerts[adjOff];
	int adjIdZ = adjVerts[adjOff + 1];

	float adjxx = smoothPts[adjIdX * 3];
	float adjxy = smoothPts[adjIdX * 3 + 1];
	float adjxz = smoothPts[adjIdX * 3 + 2];

	float adjzx = smoothPts[adjIdZ * 3];
	float adjzy = smoothPts[adjIdZ * 3 + 1];
	float adjzz = smoothPts[adjIdZ * 3 + 2];

	// Get vectors from adj points
	float vxx = adjxx - smx;
	float vxy = adjxy - smy;
	float vxz = adjxz - smz;

	float vzx = adjzx - smx;
	float vzy = adjzy - smy;
	float vzz = adjzz - smz;
	
	// Normalize vectors
	float magx = 1 / sqrt(vxx*vxx + vxy*vxy + vxz*vxz);
	float magz = 1 / sqrt(vzx*vzx + vzy*vzy + vzz*vzz);
	
	vxx *= magx;
	vxy *= magx;
	vxz *= magx;

	vzx *= magz;
	vzy *= magz;
	vzz *= magz;

	// Cast as float vectors
	float3 vx = (float3)(vxx, vxy, vxz);
	float3 vz = (float3)(vzx, vzy, vzz);

	// Calculate y vector from cross product of x and z vectors
	float3 vy = cross(vx, vz);
	vy = normalize(vy);
	vz = cross(vx, vy);
	vz = normalize(vz);

	vxx = vx.x;
	vxy = vx.y;
	vxz = vx.z;

	float vyx = vy.x;
	float vyy = vy.y;
	float vyz = vy.z;
	
	vzx = vz.x;
	vzy = vz.y;
	vzz = vz.z;

	float4 delta = { dx, dy, dz, 0.0f };
	
	////////// MAGIC HAPPENS //////////
	//float4 mx = (float4)(vxx, vxy, vxz, 0.0f);
	//float4 my = (float4)(vyx, vyy, vyz, 0.0f);
	//float4 mz = (float4)(vzx, vzy, vzz, 0.0f);
	//float4 mw = (float4)(smx, smy, smz, 1.0f);

	float4 mx = (float4)(vxx, vyx, vzx, smx);
	float4 my = (float4)(vxy, vyy, vzy, smy);
	float4 mz = (float4)(vxz, vyz, vzz, smz);
	float4 mw = (float4)(0.0, 0.0, 0.0, 1.0f);

	float4 delta_mtx = (float4)((dot(delta, mx)),
								(dot(delta, my)),
								(dot(delta, mz)),
								(dot(delta, mw)));

	smx += delta_mtx.x;
	smy += delta_mtx.y;
	smz += delta_mtx.z;

	
	// Apply envelope
	smx = ptx + ((smx - ptx) * weights[positionId] * envelope);
	smy = pty + ((smy - pty) * weights[positionId] * envelope);
	smz = ptz + ((smz - ptz) * weights[positionId] * envelope);
	
	
	// Set final points
	finalPts[posOffset]		= smx;
	finalPts[posOffset + 1] = smy;
	finalPts[posOffset + 2] = smz;
}