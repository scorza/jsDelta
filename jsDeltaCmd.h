#pragma once
#include "common.h"

class JSDeltaCmd : public MPxCommand {
	public:
		// FUNCS ////
		JSDeltaCmd();
		static void* creator();
		static MSyntax newSyntax();

		virtual MStatus doIt(const MArgList& argList);
		virtual MStatus redoIt();
		virtual MStatus undoIt();
		virtual bool isUndoable() const;

		// VARS //////
		const static char* kName;

		// COMMAND FLAGS ////
		const static char* kNameFlagShort;
		const static char* kNameFlagLong;
		const static char* kSmoothIterationsFlagShort;
		const static char* kSmoothIterationsFlagLong;

	private:
		// FUNCS ////
		MStatus getShapeNode(MDagPath& path);
		MStatus getLastDeltaNode(MObject& oDeltaNode);
		MStatus createRefMesh(MDagPath& path);
		MStatus createSmoothMesh(MDagPath& path);
		MStatus getDagPathFromName(MString& name, MDagPath& path);

		// VARS ////
		MDagPath pathRefMesh_;
		MDagPath pathSmoothMesh_;
		MDagPath pathDeformMesh_;
		MDGModifier dgMod_;
		MObject deltaNode_;

		MPointArray initPoints_;
		MPointArray smoothPoints_;
		std::vector<MIntArray> adjVerts_;
		std::vector<MMatrix> tangentMatrices_;
		MVectorArray deltas_;
		MStringArray refGeos_;
		MStringArray smoothGeos_;
		
		MString nameArg_;
		unsigned int smoothIterations_;
};
