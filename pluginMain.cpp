#include "jsDelta.h"
#include "jsDeltaCmd.h"
#include "common.h"

#include <maya/MFnPlugin.h>

MStatus initializePlugin(MObject mobj) {
	MStatus status;

	// Main deformer and command registration
	MFnPlugin fnPlugin(mobj, "James Sumner III", "1.0", "2016");
	status = fnPlugin.registerNode(JSDelta::kName, 
									JSDelta::id,
									JSDelta::creator,
									JSDelta::initialize,
									MPxNode::kDeformerNode);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = fnPlugin.registerCommand(JSDeltaCmd::kName, 
									  JSDeltaCmd::creator,
									  JSDeltaCmd::newSyntax);
	
	// GPU deformer registration
#if MAYA_API_VERSION >= 201600
	status = MGPUDeformerRegistry::registerGPUDeformerCreator(JSDelta::kName, "jsDeltaGPU",
															  JSDeltaGPU::getGPUDeformerInfo());
	CHECK_MSTATUS_AND_RETURN_IT(status);
	JSDeltaGPU::pluginLoadPath = fnPlugin.loadPath();
	cout << "plugin loaded from " << JSDeltaGPU::pluginLoadPath << endl;

#endif
	
	// Python script import to create menu items
	if (MGlobal::mayaState() == MGlobal::kInteractive) {
		try {
			status = MGlobal::executePythonCommandOnIdle("import jsDelta.menu");
			CHECK_MSTATUS_AND_RETURN_IT(status);
			status = MGlobal::executePythonCommandOnIdle("jsDelta.menu.create_menu_items()");
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		catch (...) {
			return MS::kSuccess;
		}
	}
	return status;

}

MStatus uninitializePlugin(MObject mobj) {
	MStatus status;
	MFnPlugin fnPlugin(mobj);

	// GPU deformer deregistration
#if MAYA_API_VERSION >= 201600
	status = MGPUDeformerRegistry::deregisterGPUDeformerCreator(JSDelta::kName, "jsDeltaGPU");
	CHECK_MSTATUS_AND_RETURN_IT(status);
#endif


	// Main plugin command and deformer deregistration
	status = fnPlugin.deregisterCommand(JSDeltaCmd::kName);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = fnPlugin.deregisterNode(JSDelta::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);


	// Destroy menu items
	if (MGlobal::mayaState() == MGlobal::kInteractive) {
		try {
			status = MGlobal::executePythonCommandOnIdle("import jsDelta.menu");
			CHECK_MSTATUS_AND_RETURN_IT(status);
			status = MGlobal::executePythonCommandOnIdle("jsDelta.menu.destroy_menu_items()");
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
		catch (...) {
			return MS::kSuccess;
		}
	}
	
	
	
	return status;
}