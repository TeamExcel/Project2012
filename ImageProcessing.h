 
//**************************************************************************
//* WARNING: This file was automatically generated.  Any changes you make  *
//*          to this file will be lost if you generate the file again.     *
//**************************************************************************
#ifndef IMAGEPROCESSING_TASK_INCLUDE
#define IMAGEPROCESSING_TASK_INCLUDE
#include <nivision.h>
//#include <nimachinevision.h>

#ifdef __cplusplus
	extern "C" {
#endif
	
#define IVA_MAX_BUFFERS 10
#define IVA_STORE_RESULT_NAMES
#ifndef BOOL
	#define BOOL bool
#endif
	
typedef enum IVA_ResultType_Enum {IVA_NUMERIC, IVA_BOOLEAN, IVA_STRING} IVA_ResultType;

typedef union IVA_ResultValue_Struct    // A result in Vision Assistant can be of type double, BOOL or string.
{
	double numVal;
	BOOL   boolVal;
	char*  strVal;
} IVA_ResultValue;

typedef struct IVA_Result_Struct
{
#if defined (IVA_STORE_RESULT_NAMES)
	char resultName[256];           // Result name
#endif
	IVA_ResultType  type;           // Result type
	IVA_ResultValue resultVal;      // Result value
} IVA_Result;

typedef struct IVA_StepResultsStruct
{
#if defined (IVA_STORE_RESULT_NAMES)
	char stepName[256];             // Step name
#endif
	int         numResults;         // number of results created by the step
	IVA_Result* results;            // array of results
} IVA_StepResults;

typedef struct IVA_Data_Struct
{
	Image* buffers[IVA_MAX_BUFFERS];            // Vision Assistant Image Buffers
	IVA_StepResults* stepResults;              // Array of step results
	int numSteps;                               // Number of steps allocated in the stepResults array
	CoordinateSystem *baseCoordinateSystems;    // Base Coordinate Systems
	CoordinateSystem *MeasurementSystems;       // Measurement Coordinate Systems
	int numCoordSys;                            // Number of coordinate systems
} IVA_Data;


	
	
int IVA_ProcessImage(Image *image, IVA_Data_Struct *ivaData);
int IVA_DisposeData(IVA_Data* ivaData);
IVA_Data* IVA_InitData(int numSteps, int numCoordSys);

#ifdef __cplusplus
	}
#endif

#endif // ifndef IMAGEPROCESSING_TASK_INCLUDE
