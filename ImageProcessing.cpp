#include <nivision.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "ImageProcessing.h"
//#include <nimachinevision.h>
//#include <windows.h>

// If you call Machine Vision functions in your script, add NIMachineVision.c to the project.

//#ifndef BOOL
//	#define BOOL bool
//#endif
//#define IVA_MAX_BUFFERS 10
//#define IVA_STORE_RESULT_NAMES

#define VisionErrChk(Function) {if (!(Function)) {success = 0; goto Error;}}

//typedef enum IVA_ResultType_Enum {IVA_NUMERIC, IVA_BOOLEAN, IVA_STRING} IVA_ResultType;
//
//typedef union IVA_ResultValue_Struct    // A result in Vision Assistant can be of type double, BOOL or string.
//{
//	double numVal;
//	BOOL   boolVal;
//	char*  strVal;
//} IVA_ResultValue;
//
//typedef struct IVA_Result_Struct
//{
//#if defined (IVA_STORE_RESULT_NAMES)
//	char resultName[256];           // Result name
//#endif
//	IVA_ResultType  type;           // Result type
//	IVA_ResultValue resultVal;      // Result value
//} IVA_Result;
//
//typedef struct IVA_StepResultsStruct
//{
//#if defined (IVA_STORE_RESULT_NAMES)
//	char stepName[256];             // Step name
//#endif
//	int         numResults;         // number of results created by the step
//	IVA_Result* results;            // array of results
//} IVA_StepResults;
//
//typedef struct IVA_Data_Struct
//{
//	Image* buffers[IVA_MAX_BUFFERS];            // Vision Assistant Image Buffers
//	IVA_StepResults* stepResults;              // Array of step results
//	int numSteps;                               // Number of steps allocated in the stepResults array
//	CoordinateSystem *baseCoordinateSystems;    // Base Coordinate Systems
//	CoordinateSystem *MeasurementSystems;       // Measurement Coordinate Systems
//	int numCoordSys;                            // Number of coordinate systems
//} IVA_Data;



static int IVA_DisposeStepResults(IVA_Data* ivaData, int stepIndex);
static int IVA_CLRThreshold(Image* image, int min1, int max1, int min2, int max2, int min3, int max3, int colorMode);
static int IVA_ParticleFilter(Image* image,
									   int pParameter[],
									   float plower[],
									   float pUpper[],
									   int pCalibrated[],
									   int pExclude[],
									   int criteriaCount,
									   int rejectMatches,
									   int connectivity);
static int IVA_Particle(Image* image,
								 int connectivity,
								 int pPixelMeasurements[],
								 int numPixelMeasurements,
								 int pCalibratedMeasurements[],
								 int numCalibratedMeasurements,
								 IVA_Data* ivaData,
								 int stepIndex);

int IVA_ProcessImage(Image *image, IVA_Data_Struct *ivaData)
{
	int success = 1;
	//IVA_Data *ivaData;
	int pParameter[2] = {17,16};
	float plower[2] = {20,60};
	float pUpper[2] = {40,80};
	int pCalibrated[2] = {0,0};
	int pExclude[2] = {0,0};
	int pPixelMeasurements[28] = {0,1,2,3,4,5,6,7,16,17,18,26,27,28,
		38,39,41,42,43,45,48,49,50,51,85,86,87,88};
	int *pCalibratedMeasurements = 0;

	// Initializes internal data (buffers and array of points for caliper measurements)
	//VisionErrChk(ivaData = IVA_InitData(4, 0));

	VisionErrChk(IVA_CLRThreshold(image, 56, 125, 55, 255, 150, 255, 
		IMAQ_HSV));

	//-------------------------------------------------------------------//
	//                  Advanced Morphology: Convex Hull                 //
	//-------------------------------------------------------------------//

	// Computes the convex envelope for each labeled particle in the source image.
	VisionErrChk(imaqConvexHull(image, image, TRUE));

	VisionErrChk(IVA_ParticleFilter(image, pParameter, plower, pUpper, 
		pCalibrated, pExclude, 2, FALSE, TRUE));

	VisionErrChk(IVA_Particle(image, TRUE, pPixelMeasurements, 28, 
		pCalibratedMeasurements, 0, ivaData, 3));

	// Releases the memory allocated in the IVA_Data structure.
	//IVA_DisposeData(ivaData);

Error:
	return success;
}

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IVA_CLRThreshold
//
// Description  : Thresholds a color image.
//
// Parameters   : image      -  Input image
//                min1       -  Minimum range for the first plane
//                max1       -  Maximum range for the first plane
//                min2       -  Minimum range for the second plane
//                max2       -  Maximum range for the second plane
//                min3       -  Minimum range for the third plane
//                max3       -  Maximum range for the third plane
//                colorMode  -  Color space in which to perform the threshold
//
// Return Value : success
//
////////////////////////////////////////////////////////////////////////////////
static int IVA_CLRThreshold(Image* image, int min1, int max1, int min2, int max2, int min3, int max3, int colorMode)
{
	int success = 1;
	Image* thresholdImage;
	Range plane1Range;
	Range plane2Range;
	Range plane3Range;


	//-------------------------------------------------------------------//
	//                          Color Threshold                          //
	//-------------------------------------------------------------------//

	// Creates an 8 bit image for the thresholded image.
	VisionErrChk(thresholdImage = imaqCreateImage(IMAQ_IMAGE_U8, 7));

	// Set the threshold range for the 3 planes.
	plane1Range.minValue = min1;
	plane1Range.maxValue = max1;
	plane2Range.minValue = min2;
	plane2Range.maxValue = max2;
	plane3Range.minValue = min3;
	plane3Range.maxValue = max3;

	// Thresholds the color image.
	VisionErrChk(imaqColorThreshold(thresholdImage, image, 1, (ColorMode) colorMode, &plane1Range, &plane2Range, &plane3Range));

	// Copies the threshold image in the souce image.
	VisionErrChk(imaqDuplicate(image, thresholdImage));

Error:
	imaqDispose(thresholdImage);

	return success;
}


////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IVA_ParticleFilter
//
// Description  : Filters particles based on their morphological measurements.
//
// Parameters   : image          -  Input image
//                pParameter     -  Morphological measurement that the function
//                                  uses for filtering.
//                plower         -  Lower bound of the criteria range.
//                pUpper         -  Upper bound of the criteria range.
//                pCalibrated    -  Whether to take a calibrated measurement or not.
//                pExclude       -  TRUE indicates that a match occurs when the
//                                  value is outside the criteria range.
//                criteriaCount  -  number of particle filter criteria.
//                rejectMatches  -  Set this parameter to TRUE to transfer only
//                                  those particles that do not meet all the criteria.
//                                  Set this parameter to FALSE to transfer only those
//                                  particles that meet all the criteria to the destination.
//                connectivity   -  Set this parameter to 1 to use connectivity-8
//                                  to determine whether particles are touching.
//                                  Set this parameter to 0 to use connectivity-4
//                                  to determine whether particles are touching.
//
// Return Value : success
//
////////////////////////////////////////////////////////////////////////////////
static int IVA_ParticleFilter(Image* image,
									   int pParameter[],
									   float plower[],
									   float pUpper[],
									   int pCalibrated[],
									   int pExclude[],
									   int criteriaCount,
									   int rejectMatches,
									   int connectivity)
{
	int success = 1;
	ParticleFilterCriteria2* particleCriteria = NULL;
	int i;
	ParticleFilterOptions particleFilterOptions;
	int numParticles;


	//-------------------------------------------------------------------//
	//                          Particle Filter                          //
	//-------------------------------------------------------------------//

	if (criteriaCount > 0)
	{
		// Fill in the ParticleFilterCriteria2 structure.
		particleCriteria = (ParticleFilterCriteria2*)malloc(criteriaCount * sizeof(ParticleFilterCriteria2));

		for (i = 0 ; i < criteriaCount ; i++)
		{
			particleCriteria[i].parameter = (MeasurementType)pParameter[i];
			particleCriteria[i].lower = plower[i];
			particleCriteria[i].upper = pUpper[i];
			particleCriteria[i].calibrated = pCalibrated[i];
			particleCriteria[i].exclude = pExclude[i];
		}
		
		particleFilterOptions.rejectMatches = rejectMatches;
		particleFilterOptions.rejectBorder = 0;
		particleFilterOptions.connectivity8 = connectivity;
		
		// Filters particles based on their morphological measurements.
		VisionErrChk(imaqParticleFilter3(image, image, particleCriteria, criteriaCount, &particleFilterOptions, NULL, &numParticles));
	}

Error:
	free(particleCriteria);

	return success;
}


////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IVA_Particle
//
// Description  : Computes the number of particles detected in a binary image and
//                a 2D array of requested measurements about the particle.
//
// Parameters   : image                      -  Input image
//                connectivity               -  Set this parameter to 1 to use
//                                              connectivity-8 to determine
//                                              whether particles are touching.
//                                              Set this parameter to 0 to use
//                                              connectivity-4 to determine
//                                              whether particles are touching.
//                pixelMeasurements          -  Array of measuremnets parameters
//                numPixelMeasurements       -  Number of elements in the array
//                calibratedMeasurements     -  Array of measuremnets parameters
//                numCalibratedMeasurements  -  Number of elements in the array
//                ivaData                    -  Internal Data structure
//                stepIndex                  -  Step index (index at which to store
//                                              the results in the resuts array)
//
// Return Value : success
//
////////////////////////////////////////////////////////////////////////////////
static int IVA_Particle(Image* image,
								 int connectivity,
								 int pPixelMeasurements[],
								 int numPixelMeasurements,
								 int pCalibratedMeasurements[],
								 int numCalibratedMeasurements,
								 IVA_Data* ivaData,
								 int stepIndex)
{
	int success = 1;
	int numParticles;
	double* pixelMeasurements = NULL;
	double* calibratedMeasurements = NULL;
	unsigned int visionInfo;
	IVA_Result* particleResults;
	int i;
	int j;
	double centerOfMassX;
	double centerOfMassY;


	//-------------------------------------------------------------------//
	//                         Particle Analysis                         //
	//-------------------------------------------------------------------//

	// Counts the number of particles in the image.
	VisionErrChk(imaqCountParticles(image, connectivity, &numParticles));

	// Allocate the arrays for the measurements.
	pixelMeasurements = (double*)malloc(numParticles * numPixelMeasurements * sizeof(double));
	calibratedMeasurements = (double*)malloc(numParticles * numCalibratedMeasurements * sizeof(double));

	// Delete all the results of this step (from a previous iteration)
	IVA_DisposeStepResults(ivaData, stepIndex);

	// Check if the image is calibrated.
	VisionErrChk(imaqGetVisionInfoTypes(image, &visionInfo));

	// If the image is calibrated, we also need to log the calibrated position (x and y)
	ivaData->stepResults[stepIndex].numResults = (visionInfo & IMAQ_VISIONINFO_CALIBRATION ?
												  numParticles * 4 + 1 : numParticles * 2 + 1);
	ivaData->stepResults[stepIndex].results = (IVA_Result *)malloc (sizeof(IVA_Result) * ivaData->stepResults[stepIndex].numResults);
	
	particleResults = ivaData->stepResults[stepIndex].results;

	#if defined (IVA_STORE_RESULT_NAMES)
		sprintf(particleResults->resultName, "Object #");
	#endif
	particleResults->type = IVA_NUMERIC;
	particleResults->resultVal.numVal = numParticles;
	particleResults++;
	
	for (i = 0 ; i < numParticles ; i++)
	{
		// Computes the requested pixel measurements about the particle.
		for (j = 0 ; j < numPixelMeasurements ; j++)
		{
			VisionErrChk(imaqMeasureParticle(image, i, FALSE, (MeasurementType)pPixelMeasurements[j], &pixelMeasurements[i*numPixelMeasurements + j]));
		}

		// Computes the requested calibrated measurements about the particle.
		for (j = 0 ; j < numCalibratedMeasurements ; j++)
		{
			VisionErrChk(imaqMeasureParticle(image, i, TRUE, (MeasurementType)pCalibratedMeasurements[j], &calibratedMeasurements[i*numCalibratedMeasurements + j]));
		}
		
		#if defined (IVA_STORE_RESULT_NAMES)
			sprintf(particleResults->resultName, "Particle %d.X Position (Pix.)", i + 1);
		#endif
		particleResults->type = IVA_NUMERIC;
		VisionErrChk(imaqMeasureParticle(image, i, FALSE, IMAQ_MT_CENTER_OF_MASS_X, &centerOfMassX));
		particleResults->resultVal.numVal = centerOfMassX;
		particleResults++;

		#if defined (IVA_STORE_RESULT_NAMES)
			sprintf(particleResults->resultName, "Particle %d.Y Position (Pix.)", i + 1);
		#endif
		particleResults->type = IVA_NUMERIC;
		VisionErrChk(imaqMeasureParticle(image, i, FALSE, IMAQ_MT_CENTER_OF_MASS_Y, &centerOfMassY));
		particleResults->resultVal.numVal = centerOfMassY;
		particleResults++;

		if (visionInfo & IMAQ_VISIONINFO_CALIBRATION)
		{
			#if defined (IVA_STORE_RESULT_NAMES)
				sprintf(particleResults->resultName, "Particle %d.X Position (Calibrated)", i + 1);
			#endif
			particleResults->type = IVA_NUMERIC;
			VisionErrChk(imaqMeasureParticle(image, i, TRUE, IMAQ_MT_CENTER_OF_MASS_X, &centerOfMassX));
			particleResults->resultVal.numVal = centerOfMassX;
			particleResults++;

			#if defined (IVA_STORE_RESULT_NAMES)
				sprintf(particleResults->resultName, "Particle %d.Y Position (Calibrated)", i + 1);
			#endif
			particleResults->type = IVA_NUMERIC;
			VisionErrChk(imaqMeasureParticle(image, i, TRUE, IMAQ_MT_CENTER_OF_MASS_Y, &centerOfMassY));
			particleResults->resultVal.numVal = centerOfMassY;
			particleResults++;
		}
	}

Error:
	free(pixelMeasurements);
	free(calibratedMeasurements);

	return success;
}


////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IVA_InitData
//
// Description  : Initializes data for buffer management and results.
//
// Parameters   : # of steps
//                # of coordinate systems
//
// Return Value : success
//
////////////////////////////////////////////////////////////////////////////////
IVA_Data* IVA_InitData(int numSteps, int numCoordSys)
{
	int success = 1;
	IVA_Data* ivaData = NULL;
	int i;


	// Allocate the data structure.
	VisionErrChk(ivaData = (IVA_Data*)malloc(sizeof (IVA_Data)));

	// Initializes the image pointers to NULL.
	for (i = 0 ; i < IVA_MAX_BUFFERS ; i++)
		ivaData->buffers[i] = NULL;

	// Initializes the steo results array to numSteps elements.
	ivaData->numSteps = numSteps;

	ivaData->stepResults = (IVA_StepResults*)malloc(ivaData->numSteps * sizeof(IVA_StepResults));
	for (i = 0 ; i < numSteps ; i++)
	{
		#if defined (IVA_STORE_RESULT_NAMES)
			sprintf(ivaData->stepResults[i].stepName, "");
		#endif
		ivaData->stepResults[i].numResults = 0;
		ivaData->stepResults[i].results = NULL;
	}

	// Create the coordinate systems
	ivaData->baseCoordinateSystems = NULL;
	ivaData->MeasurementSystems = NULL;
	if (numCoordSys)
	{
		ivaData->baseCoordinateSystems = (CoordinateSystem*)malloc(sizeof(CoordinateSystem) * numCoordSys);
		ivaData->MeasurementSystems = (CoordinateSystem*)malloc(sizeof(CoordinateSystem) * numCoordSys);
	}

	ivaData->numCoordSys = numCoordSys;

Error:
	return ivaData;
}


////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IVA_DisposeData
//
// Description  : Releases the memory allocated in the IVA_Data structure
//
// Parameters   : ivaData  -  Internal data structure
//
// Return Value : success
//
////////////////////////////////////////////////////////////////////////////////
int IVA_DisposeData(IVA_Data* ivaData)
{
	int i;


	// Releases the memory allocated for the image buffers.
	for (i = 0 ; i < IVA_MAX_BUFFERS ; i++)
		imaqDispose(ivaData->buffers[i]);

	// Releases the memory allocated for the array of measurements.
	for (i = 0 ; i < ivaData->numSteps ; i++)
		IVA_DisposeStepResults(ivaData, i);

	free(ivaData->stepResults);

	// Dispose of coordinate systems
	if (ivaData->numCoordSys)
	{
		free(ivaData->baseCoordinateSystems);
		free(ivaData->MeasurementSystems);
	}

	free(ivaData);

	return TRUE;
}


////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IVA_DisposeStepResults
//
// Description  : Dispose of the results of a specific step.
//
// Parameters   : ivaData    -  Internal data structure
//                stepIndex  -  step index
//
// Return Value : success
//
////////////////////////////////////////////////////////////////////////////////
static int IVA_DisposeStepResults(IVA_Data* ivaData, int stepIndex)
{
	int i;

	
	for (i = 0 ; i < ivaData->stepResults[stepIndex].numResults ; i++)
	{
		if (ivaData->stepResults[stepIndex].results[i].type == IVA_STRING)
			free(ivaData->stepResults[stepIndex].results[i].resultVal.strVal);
	}

	free(ivaData->stepResults[stepIndex].results);

	return TRUE;
}


