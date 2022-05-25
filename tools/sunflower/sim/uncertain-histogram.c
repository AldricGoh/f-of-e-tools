/*
	Copyright (c)	2019, Jan Heck (author), based on work from
	M.Eng. project of Alexa Belsham.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <string.h>
#include "sf.h"


/*
 *	Add two distributions, considering overflow.
 */
void
Histogram_AddDist(Engine *E, State *S, Histogram *hist1, Histogram *hist2, Histogram *histDest)
{
	uint16_t	overflow_wid = 0;
	uint32_t	overflow_hi = 0;

	/*
	 *	Zero-out destination histogram
	 */
	for (int k = 0; k < kUncertainAluHistogramBins; k++)
	{
		histDest->bins[k] = 0;
	}

	/*
	 *	Iterate, adding with overflow
	 */
	for (int j = 0; j < kUncertainAluHistogramBins; j++)\
	{
		for (int i = 0; i < kUncertainAluHistogramBins; i++)
		{
			overflow_wid = i+j;

			if (overflow_wid < kUncertainAluHistogramBins)
			{
				overflow_hi = histDest->bins[i+j] + (uint32_t)((uint32_t)hist1->bins[i] * hist2->bins[j]);

				if (overflow_hi < 65536)
				{
					histDest->bins[i+j] += hist1->bins[i] * hist2->bins[j];
				}
				else
				{
					/*
					 *	Bin overflow error
					 */
					mprint(E, S, nodeinfo, "WARN: encountered bin overflow in histogram operation\n");
				}
			}
			else
			{
				/*
				 *	Value overflow error
				 */
				mprint(E, S, nodeinfo, "WARN: encountered value overflow in histogram operation\n");
			}
		}
	}

	return;
}


/*
 *	Multiply each bin with a scalar
 */
void
Histogram_ScalarMultiply(Engine *E, State *S, Histogram *hist, HistogramBinDatatype scalar)
{
	for (int k = 0; k < kUncertainAluHistogramBins; k++)
	{
		hist->bins[k] = scalar * hist->bins[k];
	}

	return;
}


/*
 *	Subtract two distributions, considering overflow
 */
void
Histogram_SubDist(Engine *E, State *S, Histogram *hist1, Histogram *hist2, Histogram *histDest)
{
	/*
	 *	We want to reuse the code for AddDist. To do this, we transform in-place as follows:
	 */


	/*
	 *	Negate one of the histograms (pick hist2)
	 */
	Histogram_ScalarMultiply(E, S, hist2, -1);

	/*
	 *	Add together to give the result
	 */
	Histogram_AddDist(E, S, hist1, hist2, histDest);

	/*
	 *	Undo the change to hist2
	 */
	Histogram_ScalarMultiply(E, S, hist2, -1);


	/*
	 *	Caveat: Is the above equivalent to the following code in all cases?
	 *
	 UInt uncertain_SubDist(UInt Urs1, UInt Urs2, UInt Ud){ //Subtracts distributions

		uint32_t overflow_hi = 0;


		int k = 0;
		int j = 0;
		int i = 0;

		// Empty Ud (just in case)
		for(k = 0; k < 256; k++){
			Ud.weights[i] = 0;
		}


		for (j = 0; j < 256; j++){
			for(i=0; i<256; i++){
				if(i >= j){

					overflow_hi = Ud.weights[i - j] + (uint32_t)((uint32_t)Urs1.weights[i] * Urs2.weights[j]); //Nasty casting here

					if(overflow_hi < 65536){
						Ud.weights[i - j] += Urs1.weights[i] * Urs2.weights[j];

					}else{
						//Bin overflow error
					}
				}else{
					//Value underflow error
				}
			}
		}

			return Ud;
		}
	*/

	return;
}


/*
 *	Add two distograms in the simple fashion of adding corresponding bins together
 */
void
Histogram_CombDist(Engine *E, State *S, Histogram *hist1, Histogram *hist2, Histogram *histDest)
{
	for (int k = 0; k < kUncertainAluHistogramBins; k++)
	{
		histDest->bins[k] = hist1->bins[k] * hist2->bins[k];
	}

	return;
}


/*
 *	Returns the lower bound of the distribution, i.e. the bin index of the lowermost non-zero bin.
 *
 *	Returns -1 if all bins are empty.
 */
int
Histogram_LowerBound(Engine *E, State *S, Histogram *hist)
{
	for (int k = 0; k < kUncertainAluHistogramBins; k++)
	{
		if (hist->bins[k] != 0)
		{
			return k;
		}
	}

	return -1;
}


/*
 *	Returns the upper bound of the distribution, i.e. the bin index of the uppermost non-zero bin.
 *
 *	Returns -1 if all bins are empty.
 */
int
Histogram_UpperBound(Engine *E, State *S, Histogram *hist)
{
	for (int k = kUncertainAluHistogramBins-1; k >= 0; k--)
	{
		if (hist->bins[k] != 0)
		{
			return k;
		}
	}

	return -1;
}


/*
 *	DistLShift shifts the heights of the bins left by rs2, effectively subtracting rs2 from each bin.
 *	This reduces the mean of the distribution by rs2.
 */
void
Histogram_DistLShift(Engine *E, State *S, Histogram *hist1, uint8_t Rs2, Histogram *histDest)
{
	int	k = 0;
	int	i = 0;

	for(k = 0; k < 256; k++)
	{
		histDest->bins[k] = 0;
	}

	for(i = 0; i < 256; i++)
	{
		if(i >= Rs2)
		{
			histDest->bins[i-Rs2] = hist1->bins[i];
		}
	}

	return;
}


/*
 *	DistRShift shifts the heights of the bins right by rs2, effectively adding rs2 to each bin.
 *	This increases the mean of the distribution by rs2.
 */
void
Histogram_DistRShift(Engine *E, State *S, Histogram *hist1, uint8_t Rs2, Histogram *histDest)
{
	int		k = 0;
	int		i = 0;
	uint16_t	overflow_wid = 0;

	for(k = 0; k < 256; k++)
	{
		histDest->bins[k] = 0;
	}

	for(i = 0; i < 256; i++)
	{
		overflow_wid = (int16_t) (i + Rs2);
		if(overflow_wid < 256)
		{
			histDest->bins[i+Rs2] = hist1->bins[i];
		}
	}

	return;
}


/*
 *	Exp returns the expected value of the histogram. This provides an estimate of the 
 *	variable and allows the histogram to be converted into a point value.
 */
uint8_t
Histogram_ExpectedValue(Engine *E, State *S, Histogram *hist)
{
	int	sum = 0;
	int	n = 0;
	int	mean = 0;
	int	i = 0;
	uint8_t	Rd = 0;

	for(i = 0; i<256; i++)
	{
			sum += ((int)hist->bins[i] * i);
			n += (int)hist->bins[i];
	}

	mean = sum / n;
	Rd = (uint8_t) mean;

	return Rd;
}


/*
 *	DistLess returns the probability Pr(X < Rs2). 
 *	X is a discrete random variable distributed according to the relative frequencies of hist1. 
 *	The probability is returned as an unsigned integer between 0 and 100 representing a percentage. 
 *	It is expected that this instruction will often be followed by one of the branch instructions 
 *	in the base instruction set.
 */
uint32_t
Histogram_DistLess(Engine *E, State *S, Histogram *hist, uint32_t Rs2)
{
	int		i = 0;
	int		num = 0;
	int		denom = 0;
	uint32_t	outcome = 0;
	uint32_t	Rd = 0;

	for(i = 0; i < 256; i++)
	{
			if(i < Rs2)
			{
					num = num + hist->bins[i]; //If it is less then
			}
			denom = denom + hist->bins[i];
	}

	if(denom!=0)
	{
			outcome = (num * 100) / denom; //Note this is integer division. Note that the times by 100 is to make it a percent rather than decimal which cannot be stored easily
			Rd = outcome;
			return Rd;
	}
	else
	{
			Rd = -1;
			return Rd;
	}
}


/*
 *	DistLess returns the probability Pr(X >= Rs2). 
 *	X is a discrete random variable distributed according to the relative frequencies of hist1. 
 *	The probability is returned as an unsigned integer between 0 and 100 representing a percentage. 
 *	It is expected that this instruction will often be followed by one of the branch instructions 
 *	in the base instruction set.
 */
uint32_t
Histogram_DistGrt(Engine *E, State *S, Histogram *hist, uint32_t Rs2)
{
	int		i = 0;
	int		num = 0;
	int		denom = 0;
	uint32_t	outcome = 0;
	uint32_t	Rd = 0;

	for(i = 0; i < 256; i++)
	{
			if(i >= Rs2)
			{
					num = num + hist->bins[i]; //If it is less then
			}
			denom = denom + hist->bins[i];
	}

	if(denom!=0)
	{
			/*
			 *	Note this is integer division. Note that the times by 100 is
			 *	to make it a percent rather than decimal which cannot be stored
			 *	easily.
			 */
			outcome = (num * 100) / denom;
			Rd = outcome;

			return Rd;
	}
	else
	{
			Rd = -1;

			return Rd;
	}
}


/*
 *	Load a kUncertainAluHistogramBins-sized array of HistogramBinDatatype into the Histogram class
 */
void
Histogram_LDDist(Engine *E, State *S, Histogram *histogram, HistogramBinDatatype *bins)
{
	memcpy(histogram->bins, bins, sizeof(HistogramBinDatatype)*kUncertainAluHistogramBins);

	return;
}


/*
 *	Initialise *histogram with random values in each bin
 */
void
Histogram_LDRandom(Engine *E, State *S, Histogram *histogram)
{
	/*
	 *	Create array
	 */
	HistogramBinDatatype array[kUncertainAluHistogramBins] = {};
	for (int i = 0; i < kUncertainAluHistogramBins; i++)
	{
		array[i] = (rand()/(double)RAND_MAX) * 255;
		/*
		 *  Picked some reasonable max value allowing by-eye debugging, increase to data type maximum later:
		 *	array[i] = (rand()/(double)RAND_MAX) * ((HistogramBinDatatype)~(HistogramBinDatatype)0);
		 *	The final expression finds the maximum value this datatype can take
		 */
	}

	/*
	 *	Load into histogram
	 */
	Histogram_LDDist(E, S, histogram, array);

	return;
}


/*
 *	Return the mean frequency of a histogram, i.e. the average bin value (not weighted by index)
 */
double
Histogram_MeanFrequency(Engine *E, State *S, Histogram *histogram)
{
	double sum = 0;

	for (int i = 0; i < kUncertainAluHistogramBins; i++){
		sum += histogram->bins[i];
	}

	return sum / (double)kUncertainAluHistogramBins;
}


/*
 *	Pretty-print ("ASCII-graph") a normalised histogram representation, like so:
 *
 *	+-----> bin value
 *	|
 *	| #
 *	| ##
 *	| ###
 *	| ##
 *	| #
 *	|
 *	V bin index
 *
 */
void
Histogram_PrettyPrint(Engine *E, State *S, Histogram *histogram)
{
	double	normalised[kUncertainAluHistogramBins] = {};
	double	meanFreq = Histogram_MeanFrequency(E, S, histogram);

	/*HistogramBinDatatype FULLSCALE = (HistogramBinDatatype)~(HistogramBinDatatype)0;*/
	// This expression finds the maximum value this datatype can take
	
	/*
	 *	Alternatively, auto-scaling could be done by the following:
	 */
	double FULLSCALE = 3 * meanFreq;

	const int FULLSCALE_NUMBER_OF_CHARS = 40;

	for (int i = 0; i < kUncertainAluHistogramBins; i++)
	{
		normalised[i] = histogram->bins[i] / FULLSCALE;
	}

	mprint(E, S, nodeinfo, "Histogram mean frequency (mean bin occupation): %.3f\n", meanFreq);
	mprint(E, S, nodeinfo, "bin | value | graphical representation (scaled rel. to mean freq)\n");
	mprint(E, S, nodeinfo, "----+-------+----------------------------------------------------\n");

	for (int i = 0; i < kUncertainAluHistogramBins; i++)
	{
		mprint(E, S, nodeinfo, "%03u | %-5u | ", i, histogram->bins[i]);
		/*mprint(E, S, nodeinfo, "%f\n", (normalised[i]));*/
		/*mprint(E, S, nodeinfo, "%f\n", (normalised[i]*FULLSCALE_NUMBER_OF_CHARS));*/
		for (int j = 0; j < (int)(normalised[i]*FULLSCALE_NUMBER_OF_CHARS); j++){
			mprint(E, S, nodeinfo, "#");
		}
		mprint(E, S, nodeinfo, "\n");
	}

	return;
}
