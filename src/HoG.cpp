/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

/* 
// Author: Juergen Gall, BIWI, ETH Zurich
// Email: gall@vision.ee.ethz.ch
*/

#include <vector>
#include <iostream>
#include "HoG.h"

using namespace std;

HoG::HoG() {
	bins = 9;
	binsize = (3.14159265f*80.0f)/float(bins);;

	g_w = 5;
	Gauss = cvCreateMat( g_w, g_w, CV_32FC1 );
	double a = -(g_w-1)/2.0;
	double sigma2 = 2*(0.5*g_w)*(0.5*g_w);
	double count = 0;
	for(int x = 0; x<g_w; ++x) {
		for(int y = 0; y<g_w; ++y) {
			double tmp = exp(-( (a+x)*(a+x)+(a+y)*(a+y) )/sigma2);
			count += tmp;
			cvSet2D( Gauss, x, y, cvScalar(tmp) );
		}
	}
	cvConvertScale( Gauss, Gauss, 1.0/count);

	ptGauss = new float[g_w*g_w];
	int i = 0;
	for(int y = 0; y<g_w; ++y) 
		for(int x = 0; x<g_w; ++x)
			ptGauss[i++] = (float)cvmGet( Gauss, x, y );

}


void HoG::extractOBin(IplImage *Iorient, IplImage *Imagn, std::vector<IplImage*>& out, int off) {
	double* desc = new double[bins];

	// reset output image (border=0) and get pointers
	uchar** ptOut     = new uchar*[bins];
	uchar** ptOut_row = new uchar*[bins];
	for(int k=off; k<bins+off; ++k) {
		cvSetZero( out[k] );
		cvGetRawData( out[k], (uchar**)&(ptOut[k-off]));
	}

	// get pointers to orientation, magnitude
	int step;
	uchar* ptOrient;
	uchar* ptOrient_row;
	cvGetRawData( Iorient, (uchar**)&(ptOrient), &step);
	step /= sizeof(ptOrient[0]);

	uchar* ptMagn;
	uchar* ptMagn_row;
	cvGetRawData( Imagn, (uchar**)&(ptMagn));

	int off_w = int(g_w/2.0);   // 2 (g_w is local pixel window width == 5)
	for(int l=0; l<bins; ++l)
		ptOut[l] += off_w*step;

	for(int y=0;y<Iorient->height-g_w; y++, ptMagn+=step, ptOrient+=step) {

		// Get row pointers
		ptOrient_row = &ptOrient[0];
		ptMagn_row = &ptMagn[0];
		for(int l=0; l<bins; ++l)
			ptOut_row[l] = &ptOut[l][0]+off_w;

		for(int x=0; x<Iorient->width-g_w; ++x, ++ptOrient_row, ++ptMagn_row) {
		
			calcHoGBin( ptOrient_row, ptMagn_row, step, desc );

			for(int l=0; l<bins; ++l) {
				*ptOut_row[l] = (uchar)desc[l];
				++ptOut_row[l];
			}
		}

		// update pointer
		for(int l=0; l<bins; ++l)
			ptOut[l] += step;
	}

	delete[] desc;
	delete[] ptOut;
	delete[] ptOut_row;
}



