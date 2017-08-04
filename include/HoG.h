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

#pragma once

#include <cxcore.h>

#include <vector>

class HoG {
public:
	HoG();
	~HoG() {cvReleaseMat(&Gauss);delete ptGauss;}
	void extractOBin(IplImage *Iorient, IplImage *Imagn, std::vector<IplImage*>& out, int off);
private:

	void calcHoGBin(uchar* ptOrient, uchar* ptMagn, int step, double* desc);
	void binning(float v, float w, double* desc, int maxb);

	int bins;
	float binsize; 

	int g_w;
	CvMat* Gauss;

	// Gauss as vector
	float* ptGauss;
};

inline void HoG::calcHoGBin(uchar* ptOrient, uchar* ptMagn, int step, double* desc) {
	for(int i=0; i<bins;i++)
		desc[i]=0;

	uchar* ptO = &ptOrient[0];
	uchar* ptM = &ptMagn[0];
	int i=0;
	for(int y=0;y<g_w; ++y, ptO+=step, ptM+=step) {
		for(int x=0;x<g_w; ++x, ++i) {
			binning((float)ptO[x]/binsize, (float)ptM[x] * ptGauss[i], desc, bins);
		}
	}
}

inline void HoG::binning(float v, float w, double* desc, int maxb) {
	int bin1 = int(v);
	int bin2;
	float delta = v-bin1-0.5f;
	if(delta<0) {
		bin2 = bin1 < 1 ? maxb-1 : bin1-1; 
		delta = -delta;
	} else
		bin2 = bin1 < maxb-1 ? bin1+1 : 0; 
	desc[bin1] += (1-delta)*w;
	desc[bin2] += delta*w;
}

