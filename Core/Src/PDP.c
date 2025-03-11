/*
 * PDP.c
 *
 *  Created on: Feb 28, 2025
 *      Author: diana
 */

#include "PDP.h"

void requestCurrentReadings(PDP *pdp)
{
	sendCANMessage(pdp->hcan, 0x8041640 | pdp->identifier, "\x00\x00\x00\x00\x20\x00", 6);
}


void getSixParam(PDP* pdp, int arbID) {
	requestCurrentReadings(pdp);
	CANPacket packet;
	// TODO: break out after waiting; has potential to make code hang indefinitely
	while ((packet = receiveCAN(0x8041400)).data == NULL);
	pdp->cache = *packet.data;
	pdp->cacheWords[0] = (uint8_t) pdp->cache;

	short* numPtr1 = pdp->cacheWords;
	numPtr1[0] = (short) (numPtr1[0] << 2);

	short* numPtr2 = pdp->cacheWords;
	numPtr2[0] = (short) (numPtr2[0] | ((short) ((pdp->cache >> 14) & ((unsigned long) 3))));
	pdp->cacheWords[1] = (short) ((pdp->cache >> 8) & 0x3f);

	short* numPtr3 = &(pdp->cacheWords[1]);
	numPtr3[0] = (short) (numPtr3[0] << 4);

	short* numPtr4 = &(pdp->cacheWords[1]);
	numPtr4[0] = (short) (numPtr4[0] | ((short) ((pdp->cache >> 20) & 15)));
	pdp->cacheWords[2] = (short) ((pdp->cache >> 0x10) & 15);

	short* numPtr5 = &(pdp->cacheWords[2]);
	numPtr5[0] = (short) (numPtr5[0] << 6);

	short* numPtr6 = &(pdp->cacheWords[2]);
	numPtr6[0] = (short) (numPtr6[0] | ((short) ((pdp->cache >> 0x1a) & 0x3f)));
	pdp->cacheWords[3] = (short) ((pdp->cache >> 0x18) & ((unsigned long) 3));

	short* numPtr7 = &(pdp->cacheWords[3]);
	numPtr7[0] = (short) (numPtr7[0] << 8);

	short* numPtr8 = &(pdp->cacheWords[3]);
	numPtr8[0] = (short) (numPtr8[0] | ((uint8_t) (pdp->cache >> 0x20)));
	pdp->cacheWords[4] = (pdp->cache >> 40);

	short* numPtr9 = &(pdp->cacheWords[4]);
	numPtr9[0] = (short) (numPtr9[0] << 2);

	short* numPtr10 = &(pdp->cacheWords[4]);
	numPtr10[0] = (short) (numPtr10[0] | ((short) ((pdp->cache >> 0x36) & ((unsigned long) 3))));
	pdp->cacheWords[5] = (short) ((pdp->cache >> 0x30) & 0x3f);

	short* numPtr11 = &(pdp->cacheWords[5]);
	numPtr11[0] = (short) (numPtr11[0] << 4);

	short* numPtr12 = &(pdp->cacheWords[5]);
	numPtr12[0] = (short) (numPtr12[0] | ((short) ((pdp->cache >> 60) & 15)));
}

float getChannelCurrent(PDP* pdp, int channelID)
{
	float num = 0;
	if (channelID >= 0 && channelID <= 5) {
		getSixParam(pdp, 0x8041400);
		num = pdp->cacheWords[channelID] * 0.125;
	} else if (channelID >= 6 && channelID <= 11) {
		getSixParam(pdp, 0x8041440);
		num = pdp->cacheWords[channelID - 6] * 0.125;
	} else {
		getSixParam(pdp, 0x8041480);
		num = pdp->cacheWords[channelID - 12] * 0.125;
	}
	return num;
}

