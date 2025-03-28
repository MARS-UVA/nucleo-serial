/*
 * PDP.c
 *
 *  Created on: Feb 28, 2025
 *      Author: diana
 */

#include "PDP.h"

void requestCurrentReadingsPDP(PDP *pdp)
{
	sendCANMessage(pdp->hcan, 0x8041640 | pdp->identifier, "\x00\x00\x00\x00\x20\x00", 6);
}


void getSixParamPDP(PDP* pdp, uint64_t *cache) {
	for (int i = 0; i < 5; i++)
	{
		if (pdp->receivedNew)
			break;

		HAL_Delay(10);
	}


	pdp->cacheWords[0] = (uint8_t) *cache;

	short* numPtr1 = pdp->cacheWords;
	numPtr1[0] = (short) (numPtr1[0] << 2);

	short* numPtr2 = pdp->cacheWords;
	numPtr2[0] = (short) (numPtr2[0] | ((short) ((*cache >> 14) & ((unsigned long) 3))));
	pdp->cacheWords[1] = (short) ((*cache >> 8) & 0x3f);

	short* numPtr3 = &(pdp->cacheWords[1]);
	numPtr3[0] = (short) (numPtr3[0] << 4);

	short* numPtr4 = &(pdp->cacheWords[1]);
	numPtr4[0] = (short) (numPtr4[0] | ((short) ((*cache >> 20) & 15)));
	pdp->cacheWords[2] = (short) ((*cache >> 0x10) & 15);

	short* numPtr5 = &(pdp->cacheWords[2]);
	numPtr5[0] = (short) (numPtr5[0] << 6);

	short* numPtr6 = &(pdp->cacheWords[2]);
	numPtr6[0] = (short) (numPtr6[0] | ((short) ((*cache >> 0x1a) & 0x3f)));
	pdp->cacheWords[3] = (short) ((*cache >> 0x18) & ((unsigned long) 3));

	short* numPtr7 = &(pdp->cacheWords[3]);
	numPtr7[0] = (short) (numPtr7[0] << 8);

	short* numPtr8 = &(pdp->cacheWords[3]);
	numPtr8[0] = (short) (numPtr8[0] | ((uint8_t) (*cache >> 0x20)));
	pdp->cacheWords[4] = (*cache >> 40);

	short* numPtr9 = &(pdp->cacheWords[4]);
	numPtr9[0] = (short) (numPtr9[0] << 2);

	short* numPtr10 = &(pdp->cacheWords[4]);
	numPtr10[0] = (short) (numPtr10[0] | ((short) ((*cache >> 0x36) & ((unsigned long) 3))));
	pdp->cacheWords[5] = (short) ((*cache >> 0x30) & 0x3f);

	short* numPtr11 = &(pdp->cacheWords[5]);
	numPtr11[0] = (short) (numPtr11[0] << 4);

	short* numPtr12 = &(pdp->cacheWords[5]);
	numPtr12[0] = (short) (numPtr12[0] | ((short) ((*cache >> 60) & 15)));
}

// given PDP channel ID, returns the current in Amps at the channel
float getChannelCurrentPDP(PDP* pdp, int channelID)
{
	pdp->requestCurrentReadings(pdp);

	float num = 0;
	if (channelID >= 0 && channelID <= 5) {
		pdp->getSixParam(pdp, &(pdp->cache0));
		num = pdp->cacheWords[channelID] * 0.125;
	} else if (channelID >= 6 && channelID <= 11) {
		pdp->getSixParam(pdp, &(pdp->cache40));
		num = pdp->cacheWords[channelID - 6] * 0.125;
	} else {
		pdp->getSixParam(pdp, &(pdp->cache80));
		num = pdp->cacheWords[channelID - 12] * 0.125;
	}

	pdp->receivedNew = false;

	return num;
}

void receiveCANPDP(PDP *pdp, CAN_RxHeaderTypeDef *msg, uint64_t *data)
{
	  // not a pdp
	  if ((msg->ExtId & 0x8041400) != 0x8041400)
		  return;

	  // not correct pdp id
	  if ((msg->ExtId & pdp->identifier) != pdp->identifier)
		  return;

	  switch (msg->ExtId & 0xc0)
	  {
	  case 0x00:
		  pdp->cache0 = *data;
		  break;
	  case 0x40:
		  pdp->cache40 = *data;
		  break;
	  case 0x80:
		  pdp->cache80 = *data;
		  break;
	  }

	  pdp->receivedNew = true;
}

PDP PDPInit(CAN_HandleTypeDef *hcan, int32_t identifier)
{
	PDP pdp = {
		.hcan = hcan,
		.identifier = identifier,
		.cache0 = 0,
		.cache40 = 0,
		.cache80 = 0,
		.cacheWords = {0},
		.getChannelCurrent = getChannelCurrentPDP,
		.getSixParam = getSixParamPDP,
		.requestCurrentReadings = requestCurrentReadingsPDP,
		.receiveCAN = receiveCANPDP,
		.receivedNew = false
	};

	return pdp;
}
