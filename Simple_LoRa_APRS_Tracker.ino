/* ========================================================================== *\
||		 Arduino with UART LoRa module as Simple APRS Tracker	      ||
||                                 Neo_Chen                                   ||
\* ========================================================================== */

/* ========================================================================== *\
|| Arduino with UART LoRa module as Simple APRS Tracker			      ||
|| Copyright (C) 2020 Kelei Chen					      ||
||									      ||
|| This program is free software: you can redistribute it and/or modify	      ||
|| it under the terms of the GNU General Public License as published by	      ||
|| the Free Software Foundation, either version 3 of the License, or	      ||
|| (at your option) any later version.					      ||
||									      ||
|| This program is distributed in the hope that it will be useful,	      ||
|| but WITHOUT ANY WARRANTY; without even the implied warranty of	      ||
|| MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the	      ||
|| GNU General Public License for more details.				      ||
||									      ||
|| You should have received a copy of the GNU General Public License	      ||
|| along with this program.  If not, see <https://www.gnu.org/licenses/>.     ||
\* ========================================================================== */

#include <stdint.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <limits.h>
#include <math.h>
#include <TinyGPS++.h>

/* Pin Configuration */
#define AUX	2
#define PPS	3

#define XCVR	Serial1
#define GPS	Serial3
#define DBG	Serial

/* AX.25 */
#define _AX25_FLAG	((byte)0x7E)
#define _AX25_CALLSIGN_LENGTH	6

#define _MAX_PACKET_SIZE	332

/* Callsign & SSID */
typedef struct
{
	byte callsign[_AX25_CALLSIGN_LENGTH + 1];
	byte ssid;
} id_t;

/* User configurable variables */
const unsigned int period = 30;

id_t callsigns[] =
{
	// Callsign, SSID
	{"APZ072", 2},	// Destination
	{"BX4ACV", 0},	// Source (replace this with your own callsign & SSID
	{"WIDE1", 1},	// First repeater
	{"WIDE2", 2}	// Second repeater (up to 8)
};

#define CALLSIGNS_NUM	(sizeof(callsigns) / sizeof(callsigns[0]))

const static char icon[] = "/$";
const static char comment[] = "SENT FROM BX4ACV'S HOMEMADE TRACKER";
TinyGPSPlus gps;

unsigned int volatile timer = period;

void pps(void);

void setup(void)
{
	DBG.begin(115200);
	XCVR.begin(9600);
	GPS.begin(9600);

	pinMode(AUX, INPUT_PULLUP);
	pinMode(PPS, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(PPS), pps, RISING);

	DBG.println("INIT DONE!");
}

/* Idle spinning */

void spin(void)
{
	digitalWrite(LED_BUILTIN, digitalRead(AUX));
}

enum wait_type
{
	WAIT_GPS,
	WAIT_XCVR,
	WAIT_AUX
};

/* Waiting for data or operation completion */
void wait(int type)
{
	switch(type)
	{
		case WAIT_AUX:
			while(digitalRead(AUX) == LOW)
				spin();
			break;
		case WAIT_XCVR:
			while(XCVR.available() <= 0)
				spin();
			break;
		case WAIT_GPS:
			while(GPS.available() <= 0)
				spin();
			break;
		default:
			break;
	}
}

#define CONTROL_FIELD	0x3
#define PROTOCOL_ID	0xF0

void construct_packet(uint8_t *packet, unsigned int *len)
{
	unsigned int i = 0, j = 0;
	(*len) = 0;
	memset(packet, 0x00, _MAX_PACKET_SIZE);

	packet[(*len)++] = 0x7E;

	for(j = 0; j < CALLSIGNS_NUM; j++)
	{
		for(i = 0; i < 7; i++)
		{
			packet[(*len)++] = callsigns[j].callsign[i] << 1;
			if(callsigns[j].callsign[i] == '\0')
				packet[(*len) - 1] = '\x20' << 1;
			if(i == 6)
				packet[(*len) - 1] = ((j >= 2 ? 0x30 : 0x70) + callsigns[j].ssid) << 1;
		}
	}

	packet[(*len) - 1]++;

	packet[(*len)++] = CONTROL_FIELD;
	packet[(*len)++] = PROTOCOL_ID;

	double lat_f = gps.location.lat();
	double lng_f = gps.location.lng();

	char lat_cardinal = signbit(lat_f) ? 'S' : 'N';
	char lng_cardinal = signbit(lng_f) ? 'W' : 'E';

	long lat_decmin = lround(fabs(lat_f) * 6000.0);
	long lng_decmin = lround(fabs(lng_f) * 6000.0);

	long lat_min = lat_decmin / 100;
	long lng_min = lng_decmin / 100;

	long lat_deg = lat_min / 60;
	long lng_deg = lng_min / 60;

	lat_decmin %= 100;
	lng_decmin %= 100;

	lat_min %= 60;
	lng_min %= 60;

	char pos[64];

	sprintf(pos, "!%02ld%02ld.%02ld%c%c%03ld%02ld.%02ld%c%c%03d/%03d%s /A=%06ld",
		lat_deg, lat_min, lat_decmin, lat_cardinal,
		icon[0],
		lng_deg, lng_min, lng_decmin, lng_cardinal,
		icon[1],
		gps.course.isValid() ? (int)gps.course.deg() : 0,
		gps.speed.isValid() ? (int)gps.speed.knots() : 0,
		comment,
		gps.altitude.isValid() ? (long)gps.altitude.feet() : 0);

	DBG.println(pos);
	strcpy((char *)packet + (*len), pos);
	(*len) += strlen(pos);

	packet[(*len)++] = 0x7E;

	return;
}

long unsigned lastmillis = 0;

void loop()
{
	uint8_t packet[_MAX_PACKET_SIZE];
	unsigned int len = 0;
	char c;

	/* Read GPS data */
	if(GPS.available() > 0)
	{
		c = GPS.read();
		gps.encode(c);
		//DBG.write(c);
	}

	if(timer == 0)	/* Start transmitting APRS packet */
	{
		DBG.println("Time to transmit.");
		if(gps.location.isValid())
		{
			DBG.println("GPS Valid");
			construct_packet(packet, &len);
			DBG.println("Packet Constructed");
			XCVR.write(packet, len);
			DBG.println("Packet Sent");
			wait(WAIT_AUX);
			DBG.println("transmission complete.");
		}
		timer = UINT_MAX;	/* Reset timer */
	}

	spin();
}

void pps(void)
{
	timer--;
	if(timer >= period)
		timer = period;

	return;
}
