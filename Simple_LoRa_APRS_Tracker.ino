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
#define _AX25_CALLSIGN_LENGTH	7

#define _MAX_PACKET_SIZE	332

/* User configurable variables */
byte src_callsign[_AX25_CALLSIGN_LENGTH] = "BX4ACV\x70";
byte dst_callsign[_AX25_CALLSIGN_LENGTH] = "APZ072\x72";

/* NOTE: The LSB of SSID needs to be 1 */
byte digi_callsign[][_AX25_CALLSIGN_LENGTH] =
{
	"WIDE1 \x31",
	"WIDE2 \x32"
};

char icon[] = "/$";

char comment[] = "SENT FROM BX4ACV'S TRACKER";

TinyGPSPlus gps;

unsigned int volatile timer = UINT_MAX;

void pps(void);

void setup(void)
{
	DBG.begin(115200);
	XCVR.begin(9600);
	GPS.begin(9600);

	pinMode(AUX, INPUT_PULLUP);
	pinMode(PPS, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);

	//attachInterrupt(digitalPinToInterrupt(PPS), pps, RISING);
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

void construct_packet(uint8_t packet[], unsigned int *len)
{
	int i = 0, j = 0;
	(*len) = 0;
	memset(&packet, 0x00, _MAX_PACKET_SIZE);
	DBG.println("Memset");

	packet[(*len)++] = 0x7E;

	/* Calculate & copy real callsign data */
	for(i = 0; i < 7; i++)
	{
		packet[(*len)++] = dst_callsign[i] << 1;
	}
	DBG.println("dst callsign");

	for(i = 0; i < 7; i++)
	{
		packet[(*len)++] = src_callsign[i] << 1;
	}
	DBG.println("src callsign");

	for(j = 0; j < 2; j++)
	{
		for(i = 0; i < 7; i++)
		{
			packet[(*len)++] = digi_callsign[j][i];
			if(i == 6)
				packet[(*len) - 1]++;
		}
		DBG.println("digi callsign");
	}

	packet[(*len)++] = CONTROL_FIELD;
	packet[(*len)++] = PROTOCOL_ID;

	double lat = gps.location.lat();
	double lng = gps.location.lng();
	DBG.println("gps get location");

	char lat_cardinal = lat > 0 ? 'N' : 'S';
	char lng_cardinal = lng > 0 ? 'E' : 'W';

	lat = fabs(lat);
	lng = fabs(lng);

	double lat_min = modf(lat, &lat) * 60;
	double lng_min = modf(lng, &lng) * 60;

	double lat_sec = modf(lat_min, &lat_min) * 60;
	double lng_sec = modf(lng_min, &lng_min) * 60;

	lat_sec = floor(lat_sec);
	lng_sec = floor(lng_sec);

	DBG.println("location unit conversion complete");

	char pos[64];

	sprintf(pos, "!%2.0f%2.0f.%2.0f%c%c%3.0f%2.0f.%2.0f%c%c%3.0f/%3.0f%s /A=%6.0f",
		lat, lat_min, lat_sec, lat_cardinal,
		icon[0],
		lng, lng_min, lng_sec, lng_cardinal,
		icon[1],
		gps.course.isValid() ? gps.course.deg() : 0.0,
		gps.speed.isValid() ? gps.speed.knots() : 0.0,
		comment,
		gps.altitude.isValid() ? gps.altitude.feet() : 0.0);

	DBG.println(pos);
	strcpy(packet, pos);
	(*len) += strlen(pos);

	packet[(*len)++] = 0x7E;

	return;
}

long unsigned lastmillis = 0;

void loop()
{
	uint8_t packet[_MAX_PACKET_SIZE];
	unsigned int len = 0;
	byte c;

	/* Debu */

	if((millis() - lastmillis) >= 10*1000)
	{
		DBG.println("Timer triggered!");
		timer = 0;
		lastmillis = millis();
	}

	/* Read GPS data */
	if(GPS.available() > 0)
	{
		c = GPS.read();
		gps.encode(c);
		// DBG.write(c);
	}

	if(timer == 0)	/* Start transmitting APRS packet */
	{
		DBG.println("It's time to transmit!");
		if(gps.location.isValid())
		{
			DBG.println("GPS Valid");
			construct_packet(packet, &len);
			DBG.println("Packet Constructed");
			XCVR.write(packet, len);
			DBG.println("Packet Sent");
			wait(WAIT_AUX);
			DBG.println("Waiting for transmission complete.");
		}
		timer = UINT_MAX;	/* Reset timer */
	}

	spin();
}

void pps(void)
{
	timer--;
	timer &= 0x1F;
}
