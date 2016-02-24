/*************************************************************************
* Data2Kml - Converting OBD-II/GPS logger data to KML (Google Earth)
* Distributed under GPL v2.0
* (c)2013 Written by Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "logdata.h"

typedef struct {
	uint32_t timestamp;
	float lat;
	float lng;
	uint16_t speed;
	uint16_t speedgps;
	uint16_t rpm;
	uint16_t throttle;
	int16_t coolant;
	int16_t intake;
	uint16_t load;
	uint16_t absload;
	int16_t alt;
	int16_t acc[3];
} DATASET;

typedef struct {
	int state;
	FILE* fp;
	HEADER hdr;
	char buffer[256];
	int bufbytes;
	DATASET* dataset;
	int datacount;
	float startLat;
	float startLng;
	uint32_t curDate;
	uint32_t curTime;
	uint32_t ts;
	uint32_t lastts;
	float lastLat;
	float lastLng;
	uint32_t lastTime;
	DATASET datas;
} KML_DATA;

uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (char n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		}
		else if (c >= 'a' && c <= 'f') {
			c -= 39;
		}
		else if (c == ' ') {
			continue;
		}
		else if (c < '0' || c > '9') {
			break;
		}
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

static int isFloat(const char* s)
{
	while (*s && *s != ',') {
		if (*s == '.')
			return 1;
		s++;
	}
	return 0;
}

void WriteKMLData(KML_DATA* kd, uint32_t timestamp, uint16_t pid, const char* value)
{
	kd->ts = timestamp;
	switch (pid) {
	case PID_GPS_LATITUDE:
		kd->datas.lat = isFloat(value) ? (float)atof(value) : (float)atoi(value) / 1000000;
		if (!kd->startLat) {
			kd->startLat = kd->datas.lat;
		}
		if (kd->datacount > 0) {
			float diff = kd->datas.lat - kd->dataset[kd->datacount - 1].lat;
			if (diff > 0.1 || diff < -0.1)
				kd->datas.lat = 0;	
		}
		break;
	case PID_GPS_LONGITUDE:
		kd->datas.lng = isFloat(value) ? (float)atof(value) : (float)atoi(value) / 1000000;
		if (!kd->startLng) {
			kd->startLng = kd->datas.lng;
		}
		if (kd->datacount > 0) {
			float diff = kd->datas.lng - kd->dataset[kd->datacount - 1].lng;
			if (diff > 0.1 || diff < -0.1)
				kd->datas.lng = 0;	
		}
		break;
	case PID_GPS_ALTITUDE:
		kd->datas.alt = atoi(value);
		break;
	case PID_SPEED:
		kd->datas.speed = (uint16_t)atoi(value);
		break;
	case PID_RPM:
		kd->datas.rpm = (uint16_t)atoi(value);
		break;
	case PID_THROTTLE:
		kd->datas.throttle = (uint16_t)atoi(value);
		break;
	case PID_COOLANT_TEMP:
		kd->datas.coolant = (int16_t)atoi(value);
		break;
	case PID_INTAKE_TEMP:
		kd->datas.intake = (int16_t)atoi(value);
		break;
	case PID_ENGINE_LOAD:
		kd->datas.load = (uint16_t)atoi(value);
		break;
	case PID_ABS_ENGINE_LOAD:
		kd->datas.absload = (uint16_t)atoi(value);
		break;
	case PID_GPS_SPEED:
		kd->datas.speedgps = (uint16_t)atoi(value);
		break;
	case PID_ACC:
		{
			const char *p = value;
			kd->datas.acc[0] = atoi(p);
			if (!(p = strchr(p, ','))) break;
			kd->datas.acc[1] = atoi(++p);
			if (!(p = strchr(p, ','))) break;
			kd->datas.acc[2] = atoi(++p);
		}
		break;
	case PID_GPS_DATE:
		kd->curDate = (uint32_t)atoi(value);
		break;
	case PID_GPS_TIME:
		kd->curTime = (uint32_t)atoi(value);
		break;
	}
	if (kd->curTime != kd->lastTime && kd->datas.lat && kd->datas.lng) {
		fprintf(kd->fp, "<when>");
		if (kd->curDate) {
			fprintf(kd->fp, "%04u-%02u-%02u", 2000 + (kd->curDate % 100), (kd->curDate / 100) % 100, kd->curDate / 10000);
		} else {
			time_t yesterday = time(0) - 86400;
			struct tm *btm = localtime(&yesterday);
			fprintf(kd->fp, "%04d-%02d-%02d", 1900+btm->tm_year, btm->tm_mon + 1, btm->tm_mday);
		}
		
		if (kd->curTime) {
			fprintf(kd->fp, "T%02u:%02u:%02u.%03uZ", kd->curTime / 1000000, (kd->curTime / 10000) % 100, (kd->curTime / 100) % 100, (kd->curTime % 100) * 10);
		}
		fprintf(kd->fp, "</when>");
		fprintf(kd->fp, "<gx:coord>%f %f %d</gx:coord>", kd->datas.lng, kd->datas.lat, kd->datas.alt);

		kd->datas.timestamp = timestamp;
		kd->dataset = (DATASET*)realloc(kd->dataset, sizeof(DATASET) * (kd->datacount + 1));
		memcpy(kd->dataset + kd->datacount, &kd->datas, sizeof(DATASET));
		kd->datacount++;

		kd->lastLat = kd->datas.lat;
		kd->lastLng = kd->datas.lng;
		kd->lastTime = kd->curTime;
	}
}

void AppendFile(FILE* fp, char* filename)
{
	int uint8_ts;
	char buffer[256];
    FILE* fpHeader = fopen(filename, "rb");
	if (!fpHeader) return;
	while ((uint8_ts = fread(buffer, 1, sizeof(buffer), fpHeader)) > 0) {
		fwrite(buffer, 1, uint8_ts, fp);
	}
	fclose(fpHeader);
}

void WriteKMLTail(KML_DATA* kd)
{
	int i;
	int lowThrottle = 50;
	printf("Generating extended data\n");

	fprintf(kd->fp, "<ExtendedData><SchemaData schemaUrl=\"#schema\">");
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"speed\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].speed);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");
	/*
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"speedgps\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].speedgps);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");
	*/
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"rpm\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].rpm);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"gear\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].speed ? kd->dataset[i].rpm / kd->dataset[i].speed : 1);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"coolant\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].coolant);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"intake\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].intake);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"load\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].load);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	/*
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"abs\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].absload);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");
	*/

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"thr\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].throttle);
		if (kd->dataset[i].speed == 0)
			lowThrottle = kd->dataset[i].throttle;
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"alt\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].alt);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");

	/*
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"sats\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%d</gx:value>", kd->dataset[i].sats);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");
	*/

	fprintf(kd->fp, "<gx:SimpleArrayData name=\"acc\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>X:%d Y:%d Z:%d</gx:value>", kd->dataset[i].acc[0], kd->dataset[i].acc[1], kd->dataset[i].acc[2]);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");	
	fprintf(kd->fp, "<gx:SimpleArrayData name=\"ts\">");
	for (i = 0; i < kd->datacount; i++) {
		fprintf(kd->fp, "<gx:value>%u</gx:value>", kd->dataset[i].timestamp);
	}
	fprintf(kd->fp, "</gx:SimpleArrayData>");	
	fprintf(kd->fp, "</SchemaData></ExtendedData>\r\n</gx:Track></Placemark>");

	int n = 0;
	for (i = 0; i < kd->datacount - 1; i++) {
		if (kd->dataset[i].speed < 25) {
			continue;
		}
		if (kd->dataset[i].throttle > lowThrottle + 2) {
			// throttle pedal is still down
			continue;
		}
		float g = 0;
		if (kd->dataset[i + 1].speed < kd->dataset[i].speed)
			g = (((float)kd->dataset[i + 1].speed - kd->dataset[i].speed) * 1000 / (kd->dataset[i + 1].timestamp - kd->dataset[i].timestamp) / 3.6) / 9.8f;
		else if (kd->dataset[i].speed < kd->dataset[i - 1].speed)
			g = (((float)kd->dataset[i].speed - kd->dataset[i - 1].speed) * 1000 / (kd->dataset[i].timestamp - kd->dataset[i - 1].timestamp) / 3.6) / 9.8f;
		else
			continue;

		// determine brake point
		if (g <= -0.2f) {
			n++;
			fprintf(kd->fp, "<Placemark><name>#%d %u:%02u</name>", n, kd->dataset[i].timestamp / 60000, (kd->dataset[i].timestamp / 1000) % 60);
			fprintf(kd->fp, "<styleUrl>#brakepoint</styleUrl><Point><coordinates>%f,%f</coordinates></Point>", kd->dataset[i].lng, kd->dataset[i].lat);
			fprintf(kd->fp, "<ExtendedData>");
			fprintf(kd->fp, "<Data name=\"Speed\"><value>%d</value></Data>", kd->dataset[i].speed);
			fprintf(kd->fp, "<Data name=\"RPM\"><value>%d</value></Data>", kd->dataset[i].rpm);
			fprintf(kd->fp, "<Data name=\"ACC\"><value>%.2fG</value></Data>", g);
			fprintf(kd->fp, "</ExtendedData>");
			fprintf(kd->fp, "</Placemark>\r\n");
			uint32_t t = kd->dataset[i].timestamp + 500;
			while (kd->dataset[++i].timestamp < t);
		}
	}
	fprintf(kd->fp, "</Folder></Document></kml>");

}

void CleanupKml(KML_DATA* kd)
{
	if (kd->dataset) free(kd->dataset);
	if (kd->fp) fclose(kd->fp);
	free(kd);
}

int ReadLine(FILE* fp, char* buf, int bufsize)
{
	int c;
	int n = 0;
	for (;;) {
		c = fgetc(fp);
		if (c == -1) break;
		if (c == '\r' || c == '\n') {
			if (n == 0)
				continue;
			else
				break;
		}
		if (n == bufsize - 1) break;
		buf[n++] = c;
	}
	buf[n] = 0;
	return n;
}

int ConvertToKML(const char* logfile, const char* kmlfile, uint32_t startpos, uint32_t endpos)
{
	FILE* fp = fopen(logfile, "r");
	if (!fp) {
		printf("Error opening file - %s\n", logfile);	
		return -1;
	}

	uint32_t ts = 0;
	KML_DATA* kd = (KML_DATA*)calloc(1, sizeof(KML_DATA));

	int elapsed;
	int pid;
	int count = 0;

	char buf[1024];
	while (ReadLine(fp, buf, sizeof(buf)) > 0) {
		if (buf[0] == '#') {
			// absolute timestamp
			ts = atoi(buf + 1);
		}
		else {
			ts += atoi(buf);
		}
		char *p = strchr(buf, ',');
		if (!p++) continue;
		pid = 0;
		if (*(p + 3) == ',') {
			if (!memcmp(p, "DTE", 3))
				pid = PID_GPS_DATE;
			else if (!memcmp(p, "UTC", 3))
				pid = PID_GPS_TIME;
			else if (!memcmp(p, "UTC", 3))
				pid = PID_GPS_TIME;
			else if (!memcmp(p, "LAT", 3))
				pid = PID_GPS_LATITUDE;
			else if (!memcmp(p, "LNG", 3))
				pid = PID_GPS_LONGITUDE;
			else if (!memcmp(p, "ALT", 3))
				pid = PID_GPS_ALTITUDE;
			else if (!memcmp(p, "SPD", 3))
				pid = PID_GPS_SPEED;
			else if (!memcmp(p, "CRS", 3))
				pid = PID_GPS_HEADING;
			else if (!memcmp(p, "SAT", 3))
				pid = PID_GPS_SAT_COUNT;
			else if (!memcmp(p, "ACC", 3))
				pid = PID_ACC;
			else if (!memcmp(p, "GYR", 3))
				pid = PID_GYRO;
			else if (!memcmp(p, "MAG", 3))
				pid = PID_COMPASS;
			else if (!memcmp(p, "BAT", 3))
				pid = PID_BATTERY_VOLTAGE;
		}
		if (pid == 0)
			pid = hex2uint16(p);

		p = strchr(p, ',');
		if (!p++) continue;

		char* value = p;
		printf("Time=%02u:%02u.%03u %X=%s\t\t\r", ts / 60000, (ts % 60000) / 1000, ts % 1000, pid, value);
		if (!kd->fp) {
			kd->fp = fopen(kmlfile, "w");
			//fprintf(kd->fp, "%s", kmlhead);
			AppendFile(kd->fp, "kmlhead.txt");
		}

		WriteKMLData(kd, ts, pid, value);
		count++;

		if (endpos && ts > endpos)
			break;
	}

	if (kd->fp) {
		WriteKMLTail(kd);
		CleanupKml(kd);
	}
	else {
		printf("No GPS data available in this file. KML not created.\n");
	}
	
	return 0;
}

int main(int argc, const char* argv[])
{
	int startpos = 0;
	int endpos = 0;
	char outfile[256];

	printf("Data2KML (C)2013-16 Written by Stanley Huang <http://freematics.com> \n\n");
	if (argc <= 1) {
		printf("Usage: %s [Input file] [Output file] [Start Pos] [End Pos]\n\n", argv[0]);
		printf("Description about the arguments:\n\n\
Input file: path to logged CSV file\n\
Output file: path to KML file (output in the input directory if unspecified)\n\
Start Pos: start time (seconds) for processing\n\
End Pos: end time (seconds) for processing\n");
		return -1;
	}

	if (argc > 3)
		startpos = (uint32_t)atoi(argv[3]) * 1000;
	if (argc > 4)
		endpos = (uint32_t)atoi(argv[4]) * 1000;

	_snprintf(outfile, sizeof(outfile), "%s.kml", argv[1]);

	ConvertToKML(argv[1], argc > 2 ? argv[2] : outfile, startpos, endpos);

	return 0;
}
