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
	float lon;
	uint16_t speed;
	uint16_t speedgps;
	uint16_t rpm;
	uint16_t throttle;
	uint16_t coolant;
	uint16_t intake;
	uint16_t load;
	uint16_t absload;
	uint16_t alt;
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
	float startLon;
	uint32_t curDate;
	uint32_t curTime;
	uint32_t ts;
	uint32_t lastts;
	float lastLat;
	float lastLon;
	uint32_t lastTime;
	DATASET datas;
} KML_DATA;

void WriteKMLData(KML_DATA* kd, uint32_t timestamp, uint16_t pid, float value[])
{
	kd->ts = timestamp;
	switch (pid) {
	case PID_GPS_LATITUDE:
		kd->datas.lat = value[0] / 100000;
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
		kd->datas.lon = value[0] / 100000;
		if (!kd->startLon) {
			kd->startLon = kd->datas.lon;
		}
		if (kd->datacount > 0) {
			float diff = kd->datas.lon - kd->dataset[kd->datacount - 1].lon;
			if (diff > 0.1 || diff < -0.1)
				kd->datas.lon = 0;	
		}
		break;
	case PID_GPS_ALTITUDE:
		kd->datas.alt = (uint16_t)value[0];
		break;
	case PID_SPEED:
		kd->datas.speed = (uint16_t)value[0];
		break;
	case PID_RPM:
		kd->datas.rpm = (uint16_t)value[0];
		break;
	case PID_THROTTLE:
		kd->datas.throttle = (uint16_t)value[0];
		break;
	case PID_COOLANT_TEMP:
		kd->datas.coolant = (uint16_t)value[0];
		break;
	case PID_INTAKE_TEMP:
		kd->datas.intake = (uint16_t)value[0];
		break;
	case PID_ENGINE_LOAD:
		kd->datas.load = (uint16_t)value[0];
		break;
	case PID_ABS_ENGINE_LOAD:
		kd->datas.absload = (uint16_t)value[0];
		break;
	case PID_GPS_SPEED:
		kd->datas.speedgps = (uint16_t)value[0];
		break;
	case PID_ACC:
		kd->datas.acc[0] = (int16_t)value[0];
		kd->datas.acc[1] = (int16_t)value[1];
		kd->datas.acc[2] = (int16_t)value[2];
		break;
	case PID_GPS_DATE:
		kd->curDate = (uint32_t)value[0];
		break;
	case PID_GPS_TIME:
		kd->curTime = (uint32_t)value[0];
		break;
	}
	if (kd->curTime != kd->lastTime && kd->datas.lat && kd->datas.lon) {
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
		fprintf(kd->fp, "<gx:coord>%f %f %d</gx:coord>", kd->datas.lon, kd->datas.lat, kd->datas.alt);

		kd->datas.timestamp = timestamp;
		kd->dataset = (DATASET*)realloc(kd->dataset, sizeof(DATASET) * (kd->datacount + 1));
		memcpy(kd->dataset + kd->datacount, &kd->datas, sizeof(DATASET));
		kd->datacount++;

		kd->lastLat = kd->datas.lat;
		kd->lastLon = kd->datas.lon;
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
		fprintf(kd->fp, "<gx:value>%.2f</gx:value>", (float)kd->dataset[i].alt / 100);
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
		fprintf(kd->fp, "<gx:value>X:%d Y:%d Z:%d</gx:value>", kd->dataset[i].acc[0] / 64, kd->dataset[i].acc[1] / 64, kd->dataset[i].acc[2] / 64);
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

		if (g <= -0.15f) {
			n++;
			fprintf(kd->fp, "<Placemark><name>#%d %u:%02u</name>", n, kd->dataset[i].timestamp / 60000, (kd->dataset[i].timestamp / 1000) % 60);
			fprintf(kd->fp, "<styleUrl>#brakepoint</styleUrl><Point><coordinates>%f,%f</coordinates></Point>", kd->dataset[i].lon, kd->dataset[i].lat);
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

void Cleanup(KML_DATA* kd)
{
	if (kd->dataset) free(kd->dataset);
	if (kd->fp) fclose(kd->fp);
	free(kd);
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
	kd->fp = fopen(kmlfile, "w");
	
	//fprintf(kd->fp, "%s", kmlhead);
	AppendFile(kd->fp, "kmlhead.txt");	

	int elapsed;
	int pid;

	while (fscanf(fp, "%d,%X,", &elapsed, &pid) > 0) {
		char c;
		char data[64];
		int i = 0;
		int index = 0;
		float value[3] = {0};
		while ((c = fgetc(fp)) != '\r') {
			if (i == sizeof(data)) continue;
			data[i++] = c;
			if (c == ' ') {
				if (index < 2) {
					data[i] = 0;
					value[index++] = atof(data);
					i = 0;
				}
			}
		}
		data[i] = 0;
		value[index] = atof(data);
		ts += elapsed;
		if (ts < startpos)
			continue;

		printf("Time=%.1f PID=%X", (float)ts / 1000, pid);
		for (int n = 0; n <= index; n++) {
			if (value[n] == (int)value[n])
				printf(" D%d=%d", n, (int)value[n]);
			else
				printf(" D%d=%f", n, value[n]);
		}
		putchar('\n');
		WriteKMLData(kd, ts, pid, value);

		if (endpos && ts > endpos)
			break;
	}
	
	WriteKMLTail(kd);
	Cleanup(kd);
	fclose(fp);
	return 0;
}

int main(int argc, const char* argv[])
{
	int startpos = 0;
	int endpos = 0;
	char outfile[256];

	printf("Data2KML (C)2013-14 Written by Stanley Huang <http://freematics.com> \n\n");
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
