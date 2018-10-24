#define _GNU_SOURCE

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define DIM 100
#define NB_DATA 19
#define ELE_SIZE 3
#define EARTH_RADIUS 6371000
#define PI 3.14159265359
#define DEG_TO_RAD PI/180
#define ALPHA 1

void process_points(char point_1[NB_DATA][ELE_SIZE], char point_2[NB_DATA][ELE_SIZE], double **info) { //função para calcular distância entre 2 pontos consecutivos

	for (int i = 0; i < NB_DATA; i = i + 2) {
		printf("%s, %s ", point_1[i], point_2[i]);
		printf("\n");
	}
	double distance = 0;
	double phi_1 = atof(point_1[0]) + atof(point_1[2]) / 60 + atof(point_1[4]) / 3600;  //conversão de Degrees-Minutes-Seconds para graus

	if (strcmp(point_1[6], "S") == 0) {   //Latitude -> para Sul o ângulo varia entre 0 e -90
		phi_1 = -phi_1;
	}
	printf("PHI_1 %f\n", phi_1);
	double lambda_1 = atof(point_1[8]) + atof(point_1[10]) / 60 + atof(point_1[12]) / 3600;

	if (strcmp(point_1[14], "E") == 0) {  //Longitude -> para Este o ângulo varia entre 0 e -180
		lambda_1 = -lambda_1;
	}
	printf("LAMBDA_1 %f\n", lambda_1);
	double altitude = atof(point_1[16]); // A altitude a tomar em conta é a do 1º ponto do segmento
	double phi_2 = atof(point_2[0]) + atof(point_2[2]) / 60 + atof(point_2[4]) / 3600;

	if (strcmp(point_2[6], "S") == 0) {
		phi_2 = -phi_2;
	}
	printf("PHI_2 %f\n", phi_2);
	double lambda_2 = atof(point_2[8]) + atof(point_2[10]) / 60 + atof(point_2[12]) / 3600;

	if (strcmp(point_2[14], "E") == 0) {
		lambda_2 = -lambda_2;
	}

	printf("LAMBDA_2 %f\n", lambda_2);
	(*info)[0] = phi_1;
	(*info)[1] = lambda_1;
	(*info)[2] = phi_2;
	(*info)[3] = lambda_2;
	(*info)[4] = altitude;

	return;
}

double dist_btw_2points(double info[4]) { //função para calcular distância entre 2 pontos consecutivos
	double distance;
	distance = acos(sin(info[0]*DEG_TO_RAD)*sin(info[2]*DEG_TO_RAD) + cos(info[0]*DEG_TO_RAD)*cos(info[2]*DEG_TO_RAD)*cos(info[3]*DEG_TO_RAD - info[1] * DEG_TO_RAD)) * (info[4] + EARTH_RADIUS);  // great-circle 
																													  // distance
	// https://en.wikipedia.org/wiki/Great-circle_distance
	
	return distance;
}

double calculate_height_dev(double present_height, double final_height) {
	double height_dev = ALPHA * final_height - ALPHA * present_height;
	return height_dev;
}

double calculate_true_heading(double info[4]) {
	double y = sin(info[3] - info[1])*cos(info[2]);
	double x = (cos(info[0])*sin(info[2])) - (sin(info[0])*cos(info[2])*cos(info[3] - info[1]));
	double heading = atan2(y, x);
	return heading;
}

int main() {

	FILE *file;
	double route_distance = 0;
	double *info;
	char *ch, line[DIM], point_1[NB_DATA][ELE_SIZE], point_2[NB_DATA][ELE_SIZE];
	int i = 0, j = 0;

	file = fopen("waypoints.txt", "r"); // abrir ficheiro
	info = calloc(4, sizeof(double));

	if (file == NULL) {     // check if file was correctly opened
		printf("Error opening file");
		return 0;
	}

	while (fgets(line, DIM, file) != NULL) {  //condição do while lê linha a linha do ficheiro e passa a linha respetiva para a variável line
		j = 0;                               // o objetivo é pegar em dois consecutivos do ficheiro (seguir ordem) e calcular a distância 
		ch = strtok(line, " 'mkº/h");        // entre eles (a distância constante) e assim sucessivamente.  
											 // strtok elimina os caracteres indicados da string. fiz isso para depois meter tudo num vector 
											 //(neste caso, point_1 e point_2)                           
		while (ch != NULL) {
			if (*ch != '\n' && i == 0) {
				strcpy(point_1[j], ch);
				//printf("Point_1[%d] -> %s ", j, point_1[j]);
				j = j + 2;
			}

			else if (*ch != '\n' && i == 1) {
				strcpy(point_2[j], ch);
				//printf("Point_2[%d] -> %s ", j, point_2[j]);
				j = j + 2;
			}

			else if (*ch != '\n' && i > 1) {
				strcpy(point_1[j], point_2[j]);
				strcpy(point_2[j], ch);
				//printf("Point_%d[%d] -> %s ",i+1, j, point_2[j]);
				j = j + 2;
			}

			else {
				i++;
				//printf("\n");
			}
			ch = strtok(NULL, " 'mkº/h");
		}

		if (i == 2) {
			process_points(point_1, point_2, &info);
			route_distance = dist_btw_2points(info);
		}

		else if (i > 2) {
			process_points(point_1, point_2, &info);
			route_distance = route_distance + dist_btw_2points(info);
		}
	}

	printf("DISTANCE: %f m \n", route_distance);

	fclose(file);
	return 0;
}