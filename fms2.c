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

double dist_btw_2points(char point_1[NB_DATA][ELE_SIZE], char point_2[NB_DATA][ELE_SIZE]) { //função para calcular distância entre 2 pontos consecutivos

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
	distance = acos(sin(phi_1*DEG_TO_RAD)*sin(phi_2*DEG_TO_RAD) + cos(phi_1*DEG_TO_RAD)*cos(phi_2*DEG_TO_RAD)*cos(lambda_2*DEG_TO_RAD - lambda_1 * DEG_TO_RAD)) * (altitude + EARTH_RADIUS);  // great-circle 
																														  // distance
	// https://en.wikipedia.org/wiki/Great-circle_distance

	return distance;
}

int main() {

	FILE *file;
	double route_distance = 0;
	char *ch, line[DIM], point_1[NB_DATA][ELE_SIZE], point_2[NB_DATA][ELE_SIZE];
	int i = 0, j = 0;

	file = fopen("waypoints.txt", "r"); // abrir ficheiro

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
				printf("Point_1[%d] -> %s ", j, point_1[j]);
				j = j + 2;
			}

			else if (*ch != '\n' && i == 1) {
				strcpy(point_2[j], ch);
				printf("Point_2[%d] -> %s ", j, point_2[j]);
				j = j + 2;
			}

			else if (*ch != '\n' && i > 1) {
				strcpy(point_1[j], point_2[j]);
				strcpy(point_2[j], ch);

				printf("Point_%d[%d] -> %s ",i+1, j, point_2[j]);
				j = j + 2;
			}

			else {
				i++;
				printf("\n");
			}
			ch = strtok(NULL, " 'mkº/h");
		}

		if (i == 2) {
			for (int i = 0; i < NB_DATA; i = i + 2) {
				printf("%s, %s ", point_1[i], point_2[i]);
				printf("\n");
			}
			route_distance = dist_btw_2points(point_1, point_2);
			return 0;
		}

		else if (i > 2) {
			route_distance = route_distance + dist_btw_2points(point_1, point_2);
		}
	}

	printf("DISTANCE: %f m \n", route_distance);

	fclose(file);
	return 0;
}