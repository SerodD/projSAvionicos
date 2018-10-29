#define _GNU_SOURCE

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define DIM 100
#define NB_DATA 10
#define ELE_SIZE 10
#define EARTH_RADIUS 6371000
#define PI 3.14159265359
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI
#define ALPHA 0.001
#define KMH_TO_MS 1000/3600
#define TIME_ACEL 180
#define T 20*60

void process_points(char point_1[NB_DATA][ELE_SIZE], char point_2[NB_DATA][ELE_SIZE], double **info) { //função para calcular distância entre 2 pontos consecutivos
	for (int i = 0; i < NB_DATA; i = i + 1) {
		printf("%s, %s ", point_1[i], point_2[i]);
		printf("\n");
	} 

	double phi_1 = atof(point_1[0]) + atof(point_1[1]) / 60 + atof(point_1[2]) / 3600;  //conversão de Degrees-Minutes-Seconds para graus
	if (strcmp(point_1[3], "S") == 0) {   //Latitude -> para Sul o ângulo varia entre 0 e -90
		phi_1 = -phi_1;
	}
	printf("PHI_1 %f\n", phi_1);

	double lambda_1 = atof(point_1[4]) + atof(point_1[5]) / 60 + atof(point_1[6]) / 3600;
	if (strcmp(point_1[7], "W") == 0) {  //Longitude -> para Este o ângulo varia entre 0 e -180
		lambda_1 = -lambda_1;
	}
	printf("LAMBDA_1 %f\n", lambda_1);

	double phi_2 = atof(point_2[0]) + atof(point_2[1]) / 60 + atof(point_2[2]) / 3600;
	if (strcmp(point_2[3], "S") == 0) {
		phi_2 = -phi_2;
	}
	printf("PHI_2 %f\n", phi_2);
	
	double lambda_2 = atof(point_2[4]) + atof(point_2[5]) / 60 + atof(point_2[6]) / 3600;
	if (strcmp(point_2[7], "W") == 0) {
		lambda_2 = -lambda_2;
	}
	printf("LAMBDA_2 %f\n", lambda_2);

	(*info)[0] = phi_1;
	(*info)[1] = lambda_1;
	(*info)[2] = phi_2;
	(*info)[3] = lambda_2;
	(*info)[4] = atof(point_1[8]); // A altitude a tomar em conta é a do 1º ponto do segmento
	(*info)[5] = atof(point_2[8]); // Altitude do ponto final
	(*info)[6] = atof(point_1[9]); // velocidade

	return;
}

double dist_btw_2points(double info[7]) { //função para calcular distância entre 2 pontos consecutivos
	double distance;
	distance = acos(sin(info[0]*DEG_TO_RAD)*sin(info[2]*DEG_TO_RAD) + cos(info[0]*DEG_TO_RAD)*cos(info[2]*DEG_TO_RAD)*cos(info[1]*DEG_TO_RAD - info[3] * DEG_TO_RAD)) * (info[4] + EARTH_RADIUS);  // great-circle 
																													  // distance
	// https://en.wikipedia.org/wiki/Great-circle_distance
	
	return distance;
}

double calculate_height_dev(double present_height, double final_height) {
	double height_dev = ALPHA * final_height - ALPHA * present_height;
	return height_dev;
}

double calculate_true_heading(double info[7]) {
	double phi_1 = info[0]*DEG_TO_RAD;
	double lambda_1 = info[1]*DEG_TO_RAD;
	double phi_2 = info[2]*DEG_TO_RAD;
	double lambda_2 = info[3]*DEG_TO_RAD;
	double var_lamba = lambda_2 - lambda_1;

	double y = sin(var_lamba)*cos(phi_2);
	double x = (cos(phi_1)*sin(phi_2)) - (sin(phi_1)*cos(phi_2)*cos(var_lamba));
	double heading = atan2(y, x)*RAD_TO_DEG;
	return heading;
}

void calculate_velocity_N_E(double **velocity_N_E, double V_TAS, double Theta_Path, double True_Heading) {
	(*velocity_N_E)[0] = V_TAS*cos(Theta_Path*DEG_TO_RAD)*cos(True_Heading*DEG_TO_RAD);// velocidade para norte
	(*velocity_N_E)[1] = V_TAS*cos(Theta_Path*DEG_TO_RAD)*sin(True_Heading*DEG_TO_RAD);// velocidade para este
	return;
}

double calculate_theta_path(double V_TAS, double height_dev) {
	double theta_path = asin(height_dev / V_TAS)*RAD_TO_DEG;
	return theta_path;
}

double calculate_V_m(double V_TAS, double delta_time) {
	double V_m = V_TAS(1 + 0.01*sin(2*PI*delta_time/T);
	return V_m;
}

double controller_actuator(double V_sensor, double V_ref) {
    double V_TAS = V_sensor + (V_ref - V_sensor) + 10;
    return V_TAS;
}

int main() {
	FILE *file;
	double route_distance = 0, time_between_points = 0, total_route_distance = 0, height_dev = 0, height = 0, true_heading = 0, theta_path = 0, time_div = 0, delta_time = 0;
	double *info, *velocity_N_E, *info_m;
	char *ch, line[DIM], point_1[NB_DATA][ELE_SIZE], point_2[NB_DATA][ELE_SIZE];
	int i = 0, j = 0;
	time_t seconds_prev, seconds_init, seconds_act;
	
	file = fopen("cities.txt", "r"); // abrir ficheiro
	info = calloc(7, sizeof(double));
	velocity_N_E = calloc(2, sizeof(double));

	if (file == NULL) {     // check if file was correctly opened
		printf("Error opening file");
		return 0;
	}
    
	while (fgets(line, DIM, file) != NULL) {  //condição do while lê linha a linha do ficheiro e passa a linha respetiva para a variável line
		j = 0;                               // o objetivo é pegar em dois pontos consecutivos do ficheiro (seguir ordem) e calcular a distância 
		ch = strtok(line, " 'mkº/h");        // entre eles (a distância constante) e assim sucessivamente.  
											 // strtok elimina os caracteres indicados da string. fiz isso para depois meter tudo num vector 
											 //(neste caso, point_1 e point_2) 
	                              
		while (ch != NULL) {
            //printf("CH -> %s\n", ch);
			if (*ch != '\n' && i == 0) {
				strcpy(point_1[j], ch);
				//printf("Point_1[%d] -> %s\n", j, point_1[j]);
				j = j + 1;
			}

			else if (*ch != '\n' && i == 1) {
				strcpy(point_2[j], ch);
				//printf("Point_2[%d] -> %s\n", j, point_2[j]);
				j = j + 1;
			}

			else if (*ch != '\n' && i > 1) {
				strcpy(point_1[j], point_2[j]);
				strcpy(point_2[j], ch);
				//printf("Point_1[%d] -> %s\n", j, point_1[j]);
				//printf("Point_2[%d] -> %s\n", j, point_2[j]);
				j = j + 1;
			}

			else {
				i++;
				printf("\n");
			}
			ch = strtok(NULL, " 'mkº/h");
		}

		if (i >= 2) {
			// antes de processar o caminho (ponto inicial)
			process_points(point_1, point_2, &info);
			route_distance = dist_btw_2points(info);
			total_route_distance = total_route_distance + route_distance;
			seconds_init = time(NULL);
			seconds_prev = seconds_init;
			seconds_act = seconds_init;
			height = info[4];
			height_dev = calculate_height_dev(height, info[5]);
			true_heading = calculate_true_heading(info);
			true_air_speed = info[6]*KMH_TO_MS;  // assumir que v_tas inicial é igual à v_ref
			v_ref = info[6] * KMH_TO_MS;
			theta_path = calculate_theta_path(true_air_speed, height_dev);
			calculate_velocity_N_E(&velocity_N_E, true_air_speed, theta_path, true_heading);
			
			// processar caminho (isto agora vai estar meio preso aqui, porque o tempo não está muito acelerado)

			while(dist_btw_2points(info) > 10000) {
				seconds_act = time(NULL);
				if (((double)seconds_act - (double)seconds_prev) >= 1) {
					time_div = ((double)seconds_act - (double)seconds_prev) * TIME_ACEL;
					seconds_prev = seconds_act;
					calculate_V_m(double V_TAS, double time_div)
					height = height + (height_dev * time_div);
					info[0] = info[0] + (((velocity_N_E[0] * time_div) / (height + EARTH_RADIUS)) * RAD_TO_DEG);
					info[1] = info[1] + (((velocity_N_E[1] * time_div) / (height + EARTH_RADIUS)) * RAD_TO_DEG);
					printf("V_TAS: %f | Distancia_proximo_waypoing: %f \n", info[6],dist_btw_2points(info));
					printf("Elevacao: %f | Azimute: %f\n", theta_path, true_heading);
					printf("Longitude 1: %f | Latitude 1: %f | Longitude 2: %f | Latitude 2: %f Altura: %f |  Altura final: %f\n", info[0], info[1], info[2], info[3], height, info[5]);
					height_dev = calculate_height_dev(height, info[5]);
					true_heading = calculate_true_heading(info);
					theta_path = calculate_theta_path(info[6]*KMH_TO_MS, height_dev);
					calculate_velocity_N_E(&velocity_N_E, info[6]*KMH_TO_MS, theta_path, true_heading);
					
					//getchar();
				}
			}
		}
		
	}

	printf("DISTANCE: %f m \n", total_route_distance);
	free(info);

	fclose(file);
	
	return 0;
}
