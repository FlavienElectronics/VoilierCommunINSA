#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define max 65535 // 16bits
#define min 0

#define CLK_define 72000000

int best_ARR = 0;
int best_PSC = 0;
double best_frequency = 0;

double calculate(int PSC, int ARR, int CLK)
{
	return (1/((((double)PSC + 1.) / (double)CLK) * ((double)ARR + 1)));
}

int dichotomie(int a, int b, double valeur, double tolerance, int ARR, int CLK) {
    int milieu;
    while (a <= b) {
        milieu = a + (b - a) / 2;
        double valeur_milieu = calculate(milieu, ARR, CLK);
        if (fabs(valeur_milieu - valeur) <= tolerance) {
            return milieu;
        }
        if (valeur_milieu > valeur) {
            a = milieu + 1;
        } else {
            b = milieu - 1;
        }
    }
    return -1;
}

double plus_proche(double a, double b, double valeur_centrale) {
    return (fabs(a - valeur_centrale) < fabs(b - valeur_centrale)) ? a : b;
}

int main(int argc, char **argv)
{
	int psc = min;
	int arr = max;

	if (argc > 1 && argc < 4)
	{
		double frequency = atof(argv[1]);
		double tolerance = atof(argv[2]);

        for (int _arr = max; _arr > min; _arr--){

            if ((calculate(min, _arr, CLK_define) > frequency) && (calculate(max, _arr, CLK_define) < frequency)){
                int tmp_psc = dichotomie(min, max, frequency, tolerance, _arr, CLK_define);

                if (tmp_psc != -1){
                    double tmp_frequency = calculate(tmp_psc, _arr, CLK_define);
                    if (tmp_frequency == frequency){
                        best_ARR = _arr;
                        best_frequency = frequency;
                        best_PSC = tmp_psc;
                        break;

                    }else{
                        if (plus_proche(best_frequency, tmp_frequency, frequency) == tmp_frequency){
                            best_frequency = tmp_frequency;
                            best_ARR = _arr;
                            best_PSC = tmp_psc;
                        }
                    }
                }
            }
        }

		printf("\n\t>> BEST RESULT FOUND  :\n\n\tCPU FREQ. :\t%d\n\tUSER FREQ. :\t%f\n\tUSER TOLE. :\t%f\n\t------ USER INFO ------\n\tREEL FREQ. :\t%f\n\tPRESACLER :\t%d\n\tAUTO REL. :\t%d\n\n",CLK_define,frequency,tolerance,best_frequency,best_PSC,best_ARR);
	}else{
		printf("ERROR : bad input interval -> ./timerCalculator <frequency wanted> <minimum tolerance>\n");
		exit(-1);
	}

	return (0);
}