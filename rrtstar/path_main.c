#include "optpath.h" 

/**
 * \fn int main(int argc, char *argv[])
 * \brief program entry 
 * \param argc must be 5 to run the program 
 * \param *argv[] coordinates of initial point and arrival point 
 * \return 0 if there were not enough parameter in entry, else 1
 */

int main(int argc, char *argv[])
{
    int64_t t = ts_now();
    char *obstacle;
    obstacle = malloc(50 * sizeof(char));
    obstacle = OBSTACLES;

    double x_root;
    double y_root;
    double x_arrival;
    double y_arrival;
    int c;
    int cpt = 0;
    int digit_optind = 0;
    int aopt = 0, bopt = 0;
    char *copt = 0, *dopt = 0;
    while ((c = getopt(argc, argv, "x:y:a:b:o:n")) != -1) {
	int this_option_optind = optind ? optind : 1;
	switch (c) {

	case 'x':
	    aopt = 1;
	    x_root = atof(optarg);
	    cpt++;
	    break;
	case 'y':
	    y_root = atof(optarg);
	    bopt = 1;
	    cpt++;
	    break;
	case 'a':
	    copt = 1;
	    x_arrival = atof(optarg);
	    cpt++;
	    break;
	case 'b':
	    dopt = optarg;
	    y_arrival = atof(optarg);
	    cpt++;
	    break;
	case 'o':
	    printf("option obs:%s\n", optarg);
	    dopt = optarg;
	    obstacle = optarg;
	    break;
	}
    }
    if (optind < argc) {
	printf("non-option ARGV-elements: ");
	while (optind < argc)
	    printf("%s ", argv[optind++]);
	printf("\n");
    }
    if (cpt == 4) {
	int route;
	route = final_path(x_root, y_root, x_arrival, y_arrival, obstacle);
	time_see(((double) (ts_now() - t)) / 1000000.0);
	return 1;
    } else {
	error("NOT ENOUTH PARAMETERS\n");
	return 0;
    }
}
