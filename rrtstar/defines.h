#define PUBLISH_NODES_EDGES 0

#define MED_CENTER_X 20
#define MED_CENTER_Y 40
#define MED_SIZE_X 60
#define MED_SIZE_Y 20

#define BALT_CENTER_X 23
#define BALT_CENTER_Y 60
#define BALT_SIZE_X 14
#define BALT_SIZE_Y 12

#define NORTH_CENTER_X 11
#define NORTH_CENTER_Y 56.5
#define NORTH_SIZE_X 10
#define NORTH_SIZE_Y 7

#define WORLD_CENTER_X 0
#define WORLD_CENTER_Y 0
#define WORLD_SIZE_X 360
#define WORLD_SIZE_Y 180

#define MIN_DISTANCE 20
#define SIZE_CLOSE_POINT 1
#define SIZE_FAR_POINT 4
#define MAX_ITERATION 1000000
#define COMPRESSION_FACTOR 20

#define OBSTACLES "obstacles.geojson"
#define THE_PATH "THE_path.txt"
#define OPTPATH "optpath.txt"
#define ROUTE "planisphere.geojson"

#define DISPLAY 1
#define point_see(x,y) do {if (DISPLAY) printf("%lf,%lf\n",x,y);} while(0)
#define error(x) do {if (DISPLAY) printf("ERROR: %s", x);} while(0)
#define time_cost(x,y) do {if (DISPLAY) { printf("Time: %5.5lf, Cost: %5.5lf\n",x,y); }} while(0)
#define warning(x) do {if (DISPLAY) {printf("number of iterations: %d\n\n", x);}} while(0)
#define time_see(x) do {if (DISPLAY) printf("%5.5lf\n",x);} while(0)
