/**
 * \file optmain.c
 * \brief Compute shortest ship trajectories
 * \author Houssem.M
 * \version 0.1
 * \date 23 08 2012
 *
 * Program to compute shortest ship trajectories using the implemantion of rrtstar algorithm done by Sertak Karaman
 *
 */




#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>
#include <sys/time.h>
#include <unistd.h>

#include "optpath.h"
#include "opttree.h"
#include "geos_c.h"
#include "/usr/include/json/json.h"





// Returns current time 
int64_t ts_now()
{

    struct timeval tv;

    gettimeofday(&tv, NULL);

    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;

}



//used to initialise geos
void notice1(const char *fmt, ...)
{
    va_list ap;

    fprintf(stdout, "NOTICE: ");

    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
    fprintf(stdout, "\n");
}

////used to initialise geos
void log_and_exit1(const char *fmt, ...)
{
    va_list ap;

    fprintf(stdout, "ERROR: ");

    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
    fprintf(stdout, "\n");
    exit(1);
}

/**
 * \fn location locat_point(double x, double y)
 * \brief find the location of a given point
 * \param x the x-coordinate of the point
 * \param y the y-coordinate of the point
 * \return loc the location if the point is in a particular area, else ORDINARY 
 */
location locat_point(double x, double y)
{

    location loc = ORDINARY;
    if ((x > 27) && (x < 42) && (y > 40) && (y < 48)) {
	loc = BLACK_SEA;
    } else if (((x > -8) && (x < 27) && (y > 30) && (y < 46))
	       || ((x > 27)
		   && (x < 37) && (y > 30) && (y < 39))) {
	loc = MEDETERANIAN;
    } else if ((x > 16) && (x < 30) && (y > 54) && (y < 66)) {
	loc = BALTIC_SEA;
    } else if ((x > 6) && (x < 16) && (y > 53) && (y < 60)) {
	loc = NORTH_SEA;
    }
    return loc;
}

/**
 * \fn GSList *dividing_path(double x1, double y1, double x2, double y2,char *obstacle, location loc)
 * \brief This function is used when one of the daparture or arrival point is situated in one of the complex area already defined
 * \param x1 the x-coordinate of the departure point
 * \param y1 the y-coordinate of the departure point
 * \param x2 the x-coordinate of the arrival point
 * \param y2 the y-coordinate of the arrival point
 * \param obstacle the name of the obstacles file
 * \param loc location of the unordinary point
 * \return list_ptr a GSList containing the states that constitute the path
 */
GSList *dividing_path(double x1, double y1, double x2, double y2,
		      char *obstacle, location loc)
{
    double x_rac1, y_rac1, x_rac2, y_rac2;
    switch (loc) {

    case NORTH_SEA:
	x_rac1 = 6.1;
	y_rac1 = 57.5;
	x_rac2 = 5.9;
	y_rac2 = 57.5;
	break;

    case BALTIC_SEA:
	x_rac1 = 16.1;
	y_rac1 = 55;
	x_rac2 = 15.9;
	y_rac2 = 55;
	break;

    case MEDETERANIAN:
	x_rac1 = -7;
	y_rac1 = 35.9;
	x_rac2 = -8.5;
	y_rac2 = 35.9;
	break;
    }

    GSList *list_ptr = path(x1, y1, x_rac1, y_rac1, obstacle);
    GSList *list_ptr2 = path(x_rac2, y_rac2, x2, y2, obstacle);
    GSList *list2 = list_ptr2;
    while (list2) {
	double x, y;
	state_t *d = list2->data;
	x = d->x[0];
	y = d->x[1];
	printf("%lf %lf\n", x, y);
	list_ptr = g_slist_prepend(list_ptr, d);
	list2 = g_slist_next(list2);
    }
    return list_ptr;
}

/**
 * \fn GSList* final_path(double x_root, double y_root, double x_arrival, double y_arrival,char *obstacle)
 * \brief This function calculate the final path that could be a connexion between 2 paths, then it fill the geojson file of the trajectory
 * \param x_root the x-coordinate of the departure point
 * \param y_root the y-coordinate of the departure point
 * \param x_arrival the x-coordinate of the arrival point
 * \param y_arrival the y-coordinate of the arrival point
 * \param obstacle the name of the obstacles file
 * \return l_ptr the GSList containing states of the path if it has been found, else NULL
 */
GSList*
final_path(double x_root, double y_root, double x_arrival,
	   double y_arrival, char *obstacle)
{
    location loc;
    location loc_root = locat_point(x_root, y_root);
    location loc_arrival = locat_point(x_arrival, y_arrival);

    GSList *l_ptr=NULL;
    //We use a correspondance if we have a point in a critical zone
    if ((loc_root == NORTH_SEA) && (loc_arrival != NORTH_SEA)
	&& (loc_arrival != BALTIC_SEA)) {
	loc = NORTH_SEA;
	l_ptr =
	    dividing_path(x_root, y_root, x_arrival, y_arrival, obstacle,
			  loc);
    }

    else if ((loc_arrival == NORTH_SEA) && (loc_root != NORTH_SEA)
	     && (loc_root != BALTIC_SEA)) {
	loc = NORTH_SEA;
	l_ptr =
	    dividing_path(x_arrival, y_arrival, x_root, y_root, obstacle,
			  loc);

    } else if ((loc_root == BALTIC_SEA) && (loc_arrival != BALTIC_SEA)) {
	loc = BALTIC_SEA;
	l_ptr =
	    dividing_path(x_root, y_root, x_arrival, y_arrival, obstacle,
			  loc);

    } else if ((loc_arrival == BALTIC_SEA) && (loc_root != BALTIC_SEA)) {
	loc = BALTIC_SEA;
	l_ptr =
	    dividing_path(x_arrival, y_arrival, x_root, y_root, obstacle,
			  loc);

    } else if ((loc_root == MEDETERANIAN)
	       && (loc_arrival != MEDETERANIAN)) {
	loc = MEDETERANIAN;
	l_ptr =
	    dividing_path(x_root, y_root, x_arrival, y_arrival, obstacle,
			  loc);

    } else if ((loc_arrival == MEDETERANIAN)
	       && (loc_root != MEDETERANIAN)) {
	loc = MEDETERANIAN;
	l_ptr =
	    dividing_path(x_arrival, y_arrival, x_root, y_root, obstacle,
			  loc);

    } else {
	l_ptr = path(x_root, y_root, x_arrival, y_arrival, obstacle);
    }

    FILE *planisphere = fopen(ROUTE, "wt");
    fprintf(planisphere,
	    "{\n\"type\": \"FeatureCollection\",\n\n\"features\": [\n");
    fprintf(planisphere,
	    "{\"type\": \"Feature\", \"id\": 0, \"properties\": {}, \"geometry\": { \"type\": \"LineString\", \"coordinates\": [");
    
    double x, y;
    state_t *d = l_ptr->data;
    x = d->x[0];
    y = d->x[1];
    fprintf(planisphere, "[%3.5lf, %3.5lf,0]", x, y);
    l_ptr = g_slist_next(l_ptr);
    while (l_ptr) {
	double x, y;
	state_t *d = l_ptr->data;
	x = d->x[0];
	y = d->x[1];
	fprintf(planisphere, ",[%3.5lf, %3.5lf,0]", x, y);
	l_ptr = g_slist_next(l_ptr);
    }

    fprintf(planisphere, "]}}\n");
    fprintf(planisphere, "]}\n\n\n");
    fclose(planisphere);
    return l_ptr;

}

/**
 * \fn opttree_t* create_environnement(double x_root, double y_root, double x_arrival,double y_arrival,char* obstacle, int k)
 * \brief create the work environement; actually root_state, goal_region, operating region(depending on departure and arrival) and obstacle list(continents)
 * \param x_root the x-coordinate of the departure point
 * \param y_root the y-coordinate of the departure point
 * \param x_arrival the x-coordinate of the arrival point
 * \param y_arrival the y-coordinate of the arrival point
 * \param obstacles the name of the obstacles file
 * \param k the factor of compressing the operating region. In fact, if we work on all planisphere we compresse it in order to increase the number of point randomized
 * \return 1 if the path has been found, else 0
 */
opttree_t *create_environnement(double x_root, double y_root,
				double x_arrival, double y_arrival,
				char *obstacle, int k)
{
    FILE *f_obstacles = fopen(obstacle, "r");
    opttree_t *opttree = opttree_create();
    if (!opttree) {
	error("Memory allocation error\n");
	exit(1);
    }
    int c1, c2, s1, s2;
    location loc_root = locat_point(x_root, y_root);
    location loc_arrival = locat_point(x_arrival, y_arrival);
    //1. Setup the operating_region according to points location 
    if (((loc_root == MEDETERANIAN) || (loc_root == BLACK_SEA))
	&& ((loc_arrival == MEDETERANIAN)
	    || (loc_arrival == BLACK_SEA))) {
	c1 = MED_CENTER_X / k;
	c2 = MED_CENTER_Y / k;
	s1 = MED_SIZE_X / k;
	s2 = MED_SIZE_Y / k;
    } else if ((loc_root == BALTIC_SEA) && (loc_arrival == BALTIC_SEA)) {
	c1 = BALT_CENTER_X / k;
	c2 = BALT_CENTER_Y / k;
	s1 = BALT_SIZE_X / k;
	s2 = BALT_SIZE_Y / k;
    } else if ((loc_root == NORTH_SEA) && (loc_arrival == NORTH_SEA)) {
	c1 = NORTH_CENTER_X / k;
	c2 = NORTH_CENTER_Y / k;
	s1 = NORTH_SIZE_X / k;
	s2 = NORTH_SIZE_Y / k;
    } else {
	c1 = WORLD_CENTER_X;
	c2 = WORLD_CENTER_Y;
	s1 = WORLD_SIZE_X / k;
	s2 = WORLD_SIZE_Y / k;
    }

    region_2d_t operating_region = {
	.center = {
		   c1, c2}
	,
	.size = {
		 s1, s2}
    };
    optsystem_update_operating_region(opttree->optsys, &operating_region);
    // 2.create obstacles
    initGEOS(notice1, log_and_exit1);
    state_t *node;
    GEOSCoordSequence *cs;
    GEOSGeometry *g;
    GEOSGeometry *shell;
    int j = 0;
    json_object *obj;
    fseek(f_obstacles, 0, SEEK_END);
    int t = ftell(f_obstacles);
    printf("t=%d\n ", t);
    fclose(f_obstacles);
    FILE *f = fopen(obstacle, "r");
    char *str_;
    char *str1;
    str_ = malloc(t * sizeof(char));
    str1 = malloc(t * sizeof(char));
    fgets(str_, t, f);
    while (fgets(str1, t, f) != NULL) {
	strcat(str_, str1);
	fscanf(f, "\n");
    }

    obj = json_tokener_parse(str_);
    json_object *features = json_object_object_get(obj, "features");
    GSList *obstacle_list = NULL;
    int k_;
    j = 0;
    int cpt = 0;
    for (j = 0; j < json_object_array_length(features); j++) {
	cpt++;
	GSList *obstacle = NULL;
	json_object *poly = json_object_array_get_idx(features, j);
	json_object *geometry = json_object_object_get(poly, "geometry");
	json_object *type_geo = json_object_object_get(geometry, "type");
	if (strcmp(json_object_to_json_string(type_geo), "\"Polygon\"") ==
	    0) {
	    json_object *coordinates =
		json_object_object_get(geometry, "coordinates");
	    json_object *coord = json_object_array_get_idx(coordinates, 0);
	    json_object *jo = NULL;
	    json_object *x;
	    json_object *y;
	    int i = 0;
	    for (i = 0; i < json_object_array_length(coord); i++) {
		jo = json_object_array_get_idx(coord, i);
		x = json_object_array_get_idx(jo, 0);
		y = json_object_array_get_idx(jo, 1);
		char *str_x = json_object_to_json_string(x);
		char *str_y = json_object_to_json_string(y);
		node = malloc(sizeof(state_t));
		node->x[0] = atof(str_x) / k;
		node->x[1] = atof(str_y) / k;
		obstacle = g_slist_prepend(obstacle, node);
	    }
	    cs = GEOSCoordSeq_create(g_slist_length(obstacle), 2);
	    int jj = 0;
	    while (obstacle) {
		state_t *point = (state_t *) (obstacle->data);
		k_ = GEOSCoordSeq_setX(cs, jj, point->x[0]);
		k_ = GEOSCoordSeq_setY(cs, jj, point->x[1]);
		obstacle = g_slist_next(obstacle);
		jj++;
	    }
	    shell = GEOSGeom_createLinearRing(cs);
	    g = GEOSGeom_createPolygon(shell, NULL, 0);
	    obstacle_list = g_slist_prepend(obstacle_list, g);
	}




	else if (strcmp
		 (json_object_to_json_string(type_geo),
		  "\"MultiPolygon\"") == 0) {
	    json_object *coordinates =
		json_object_object_get(geometry, "coordinates");

	    int jjj = 0;
	    for (jjj = 0; jjj < json_object_array_length(coordinates);
		 jjj++) {
		json_object *coord0 =
		    json_object_array_get_idx(coordinates, jjj);
		json_object *coord = json_object_array_get_idx(coord0, 0);
		json_object *jo = NULL;
		json_object *x;
		json_object *y;
		int i = 0;
		for (i = 0; i < json_object_array_length(coord); i++) {
		    jo = json_object_array_get_idx(coord, i);
		    x = json_object_array_get_idx(jo, 0);
		    y = json_object_array_get_idx(jo, 1);
		    char *str_x = json_object_to_json_string(x);
		    char *str_y = json_object_to_json_string(y);
		    node = malloc(sizeof(state_t));
		    node->x[0] = atof(str_x) / k;
		    node->x[1] = atof(str_y) / k;
		    obstacle = g_slist_prepend(obstacle, node);
		}
		cs = GEOSCoordSeq_create(g_slist_length(obstacle), 2);
		int jj = 0;
		while (obstacle) {
		    state_t *point = (state_t *) (obstacle->data);
		    k_ = GEOSCoordSeq_setX(cs, jj, point->x[0]);
		    k_ = GEOSCoordSeq_setY(cs, jj, point->x[1]);
		    obstacle = g_slist_next(obstacle);
		    jj++;
		}
		shell = GEOSGeom_createLinearRing(cs);
		g = GEOSGeom_createPolygon(shell, NULL, 0);
		obstacle_list = g_slist_prepend(obstacle_list, g);
	    }
	}

    }
    optsystem_update_obstacles(opttree->optsys, obstacle_list);
    // 3. create the root state and the goal region
    state_t root_state = {
	.x = {
	      x_root / k, y_root / k}
    };

    state_t arrival_state = {
	.x = {
	      x_arrival / k, y_arrival / k}
    };
    //We choose the departure from the "critical" zones
    if (loc_arrival == BLACK_SEA) {
	state_t state_perm = root_state;
	root_state = arrival_state;
	arrival_state = state_perm;
    }

    else if (loc_arrival == MEDETERANIAN) {
	state_t state_perm = root_state;
	root_state = arrival_state;
	arrival_state = state_perm;
    } else if (loc_arrival == NORTH_SEA) {
	state_t state_perm = root_state;
	root_state = arrival_state;
	arrival_state = state_perm;
    } else if (loc_arrival == BALTIC_SEA) {
	state_t state_perm = root_state;
	root_state = arrival_state;
	arrival_state = state_perm;
    }
    point_see(root_state.x[0] * k, root_state.x[1] * k);
    point_see(arrival_state.x[0] * k, arrival_state.x[1] * k);
    double d = optsystem_evaluate_distance(opttree->optsys, &root_state,
					   &arrival_state);
    int s;
    if (d < MIN_DISTANCE / k) {
	s = SIZE_CLOSE_POINT / k;
    } else {
	s = SIZE_FAR_POINT / k;
    }
    region_2d_t goal_region = {
	.center = {
		   arrival_state.x[0], arrival_state.x[1]}
	,
	.size = {
		 s, s}
    };
    opttree_set_root_state(opttree, &root_state);
    optsystem_update_goal_region(opttree->optsys, &goal_region);
    finishGEOS();
    return opttree;
}

/**
 * \fn GSList *path(double x_root, double y_root, double x_arrival, double y_arrival,char *obstacle)
 * \brief This function calculate the shortest path between two points: applicate rrtstar alogorithm, then use the correcting path algorithm to affinate the result.
 * \param x_root the x-coordinate of the departure point
 * \param y_root the y-coordinate of the departure point
 * \param x_arrival the x-coordinate of the arrival point
 * \param y_arrival the y-coordinate of the arrival point
  * \param obstacles the name of the obstacles file
 * \return list_ptr a GSList containing the states that constitute the path
 */
GSList *path(double x_root, double y_root, double x_arrival,
	     double y_arrival, char *obstacle)
{
    int i;
    int k = 1;
    GSList *list_ptr = NULL;
    // Setup the maximum number of iterations
    int num_iterations = MAX_ITERATION;
    // Setup k: 
    //if the operating region is [-180,180],[-90,90], we compresse it 
    location loc_root = locat_point(x_root, y_root);
    location loc_arrival = locat_point(x_arrival, y_arrival);
    if (((loc_root == ORDINARY) || (loc_arrival == ORDINARY))
	|| (loc_root != loc_arrival)) {
	k = COMPRESSION_FACTOR;
    }
    // 1. Create opttree structure
    opttree_t *opttree = opttree_create();
    if (!opttree) {
	error("Memory allocation error\n");
	exit(1);
    }
    // 2. setup environnement
    opttree =
	create_environnement(x_root, y_root, x_arrival, y_arrival,
			     obstacle, k);

    // 3. Run opttree in iterations
    state_t root_state = *((opttree->optsys)->initial_state);
    region_2d_t goal_region = (opttree->optsys)->goal_region;
    int64_t time_start = ts_now();
    gboolean b = FALSE;
    if (optsystem_segment_on_obstacle(opttree->optsys, &root_state,
				      &goal_region.center) == 0) {
	state_t *state_ptr1 = NULL;
	state_ptr1 = malloc(sizeof(state_t));
	(*state_ptr1).x[0] = goal_region.center[0] * k;
	(*state_ptr1).x[1] = goal_region.center[1] * k;
	list_ptr = g_slist_prepend(list_ptr, state_ptr1);
	state_t *state_ptr2 = NULL;
	state_ptr2 = malloc(sizeof(state_t));
	(*state_ptr2).x[0] = root_state.x[0] * k;
	(*state_ptr2).x[1] = root_state.x[1] * k;
	list_ptr = g_slist_prepend(list_ptr, state_ptr2);
    } else {
	i = 0;
	double ts_find = 300;
	for (i = 0; i < num_iterations; i++) {

	    opttree_iteration(opttree);
	    if ((i != 0) && (i % 100 == 0)) {
		if ((opttree->lower_bound < 99999)
		    && (b == FALSE)) {
		    b = TRUE;
		    if ((opttree->lower_bound) * k < 50)
			num_iterations = i + 502;
		    if (((opttree->lower_bound) * k >= 50)
			&& ((opttree->lower_bound)
			    * k < 200))
			num_iterations = i + 1002;
		    if (((opttree->lower_bound) * k >= 200)
			&& ((opttree->lower_bound) * k < 400))
			num_iterations = i + 2002;
		    if (((opttree->lower_bound) * k >= 400))
			num_iterations = i + 3002;
		    if (i > 50000)
			num_iterations = i + 502;
		    ts_find = (ts_now() - time_start) / 1000000.0;
		}
		time_cost(((double) (ts_now() -
				     time_start)) / 1000000.0,
			  (opttree->lower_bound) * k);
	    }
	    if ((double) (ts_now() - time_start) / 1000000.0 -
		ts_find > 10)
		num_iterations = i + 1;
	}
	warning(i);
	GSList *optstates_list = NULL;
	node_t *node_curr = opttree->lower_bound_node;
	if (node_curr == NULL) {
	    error("PATH NOT FOUND");
	}
	while (node_curr) {
	    optstates_list = g_slist_prepend(optstates_list, node_curr);
	    node_curr = node_curr->parent;
	}

	list_ptr = correcting_path(opttree, optstates_list, k);
	// 5. Destroy the opttree structure
	opttree_destroy(opttree);

    }
    return list_ptr;

}

/**
 * \fn GSList* correcting_path(opttree_t* opttree,GSList* optstates_list,int k)
 * \brief from a path between two points (the result of rrtstar algorithm but with minimum number of iterations), make a better one by eleminating nodes and adding others
 * \param opttree the opttree_t structure already created (the environnement of working)
 * \param optstates_list the list of nodes constituting the rrtstar path
 * \param k compressing factor
 * \return list_ptr a GSList containing the states that constitute the new better path
 */
GSList *correcting_path(opttree_t * opttree, GSList * optstates_list,
			int k)
{

//algorithm correcting the path given by the rrtstar algorithm
    double *tx;
    double *ty;
    double *tx_i;
    double *ty_i;
    int i = 0;
    double x, y;
    gboolean b = 0;
    int j = 0;
    tx = (double *) malloc(g_slist_length(optstates_list) * 4 *
			   sizeof(double));
    ty = (double *) malloc(g_slist_length(optstates_list) * 4 *
			   sizeof(double));
    state_t root_state = *((opttree->optsys)->initial_state);
    region_2d_t goal_region = (opttree->optsys)->goal_region;
    tx[0] = root_state.x[0];
    ty[0] = root_state.x[1];
    //we put the nodes of the path in tx and ty
    if (optstates_list) {
	GSList *optstates_ptr = optstates_list;
	while (optstates_ptr) {
	    node_t *node_curr = (node_t *) (optstates_ptr->data);
	    GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
	    while (traj_from_parent_ptr) {
		state_t *state_this =
		    (state_t *) (traj_from_parent_ptr->data);
		i++;
		tx[i] = (state_this->x[0] + tx[i - 1]) / 2;
		ty[i] = (state_this->x[1] + ty[i - 1]) / 2;
		tx[i + 1] = state_this->x[0];
		ty[i + 1] = state_this->x[1];
		i++;
		traj_from_parent_ptr = g_slist_next(traj_from_parent_ptr);
	    }
	    state_t *state_curr = node_curr->state;
	    i++;
	    tx[i] = (state_curr->x[0] + tx[i - 1]) / 2;
	    ty[i] = (state_curr->x[1] + ty[i - 1]) / 2;
	    tx[i + 1] = state_curr->x[0];
	    ty[i + 1] = state_curr->x[1];
	    i++;

	    optstates_ptr = g_slist_next(optstates_ptr);
	}
    }
    i++;
    //add the arrival point to the trajectory to exceed the goal region limit
    tx[i] = goal_region.center[0];
    ty[i] = goal_region.center[1];
    int nb_nodes0 = i;
    // we search nodes that could be eliminated 
    tx_i =
	(double *) malloc(g_slist_length(optstates_list) * 4 *
			  sizeof(double));
    ty_i =
	(double *) malloc(g_slist_length(optstates_list) * 4 *
			  sizeof(double));
    tx_i[0] = tx[nb_nodes0];
    ty_i[0] = ty[nb_nodes0];
    int k_ = 1;
    i = nb_nodes0;
    while (i != 0) {
	state_t state_a = {
	    .x = {
		  tx[i], ty[i]}
	};
	b = 0;
	j = 0;
	while (!b) {
	    state_t state_b = {
		.x = {
		      tx[j], ty[j]}
	    };
	    if (optsystem_segment_on_obstacle
		(opttree->optsys, &state_a, &state_b) == 0) {
		tx_i[k_] = tx[j];
		ty_i[k_] = ty[j];
		k_++;
		i = j;
		b = 1;
	    }

	    j++;
	}
    }

    int nb_nodes1 = k_;
    k_--;

    //find if a node i could be related with a point of the edge [i+1,i+2]
    //0->nb_nodes1
    for (i = 0; i < nb_nodes1 - 2; i++) {
	state_t state_0 = {
	    .x = {
		  tx_i[i], ty_i[i]}
	};
	state_t state_2 = {
	    .x = {
		  tx_i[i + 2], ty_i[i + 2]}
	};

	x = (tx_i[i + 1] - tx_i[i + 2]) / 30;
	y = (ty_i[i + 1] - ty_i[i + 2]) / 30;
	gboolean b = 0;
	state_t state_trans = state_2;
	while (!b) {
	    if (optsystem_segment_on_obstacle
		(opttree->optsys, &state_0, &state_trans) == 0) {
		tx_i[i + 1] = state_trans.x[0];
		ty_i[i + 1] = state_trans.x[1];
		b = 1;
	    }
	    state_trans.x[0] = state_trans.x[0] + x;
	    state_trans.x[1] = state_trans.x[1] + y;
	}
    }
    // nb_nodes1->0
    for (i = nb_nodes1 - 1; i > 1; i--) {
	state_t state_0 = {
	    .x = {
		  tx_i[i], ty_i[i]}
	};
	state_t state_2 = {
	    .x = {
		  tx_i[i - 2], ty_i[i - 2]}
	};

	x = (tx_i[i - 1] - tx_i[i - 2]) / 30;
	y = (ty_i[i - 1] - ty_i[i - 2]) / 30;
	gboolean b = 0;
	state_t state_trans = state_2;
	while (!b) {
	    if (optsystem_segment_on_obstacle
		(opttree->optsys, &state_0, &state_trans) == 0) {
		tx_i[i - 1] = state_trans.x[0];
		ty_i[i - 1] = state_trans.x[1];
		b = 1;
	    }
	    state_trans.x[0] = state_trans.x[0] + x;
	    state_trans.x[1] = state_trans.x[1] + y;
	}
    }

    //find if a point of an edge could be related to a point of the next edge
    double *px;
    double *py;
    px = (double *) malloc(g_slist_length(optstates_list) * 8 *
			   sizeof(double));
    py = (double *) malloc(g_slist_length(optstates_list) * 8 *
			   sizeof(double));
    double d1x, d1y, d2x, d2y;
    gboolean b1, b2;
    px[0] = tx_i[0];
    py[0] = ty_i[0];
    j = 1;
    int k1, k2;

    for (i = 1; i < nb_nodes1 - 1; i++) {
	d1x = (tx_i[i - 1] - tx_i[i]) / 5;
	d1y = (ty_i[i - 1] - ty_i[i]) / 5;
	d2x = (tx_i[i + 1] - tx_i[i]) / 5;
	d2y = (ty_i[i + 1] - ty_i[i]) / 5;
	k2 = 2;
	k1 = 1;
	b1 = 1;
	b2 = 1;
	state_t state_0 = {
	    .x = {
		  tx_i[i] + d1x, ty_i[i] + d1y}
	};

	while ((b1) && (k1 < 5)) {
	    state_t state_1 = {
		.x = {
		      tx_i[i] + k1 * d2x,
		      ty_i[i] + k1 * d2y}
	    };

	    if (optsystem_segment_on_obstacle
		(opttree->optsys, &state_0, &state_1) == 0)
		k1++;
	    else {
		b1 = 0;
		k1--;
	    }
	}
	if (k1 != 0) {
	    state_t state_1 = {
		.x = {
		      tx_i[i] + k1 * d2x,
		      ty_i[i] + k1 * d2y}
	    };
	    while ((b2) && (k2 < 5)) {
		state_t state_2 = {
		    .x = {
			  tx_i[i] + k2 * d1x,
			  ty_i[i] + k2 * d1y}
		};
		if (optsystem_segment_on_obstacle
		    (opttree->optsys, &state_1, &state_2) == 0)
		    k2++;
		else {
		    b2 = 0;
		    k2--;
		}
	    }
	    if (j == 1) {
		px[j] = tx_i[i] + k2 * d1x;
		py[j] = ty_i[i] + k2 * d1y;
		px[j + 1] = tx_i[i] + k1 * d2x;
		py[j + 1] = ty_i[i] + k1 * d2y;
		j = j + 2;
	    } else {
		px[j] = tx_i[i] + k2 * d1x;
		py[j] = ty_i[i] + k2 * d1y;
		px[j + 1] = tx_i[i] + k1 * d2x;
		py[j + 1] = ty_i[i] + k1 * d2y;
		if ((px[j] - px[j - 1]) * (tx_i[i] - tx_i[i - 1]) > 0)
		    j = j + 2;
		else {
		    px[j - 1] = px[j];
		    px[j] = px[j + 1];
		    py[j - 1] = py[j];
		    py[j] = py[j + 1];
		    j++;
		}

	    }
	} else {
	    px[j] = tx_i[i];
	    py[j] = ty_i[i];
	    j++;
	}
    }
    px[j] = tx_i[nb_nodes1 - 1];
    py[j] = ty_i[nb_nodes1 - 1];
    int nb_nodes2 = j;

    //find if a point of an edge could be related to a point of the previous edge
    double *px1;
    double *py1;
    px1 =
	(double *) malloc(g_slist_length(optstates_list) * 16 *
			  sizeof(double));
    py1 =
	(double *) malloc(g_slist_length(optstates_list) * 16 *
			  sizeof(double));
    px1[0] = px[nb_nodes2];
    py1[0] = py[nb_nodes2];
    j = 1;
    for (i = nb_nodes2 - 1; i > 0; i--) {
	d1x = (px[i - 1] - px[i]) / 5;
	d1y = (py[i - 1] - py[i]) / 5;
	d2x = (px[i + 1] - px[i]) / 5;
	d2y = (py[i + 1] - py[i]) / 5;
	k2 = 2;
	k1 = 1;
	b1 = 1;
	b2 = 1;
	state_t state_0 = {
	    .x = {
		  px[i] + d1x, py[i] + d1y}
	};

	while ((b1) && (k1 < 5)) {
	    state_t state_1 = {
		.x = {
		      px[i] + k1 * d2x,
		      py[i] + k1 * d2y}
	    };

	    if (optsystem_segment_on_obstacle
		(opttree->optsys, &state_0, &state_1) == 0)
		k1++;

	    else {
		b1 = 0;
		k1--;
	    }
	}
	if (k1 != 0) {
	    state_t state_1 = {
		.x = {
		      px[i] + k1 * d2x,
		      py[i] + k1 * d2y}
	    };
	    while ((b2) && (k2 < 5)) {
		state_t state_2 = {
		    .x = {
			  px[i] + k2 * d1x,
			  py[i] + k2 * d1y}
		};
		if (optsystem_segment_on_obstacle
		    (opttree->optsys, &state_1, &state_2) == 0)
		    k2++;
		else {
		    b2 = 0;
		    k2--;
		}
	    }
	    if (j == 1) {
		px1[j + 1] = px[i] + k2 * d1x;
		py1[j + 1] = py[i] + k2 * d1y;
		px1[j] = px[i] + k1 * d2x;
		py1[j] = py[i] + k1 * d2y;
		j = j + 2;
	    } else {
		px1[j + 1] = px[i] + k2 * d1x;
		py1[j + 1] = py[i] + k2 * d1y;
		px1[j] = px[i] + k1 * d2x;
		py1[j] = py[i] + k1 * d2y;
		if ((px1[j] - px1[j - 1]) * (px[i] - px[i + 1]) > 0)
		    j = j + 2;
		else {
		    px1[j - 1] = px1[j];
		    px1[j] = px1[j + 1];
		    py1[j - 1] = py1[j];
		    py1[j] = py1[j + 1];
		    j++;
		}
	    }
	} else {
	    px1[j] = px[i];
	    py1[j] = py[i];
	    j++;
	}
    }
    px1[j] = px[0];
    py1[j] = py[0];
    int nb_nodes3 = j;
    k_ = nb_nodes3;
    GSList *list_ptr = NULL;

    state_t *state_ptr = NULL;
    state_ptr = malloc(sizeof(state_t));
    (*state_ptr).x[0] = px1[k_] * k;
    (*state_ptr).x[1] = py1[k_] * k;
    list_ptr = g_slist_prepend(list_ptr, state_ptr);
    while (k_ != 0) {
	k_--;
	state_t *state_ptr1 = NULL;
	state_ptr1 = malloc(sizeof(state_t));
	(*state_ptr1).x[0] = px1[k_] * k;
	(*state_ptr1).x[1] = py1[k_] * k;
	list_ptr = g_slist_prepend(list_ptr, state_ptr1);

    }
    return list_ptr;

}
