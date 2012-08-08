
#define PUBLISH_NODES_EDGES 0

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>
#include <sys/time.h>

#include "opttree.h"
#include "geos_c.h"

// Returns current time 
int64_t
ts_now () {
  struct timeval tv;
  gettimeofday (&tv,NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void
notice1(const char *fmt, ...) {
	va_list ap;

        fprintf( stdout, "NOTICE: ");
        
	va_start (ap, fmt);
        vfprintf( stdout, fmt, ap);
        va_end(ap);
        fprintf( stdout, "\n" );
}

void
log_and_exit1(const char *fmt, ...) {
	va_list ap;

        fprintf( stdout, "ERROR: ");
        
	va_start (ap, fmt);
        vfprintf( stdout, fmt, ap);
        va_end(ap);
        fprintf( stdout, "\n" );
	exit(1);
}

int main (int argc, char *argv[]) {
    
    int64_t t = ts_now();
    FILE *planisphere = fopen("planisphere.geojson","wt");
    FILE *f_obstacles = fopen("obstacles.txt","r");
    double x;
    double y;
  
    
    // Setup the num_iterations
    int num_iterations = 100000;
    // 1. Create opttree structure
    opttree_t *opttree = opttree_create ();
    if (!opttree) {
        printf ("Memory allocation error\n");
        exit (1);
    }
    
    // 2. Setup the environment
    // 2.a. create the operating region
    region_2d_t operating_region = {
        .center = {0, 0},
        .size = {18, 9}
    };
    optsystem_update_operating_region (opttree->optsys, &operating_region);
    
    
    // 2.b create obstacles
    initGEOS(notice1, log_and_exit1);
    GSList *obstacle_list = NULL;
    GSList *obstacle = NULL;
    state_t *node;
	GEOSCoordSequence* cs;
	GEOSGeometry* g;
	GEOSGeometry* shell;
	int k;
	int cpt_continent=1;

while(cpt_continent<13){	
    while( fscanf(f_obstacles,"[ %lf, %lf ]",&x,&y)!=0){
    	node = malloc (sizeof (state_t));
  	node->x[0] = x/20;
    	node->x[1] = y/20;
       	obstacle=g_slist_prepend(obstacle,node);
    }
	cs= GEOSCoordSeq_create(g_slist_length (obstacle),2);
	int j=0;
	while(obstacle) {
		state_t *point=(state_t *)(obstacle->data);
		k=GEOSCoordSeq_setX(cs, j, point->x[0]);
		k=GEOSCoordSeq_setY(cs, j, point->x[1]);
		obstacle=g_slist_next(obstacle);
		j++;
	}
	
	
	shell = GEOSGeom_createLinearRing(cs);
	g = GEOSGeom_createPolygon(shell, NULL, 0);
	    
	 
	obstacle_list = g_slist_prepend (obstacle_list, g);
    fscanf(f_obstacles,"\n\n");
    cpt_continent++;
}

     
    fclose(f_obstacles);
   finishGEOS();
    
    optsystem_update_obstacles (opttree->optsys, obstacle_list);

    // 2.c create the root state
    state_t root_state = {
        .x = {atof(argv[1]), atof(argv[2])}
    };
    opttree_set_root_state (opttree, &root_state);

    // 2.d. create the goal region
    region_2d_t goal_region = {
        .center = {atof(argv[3]), atof(argv[4])},
        .size = {0.4,0.4}
    };
    optsystem_update_goal_region (opttree->optsys, &goal_region);

    
    
    // 3. Run opttree in iterations
    int64_t time_start = ts_now(); 
    gboolean b=FALSE;
    for (int i = 0; i < num_iterations; i++) {
	
        opttree_iteration (opttree);
          
        if ( (i != 0 ) && (i != 1000 ) && (i%1000 == 0)  ) {
	    if ((opttree->lower_bound<99999) && (b==FALSE)){
		b=TRUE;
		/*x=i/1000;
		num_iterations=i+(i*40*log(x))/(x*x);*/
		num_iterations=i+2000;
	    }
            printf ("Time: %5.5lf, Cost: %5.5lf\n", 
                    ((double)(ts_now() - time_start))/1000000.0, opttree->lower_bound); 
        }
    }
    printf("\n\n");
    
    //4.Print results
	fprintf(planisphere,"{\n\"type\": \"FeatureCollection\",\n\n\"features\": [\n");
    //4.1Print obstacles
	/*fprintf(planisphere,"{\n\"type\": \"FeatureCollection\",\n\n\"features\": [\n");
	while(obstacle_list){	
		fprintf(planisphere,"{ \"type\": \"Feature\", \"properties\": { \"id\": 0 }, \"geometry\": { \"type\": \"Polygon\", \"coordinates\": [ [");
        
		GSList *obstacle = NULL;
		obstacle = g_slist_append (obstacle, (obstacle_list->data));
		while(obstacle){
		state_t *state = (state_t *)(obstacle->data);
                fprintf(planisphere,"[%3.5lf,%3.5lf]",state->x[0],state->x[1]);
		obstacle=g_slist_next(obstacle);
		}
		fprintf(planisphere,"] ] } }\n\n,\n");
		obstacle_list = g_slist_next (obstacle_list);			
	}*/
		

    // 4.2Print the path
   
	fprintf(planisphere,"{\"type\": \"Feature\", \"id\": 0, \"properties\": {}, \"geometry\": { \"type\": \"LineString\", \"coordinates\": [");



	FILE *f_ptr = fopen ("optpath.txt", "w"); 

        GSList *optstates_list = NULL;
    
        node_t *node_curr = opttree->lower_bound_node;
        while (node_curr) {
            optstates_list = g_slist_prepend (optstates_list, node_curr);
            node_curr = node_curr->parent;
        }
    

    	if (optstates_list) {
        GSList *optstates_ptr = optstates_list;
        while (optstates_ptr) {
            node_t *node_curr = (node_t *)(optstates_ptr->data);
            
            GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
            while (traj_from_parent_ptr) {
                state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
			double dist_x;
			double dist_y;
			double dist;
			dist_x=root_state.x[0]-state_this->x[0];
			dist_y=root_state.x[1]-state_this->x[1];
			dist=sqrt(dist_x*dist_x+dist_y*dist_y);
			if ((optsystem_segment_on_obstacle (opttree->optsys, root_state, state_this, 100)==0) && (dist<node_curr->distance_from_root)){
				fclose(f_ptr); 				
				f_ptr = fopen ("optpath.txt", "w");
			} 				
                  fprintf (f_ptr, "%3.5lf, %3.5lf\n", 
                     state_this->x[0], state_this->x[1]);
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
            }

			
            state_t *state_curr = node_curr->state;
		double dist_x;
			double dist_y;
			double dist;
			dist_x=root_state.x[0]-state_curr->x[0];
			dist_y=root_state.x[1]-state_curr->x[1];
			dist=sqrt(dist_x*dist_x+dist_y*dist_y);
			if ((optsystem_segment_on_obstacle (opttree->optsys, &root_state, state_curr, 100)==0) && (dist<node_curr->distance_from_root)){
				fclose(f_ptr); 				
				f_ptr = fopen ("optpath.txt", "w");
			}     
             fprintf (f_ptr, "%3.5lf %3.5lf\n", 
                     state_curr->x[0], state_curr->x[1]);
            optstates_ptr = g_slist_next (optstates_ptr);
        }
    	}

    	fclose (f_ptr);
    	f_ptr = fopen ("optpath.txt", "r");
	fprintf (planisphere,"[%3.5lf, %3.5lf,0]",root_state.x[0]*20,root_state.x[1]*20);
	while( fscanf(f_ptr,"%lf %lf",&x,&y)!=EOF){
   		fprintf (planisphere,",[%3.5lf, %3.5lf,0]",x*20,y*20);
    	        fscanf(f_ptr,"\n");
	} 
    
	fclose(f_ptr);

	fprintf(planisphere,"]}}\n");
	fprintf(planisphere,"]}\n\n\n");

	fclose(planisphere);
    // 5. Destroy the opttree structure
    	opttree_destroy (opttree);
	
    printf("%5.5lf\n",((double)(ts_now() - t))/1000000.0);
    return 1;
}
