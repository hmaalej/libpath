
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
    char chaine[150]="";
    int i;
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
        .center = {0,0 },
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

/*while(cpt_continent<13){	
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
}*/

fgets(chaine, 2, f_obstacles);
fscanf(f_obstacles,"\n");
fgets(chaine, 29, f_obstacles);
fscanf(f_obstacles,"\n");
//fscanf(f_obstacles,"\n");
fgets(chaine, 14, f_obstacles);
fscanf(f_obstacles,"\n");

//fgets(chaine, 124, f_obstacles);
fgets(chaine, 101, f_obstacles);

	fscanf(f_obstacles,"[ %lf, %lf ]",&x,&y);
	
	node = malloc (sizeof (state_t));
  	node->x[0] = x/20;
    	node->x[1] = y/20;
       	obstacle=g_slist_prepend(obstacle,node);

	while(fscanf(f_obstacles,", [ %lf, %lf ]",&x,&y)!=0){		
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
fgets(chaine,9,f_obstacles);
fscanf(f_obstacles,"\n");


int cpt_poly=2;
while(fgetc(f_obstacles)==','){
obstacle=NULL;
fscanf(f_obstacles,"\n");
/*if ((cpt_poly<10))
fgets(chaine, 124, f_obstacles);
if ((cpt_poly==10))
fgets(chaine, 126, f_obstacles);
if ((cpt_poly<100) && (cpt_poly>10))
fgets(chaine, 127, f_obstacles);
if ((cpt_poly==100))
fgets(chaine, 129, f_obstacles);
if ((cpt_poly<1000) && (cpt_poly>100))
fgets(chaine, 130, f_obstacles);
if ((cpt_poly==1000))
fgets(chaine, 132, f_obstacles);
if ((cpt_poly<10000) && (cpt_poly>1000))
fgets(chaine, 133, f_obstacles);
if ((cpt_poly==10000))
fgets(chaine, 135, f_obstacles);
if ((cpt_poly<100000) && (cpt_poly>10000))
fgets(chaine, 136, f_obstacles);
if ((cpt_poly==100000))
fgets(chaine, 138, f_obstacles);
if ((cpt_poly<1000000) && (cpt_poly>100000))
fgets(chaine, 139, f_obstacles);*/
	fgets(chaine, 101, f_obstacles);
	fscanf(f_obstacles,"[ %lf, %lf ]",&x,&y);
	
	node = malloc (sizeof (state_t));
  	node->x[0] = x/20;
    	node->x[1] = y/20;
       	obstacle=g_slist_prepend(obstacle,node);
	while(fscanf(f_obstacles,", [ %lf, %lf ]",&x,&y)!=0){
		
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
fgets(chaine,9,f_obstacles);
fscanf(f_obstacles,"\n");
printf("%d\n",cpt_poly);
cpt_poly++;
}

     
    fclose(f_obstacles);
   printf("c fini \n");
    
    optsystem_update_obstacles (opttree->optsys, obstacle_list);

    // 2.c create the root state
    state_t root_state = {
        .x = {atof(argv[1])/20, atof(argv[2])/20}
    };
    opttree_set_root_state (opttree, &root_state);

    // 2.d. create the goal region
	state_t arrival_state = {
        .x = {atof(argv[3])/20, atof(argv[4])/20}
    	};
	double d=optsystem_evaluate_distance (opttree->optsys, &root_state, &arrival_state);
	int s;
	if (d<1) {s=0.05;} else {s=0.2;}
	region_2d_t goal_region = {
        	.center = {atof(argv[3])/20, atof(argv[4])/20},
        	.size = {s,s}
    	};
	
    optsystem_update_goal_region (opttree->optsys, &goal_region);
 /*
    //2.e rectangle
GSList *rectangle_list = NULL;
GEOSCoordSequence* cs3;
	GEOSGeometry* g3;  
cs3= GEOSCoordSeq_create(5,2);

k=GEOSCoordSeq_setX(cs3, 0, 2.1);
k=GEOSCoordSeq_setY(cs3, 0, 1.535);

k=GEOSCoordSeq_setX(cs3, 1, 5.8);
k=GEOSCoordSeq_setY(cs3, 1, 1.535);

k=GEOSCoordSeq_setX(cs3, 2, 5.8);
k=GEOSCoordSeq_setY(cs3, 2, 3.29);

k=GEOSCoordSeq_setX(cs3, 3, 2.1);
k=GEOSCoordSeq_setY(cs3, 3, 3.29);

k=GEOSCoordSeq_setX(cs3, 4, 2.1);
k=GEOSCoordSeq_setY(cs3, 4, 1.535);


shell = GEOSGeom_createLinearRing(cs3);
g3 = GEOSGeom_createPolygon(shell, NULL, 0);	   
    rectangle_list=g_slist_prepend(rectangle_list,g3);

k=GEOSCoordSeq_setX(cs3, 0, -0.5);
k=GEOSCoordSeq_setY(cs3, 0, 0.315);

k=GEOSCoordSeq_setX(cs3, 1, 1.66);
k=GEOSCoordSeq_setY(cs3, 1, 0.315);

k=GEOSCoordSeq_setX(cs3, 2, 1.66);
k=GEOSCoordSeq_setY(cs3, 2, 1.5);

k=GEOSCoordSeq_setX(cs3, 3, -0.5);
k=GEOSCoordSeq_setY(cs3, 3, 1.5);

k=GEOSCoordSeq_setX(cs3, 4, -0.5);
k=GEOSCoordSeq_setY(cs3, 4, 0.315);


shell = GEOSGeom_createLinearRing(cs3);
g3 = GEOSGeom_createPolygon(shell, NULL, 0);	   
    rectangle_list=g_slist_prepend(rectangle_list,g3);

k=GEOSCoordSeq_setX(cs3, 0, 0.69);
k=GEOSCoordSeq_setY(cs3, 0, -0.95);

k=GEOSCoordSeq_setX(cs3, 1, 1.8);
k=GEOSCoordSeq_setY(cs3, 1, -0.95);

k=GEOSCoordSeq_setX(cs3, 2, 1.8);
k=GEOSCoordSeq_setY(cs3, 2, 0.315);

k=GEOSCoordSeq_setX(cs3, 3, 0.69);
k=GEOSCoordSeq_setY(cs3, 3, 0.315);

k=GEOSCoordSeq_setX(cs3, 4, 0.69);
k=GEOSCoordSeq_setY(cs3, 4, -0.95);


shell = GEOSGeom_createLinearRing(cs3);
g3 = GEOSGeom_createPolygon(shell, NULL, 0);	   
    rectangle_list=g_slist_prepend(rectangle_list,g3);

 optsystem_update_rectangles (opttree->optsys, rectangle_list);
*/
finishGEOS();
	
    // 3. Run opttree in iterations
    int64_t time_start = ts_now(); 
    gboolean b=FALSE;
if (optsystem_segment_on_obstacle (opttree->optsys, &root_state, &goal_region.center, 100)==0){
	 FILE *f_ptr = fopen ("optpath.txt", "w"); 
	 fprintf (f_ptr, "%3.5lf %3.5lf\n", 
                     root_state.x[0], root_state.x[1]);
	 fprintf (f_ptr, "%3.5lf %3.5lf\n", 
                     goal_region.center[0], goal_region.center[1]);
	 fclose(f_ptr);
fprintf(planisphere,"{\n\"type\": \"FeatureCollection\",\n\n\"features\": [\n");
fprintf(planisphere,"{\"type\": \"Feature\", \"id\": 0, \"properties\": {}, \"geometry\": { \"type\": \"LineString\", \"coordinates\": [");
fprintf (planisphere,"[%3.5lf, %3.5lf,0]",root_state.x[0]*20,root_state.x[1]*20);
fprintf (planisphere,",[%3.5lf, %3.5lf,0]",goal_region.center[0]*20,goal_region.center[1]*20);	
}     
else {         
    for ( i = 0; i < num_iterations; i++) {
	
        opttree_iteration (opttree);
          
        if ( (i != 0 ) && (i != 100 ) && (i%100 == 0)  ) {
	    if ((opttree->lower_bound<99999) && (b==FALSE)){
		b=TRUE;
		/*x=i/1000;
		num_iterations=i+(i*40*log(x))/(x*x);*/
		if ((opttree->lower_bound)*20<50)
		num_iterations=i+1002;
		if (((opttree->lower_bound)*20>=50) && ((opttree->lower_bound)*20<200))
		num_iterations=i+2002;
		if (((opttree->lower_bound)*20>=200)&&((opttree->lower_bound)*20<400) )
		num_iterations=i+3002;
		if (((opttree->lower_bound)*20>=400))
		num_iterations=i+4002;
	
	    }
            printf ("Time: %5.5lf, Cost: %5.5lf\n", 
                    ((double)(ts_now() - time_start))/1000000.0, (opttree->lower_bound)*20); 
        }
    }
	printf("%d\n",i);
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



	FILE* f_ptr = fopen ("optpath.txt", "w"); 

        GSList *optstates_list = NULL;
	double tx[300];
	double ty[300];
	double tx_i[300];
	double ty_i[300];
	double tx2[300];
	double ty2[300];
	double tx2_i[300];
	double ty2_i[300];
	double tx3[300];
	double ty3[300];
	double tx3_i[300];
	double ty3_i[300];
		
	// list des points formants le chemmin ideal
	
	
	
        node_t *node_curr = opttree->lower_bound_node;
        while (node_curr) {
            optstates_list = g_slist_prepend (optstates_list, node_curr);
            node_curr = node_curr->parent;
        }
    	//modiiif
	/*tx = (double *) malloc (g_slist_length(optstates_list)*sizeof(double));
	ty = (double *) malloc (g_slist_length(optstates_list)*sizeof(double));
	tx_i = (double *) malloc (g_slist_length(optstates_list)*sizeof(double));
	tx_i = (double *) malloc (g_slist_length(optstates_list)*sizeof(double));
	*/
	


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
    	/*f_ptr = fopen ("optpath.txt", "r");
	fprintf (planisphere,"[%3.5lf, %3.5lf,0]",root_state.x[0]*20,root_state.x[1]*20);
	while( fscanf(f_ptr,"%lf %lf",&x,&y)!=EOF){
   		fprintf (planisphere,",[%3.5lf, %3.5lf,0]",x*20,y*20);
    	        fscanf(f_ptr,"\n");
	} 
	fclose(f_ptr);*/
    	//algo modif
		//parcours de haut en bas
		f_ptr = fopen ("optpath.txt", "r");
		tx[0]=root_state.x[0];
		ty[0]=root_state.x[1];
		int i=0;
		while( fscanf(f_ptr,"%lf %lf",&x,&y)!=EOF){
			i++;  
			tx[i]=(x+tx[i-1])/2;
			ty[i]=(y+ty[i-1])/2; 		
			tx[i+1]=x;
			ty[i+1]=y;   		
			i++;
    	        	fscanf(f_ptr,"\n");
			
		} 
		
		int nb=i;
		tx_i[0]=tx[nb];
		ty_i[0]=ty[nb];
		//indice tableau resultat
		k = 1;
		i=nb;
		while (i != 0) {
			state_t state_a = {
        				.x = {tx[i], ty[i]}
    				};			
			b=0;
			j=0;
			while( !b ){
				state_t state_b = {
        				.x = {tx[j], ty[j]}
    				};
				if(optsystem_segment_on_obstacle (opttree->optsys, &state_a, &state_b, 100)==0){
					tx_i[k]=tx[j];
					ty_i[k]=ty[j];
					k++;
					i=j;
					b=1;
				}
			
			j++;
			}
			printf("%d\n",i);
		}
		k--;

		//parcours de bas en haut

		tx2[0]=tx_i[0];
		ty2[0]=ty_i[0];
				
		int ii=0;
		for(ii=1;ii<k+1;ii++){  
			tx2[2*ii-1]=(tx_i[ii]+tx2[2*ii-2])/2;
			ty2[2*ii-1]=(ty_i[ii]+ty2[2*ii-2])/2; 		
			tx2[2*ii]=tx_i[ii];
			ty2[2*ii]=ty_i[ii];   		
		} 
		int nb2=2*k;
		tx2_i[0]=tx2[nb2];
		ty2_i[0]=ty2[nb2];
		//indice tableau resultat
		k = 1;
		i=nb2;
		printf("zaaaaaab\n");
		printf("%d\n",i);
		
		while (i != 0) {
			state_t state2_a = {
        				.x = {tx2[i], ty2[i]}
    				};			
			b=0;
			j=0;
			while( !b ){
				state_t state2_b = {
        				.x = {tx2[j], ty2[j]}
    				};
				if(optsystem_segment_on_obstacle (opttree->optsys, &state2_a, &state2_b, 100)==0){
					tx2_i[k]=tx2[j];
					ty2_i[k]=ty2[j];
					k++;
					i=j;
					b=1;
				}
			
			j++;
			
			}
			printf("%d\n",i);
		}
		k--;

		//parcours tous points->milieux

		tx3[0]=tx2_i[0];
		ty3[0]=ty2_i[0];
				
		ii=0;
		for(ii=1;ii<k+1;ii++){  
			tx3[2*ii-1]=(tx2_i[ii]+tx3[2*ii-2])/2;
			ty3[2*ii-1]=(ty2_i[ii]+ty3[2*ii-2])/2; 		
			tx3[2*ii]=tx2_i[ii];
			ty3[2*ii]=ty2_i[ii];   		
		} 
		int nb3=2*k;
		tx3_i[0]=tx3[nb3];
		ty3_i[0]=ty3[nb3];
		printf("%lf , %lf \n",tx3_i[0],ty3_i[0]);
		//indice tableau resultat
		k = 1;
		i=nb3;
		printf("zaaaaaab\n");
		printf("%d\n",i);
		
		while (i != 0) {
			
			state_t state3_a = {
        				.x = {tx3[i], ty3[i]}
    				};			
			b=0;
			j=0;
			while( !b ){
				
				if ((j%2)==1){
					
					state_t state3_b = {
        					.x = {tx3[j], ty3[j]}
    					};
					if(optsystem_segment_on_obstacle (opttree->optsys, &state3_a, &state3_b, 100)==0){
						
						tx3_i[k]=tx3[j];
						ty3_i[k]=ty3[j];
						k++;
						i=j;
						b=1;
					}
					
					
				}
				if (i==j) {
						tx3_i[k]=tx3[j-1];
						ty3_i[k]=ty3[j-1];
						k++;
						i=j-1;
						b=1;
							
					}
				j++;
			}
			printf("%d\n",i);
		}
		k--;

/*
		tx2[0]=tx_i[k];
		ty2[0]=ty_i[k];
		int ii=0;
		for(ii=1;ii<k;ii++){  
			tx2[2*ii-1]=(tx_i[k-ii]+tx2[2*ii-2])/2;
			ty2[2*ii-1]=(ty_i[k-ii]+ty2[2*ii-2])/2; 		
			tx2[2*ii]=tx_i[k-ii];
			ty2[2*ii]=ty_i[k-ii];   		
			
		} 
		
		int nb2=2*ii;
		tx2_i[0]=tx2[0];
		ty2_i[0]=ty2[0];
		//indice tableau resultat
		k = 1;
		i=0;
		while (i != nb2) {
			state_t state2_a = {
        				.x = {tx2[i], ty2[i]}
    				};			
			b=0;
			j=nb2;
			while( !b ){
				state_t state2_b = {
        				.x = {tx2[j], ty2[j]}
    				};
				if(optsystem_segment_on_obstacle (opttree->optsys, &state2_a, &state2_b, 100)==0){
					tx2_i[k]=tx2[j];
					ty2_i[k]=ty2[j];
					k++;
					i=j;
					b=1;
				}
			
			j--;
			}
			printf("%d\n",i);
		}
		printf("zaaaaaab\n");
	k--;
		printf("%d\n",k);
	
*/		//le ,
	printf("zaaaaaab\n");
	printf("%d\n",k);
	fprintf (planisphere,"[%3.5lf, %3.5lf,0]",tx3_i[k]*20,ty3_i[k]*20);
	while( k!=0 ){
		printf("%d\n",k);
		k--;
   		fprintf (planisphere,",[%3.5lf, %3.5lf,0]",tx3_i[k]*20,ty3_i[k]*20);
	} 

		

	fclose(f_ptr);
}
	fprintf(planisphere,"]}}\n");
	fprintf(planisphere,"]}\n\n\n");

	fclose(planisphere);

	printf("distance: %lf\n",d);
    // 5. Destroy the opttree structure
    	opttree_destroy (opttree);
	
    printf("%5.5lf\n",((double)(ts_now() - t))/1000000.0);
    return 1;
}
