
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
    int k=20;
    // Setup the num_iterations
    int num_iterations = 1000000;
    // 1. Create opttree structure
    opttree_t *opttree = opttree_create ();
    if (!opttree) {
        printf ("Memory allocation error\n");
        exit (1);
    }
    
    // 2. Setup the environment
    // 2.a. create the operating region
 int c1,c2,s1,s2;
    if ((atof(argv[1])>-10) && (atof(argv[1])<50) && (atof(argv[2])>30) && (atof(argv[2])<50) && (atof(argv[3])>-10) && (atof(argv[3])<50) && (atof(argv[4])>30) && (atof(argv[4])<50)) {
	c1= 20/k;
	c2= 40/k;
	s1= 60/k;
	s2= 20/k;
    }	
    else  if ((atof(argv[1])>16) && (atof(argv[1])<30) && (atof(argv[2])>54) && (atof(argv[2])<66) && (atof(argv[3])>16) && (atof(argv[3])<30) && (atof(argv[4])>54) && (atof(argv[4])<66)) {
	
	c1= 23/k;
	c2= 3;
	s1= 14/k;
	s2= 12/k;
    }	
    else if  ((atof(argv[1])>6) && (atof(argv[1])<16) && (atof(argv[2])>53) && (atof(argv[2])<60) && (atof(argv[3])>6) && (atof(argv[3])<16) && (atof(argv[4])>53) && (atof(argv[4])<60)) {
	c1= 11/k;
	c2= 56.5/k;
	s1= 10/k;
	s2= 7/k;
    } 
    else {
	c1= 0;
	c2=0;
	s1=360/k;
	s2=180/k;
    }
	
	region_2d_t operating_region = {
        .center = {c1,c2 },
        .size = {s1,s2 }
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
	int k_;
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
  	node->x[0] = x/k;
    	node->x[1] = y/k;
       	obstacle=g_slist_prepend(obstacle,node);

	while(fscanf(f_obstacles,", [ %lf, %lf ]",&x,&y)!=0){		
		node = malloc (sizeof (state_t));
  		node->x[0] = x/k;
    		node->x[1] = y/k;
       		obstacle=g_slist_prepend(obstacle,node);
	}
	cs= GEOSCoordSeq_create(g_slist_length (obstacle),2);
	int j=0;
	while(obstacle) {
		state_t *point=(state_t *)(obstacle->data);
		k_=GEOSCoordSeq_setX(cs, j, point->x[0]);
		k_=GEOSCoordSeq_setY(cs, j, point->x[1]);
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
  	node->x[0] = x/k;
    	node->x[1] = y/k;
       	obstacle=g_slist_prepend(obstacle,node);
	while(fscanf(f_obstacles,", [ %lf, %lf ]",&x,&y)!=0){
		
		node = malloc (sizeof (state_t));
  		node->x[0] = x/k;
    		node->x[1] = y/k;
       		obstacle=g_slist_prepend(obstacle,node);
	}
	cs= GEOSCoordSeq_create(g_slist_length (obstacle),2);
	int j=0;
	while(obstacle) {
		state_t *point=(state_t *)(obstacle->data);
		k_=GEOSCoordSeq_setX(cs, j, point->x[0]);
		k_=GEOSCoordSeq_setY(cs, j, point->x[1]);
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
        .x = {atof(argv[1])/k, atof(argv[2])/k}
    };
    

    // 2.d. create the goal region
	state_t arrival_state = {
        .x = {atof(argv[3])/k, atof(argv[4])/k}
    	};
	//on sort du point de la mediteranée
	if ((atof(argv[3])>27) && (atof(argv[3])<42) && (atof(argv[4])>40) && (atof(argv[4])<48)){
		state_t state_perm=root_state;
		root_state=arrival_state;
		arrival_state=state_perm;
	}
	
	else if ((atof(argv[3])>-10) && (atof(argv[3])<50) && (atof(argv[4])>30) && (atof(argv[4])<50) && ((atof(argv[1])<27) || (atof(argv[1])>42) || (atof(argv[2])<40) || (atof(argv[2])>48))){
		state_t state_perm=root_state;
		root_state=arrival_state;
		arrival_state=state_perm;
	}
	printf("departure:(%lf,%lf)\n",root_state.x[0]*k,root_state.x[1]*k);
	printf("arrival:(%lf,%lf)\n",arrival_state.x[0]*k,arrival_state.x[1]*k);
	double d=optsystem_evaluate_distance (opttree->optsys, &root_state, &arrival_state);
	int s;
	if (d<20/k) {s=1/k;} else {s=4/k;}
	region_2d_t goal_region = {
        	.center = {arrival_state.x[0],arrival_state.x[1]},
        	.size = {s,s}
    	};
    opttree_set_root_state (opttree, &root_state);
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
fprintf (planisphere,"[%3.5lf, %3.5lf,0]",root_state.x[0]*k,root_state.x[1]*k);
fprintf (planisphere,",[%3.5lf, %3.5lf,0]",goal_region.center[0]*k,goal_region.center[1]*k);	
}     
else {   
    gboolean b_aret=FALSE; 
    i=0; 
    double ts_find=300;  
    for(i=0;i<num_iterations;i++){  
	if (!b_aret)
        opttree_iteration (opttree); 
        if ( (i != 0 ) && (i%100 == 0)  ) {
	    if ((opttree->lower_bound<99999) && (b==FALSE)){
		b=TRUE;
		
		/*x=i/1000;
		num_iterations=i+(i*40*log(x))/(x*x);*/
		if ((opttree->lower_bound)*k<50)
		num_iterations=i+502;
		if (((opttree->lower_bound)*k>=50) && ((opttree->lower_bound)*k<200))
		num_iterations=i+1002;
		if (((opttree->lower_bound)*k>=200)&&((opttree->lower_bound)*k<400) )
		num_iterations=i+2002;
		if (((opttree->lower_bound)*k>=400))
		num_iterations=i+3002;
		//cas particulier: 
			//il y'a deja asez d'iteration
		if (i>50000)
		num_iterations=i+502;
		ts_find=(ts_now() - time_start)/1000000.0;
			//il y'a trés peu d'iteration
		//if (i<3000)
		//num_iterations=i+1002;
		//num_iterations=i+200;
	    }
            printf ("Time: %5.5lf, Cost: %5.5lf\n", 
                    ((double)(ts_now() - time_start))/1000000.0, (opttree->lower_bound)*k);   
        }
	if ((double)(ts_now() - time_start)/1000000.0-ts_find>10)
	    b_aret=TRUE;
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
	double tx_f[3000];
	double ty_f[3000];
	double tx_x[3000];
	double ty_y[3000];
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
		k_ = 1;
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
					tx_i[k_]=tx[j];
					ty_i[k_]=ty[j];
					k_++;
					i=j;
					b=1;
				}
			
			j++;
			}
			printf("%d\n",i);
		}
		
		int nb_point=k_;
		k_--; 
		printf("nb_point=%d\n",nb_point);
		/*
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
		k--;*/
/*
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
*/
//optimisation chemin
//tx_f[0]=tx_i[0];
//ty_f[0]=ty_i[0];
/*
tx[0]=root_state.x[0];
ty[0]=root_state.x[1];
i=0;
while( fscanf(f_ptr,"%lf %lf",&x,&y)!=EOF){
			i++;  	
			tx[i]=x;
			ty[i]=y;   		
			fscanf(f_ptr,"\n");
} 
int nb=i;
int nb_point=0;
tx_f[0]=tx[0];
ty_f[0]=ty[0];
for (i=0;i<nb;i++){
	x=-(tx[i]-tx[i+1])/10;
	y=-(ty[i]-ty[i+1])/10;
	for ( k=1;k<11;k++) {
		tx_f[k+10*i]=tx_f[k+10*i-1]+x;
		ty_f[k+10*i]=ty_f[k+10*i-1]+y;
		nb_point++;
	}
}
printf("nb_point=%d\n",nb_point);
k=nb_point;
int ind=0;
int nb_final=0;
i=0;
//for(i=0;i<nb_point;i++){
while(ind!=nb_point){
	state_t state_i = {
        				.x = {tx_f[i], ty_f[i]}
    				};	
				
	state_t state_f = {
        				.x = {tx_f[nb_point], ty_f[nb_point]}
    				};			
	
	gboolean b=0;
	state_t state_trans = state_f ;
	int cpt=nb_point;
	while(!b) {
		if ((cpt-ind<11) && (cpt%10==0)){
			tx_f[i+1]=tx_f[ind+1];
			ty_f[i+1]=ty_f[ind+1];
			b=1;
			ind++;
			nb_final++;
		} else if(optsystem_segment_on_obstacle (opttree->optsys, &state_i, &state_trans, 100)==0){
			printf("tasli7\n");			
			tx_f[i+1]= state_trans.x[0];
			ty_f[i+1]= state_trans.x[1];
			b=1;
			ind=cpt;
			nb_final++;
		}
		cpt--;
		state_trans.x[0]=tx_f[cpt];
		state_trans.x[1]=ty_f[cpt];
		
	}
i++;
}


		tx_i[0]=tx_f[nb_final];
		ty_i[0]=ty_f[nb_final];
		//indice tableau resultat
		k = 1;
		i=nb_final;
		while (i != 0) {
			state_t state_a = {
        				.x = {tx_f[i], ty_f[i]}
    				};			
			b=0;
			j=0;
			while( !b ){
				state_t state_b = {
        				.x = {tx_f[j], ty_f[j]}
    				};
				if(optsystem_segment_on_obstacle (opttree->optsys, &state_a, &state_b, 100)==0){
					tx_i[k]=tx_f[j];
					ty_i[k]=ty_f[j];
					k++;
					i=j;
					b=1;
				}
			
			j++;
			}
			printf("%d\n",i);
		}
		k--;
		int nb_points=k;
		


for (i=0;i<nb_points;i++){
	x=-(tx_i[i]-tx_i[i+1])/10;
	y=-(ty_i[i]-ty_i[i+1])/10;
	for ( k=1;k<11;k++) {
		tx_f[k+10*i]=tx_f[k+10*i-1]+x;
		ty_f[k+10*i]=ty_f[k+10*i-1]+y;
		nb_point++;
	}
}
printf("nb_point=%d\n",nb_point);
k=nb_point;
ind=0;
nb_final=0;
i=0;
//for(i=0;i<nb_point;i++){
while(ind!=nb_point){
	state_t state_i = {
        				.x = {tx_f[i], ty_f[i]}
    				};	
				
	state_t state_f = {
        				.x = {tx_f[nb_point], ty_f[nb_point]}
    				};			
	
	gboolean b=0;
	state_t state_trans = state_f ;
	int cpt=nb_point;
	while(!b) {
		if ((cpt-ind<11) && (cpt%10==0)){
			tx_f[i+1]=tx_f[ind+1];
			ty_f[i+1]=ty_f[ind+1];
			b=1;
			ind++;
			nb_final++;
		} else if(optsystem_segment_on_obstacle (opttree->optsys, &state_i, &state_trans, 100)==0){
			printf("tasli7\n");			
			tx_f[i+1]= state_trans.x[0];
			ty_f[i+1]= state_trans.x[1];
			b=1;
			ind=cpt;
			nb_final++;
		}
		cpt--;
		state_trans.x[0]=tx_f[cpt];
		state_trans.x[1]=ty_f[cpt];
		
	}
i++;
}
*/


for(i=0;i<nb_point-2;i++){
	state_t state_0 = {
        				.x = {tx_i[i], ty_i[i]}
    				};	
	state_t state_1 = {
        				.x = {tx_i[i+1], ty_i[i+1]}
    				};			
	state_t state_2 = {
        				.x = {tx_i[i+2], ty_i[i+2]}
    				};			
	
	//la division du segment doit etre en fonction de sa longeur a faire aprés	
	x=(tx_i[i+1]-tx_i[i+2])/30;
	y=(ty_i[i+1]-ty_i[i+2])/30;
	gboolean b=0;
	state_t state_trans = state_2;
	while(!b) {
		if(optsystem_segment_on_obstacle (opttree->optsys, &state_0, &state_trans, 100)==0){
			printf("tasli7\n");			
			tx_i[i+1]= state_trans.x[0];
			ty_i[i+1]= state_trans.x[1];
			b=1;
		}
		state_trans.x[0]=state_trans.x[0]+x;
		state_trans.x[1]=state_trans.x[1]+y;
	}
}

for(i=nb_point-1;i>1;i--){
	state_t state_0 = {
        				.x = {tx_i[i], ty_i[i]}
    				};	
	state_t state_1 = {
        				.x = {tx_i[i-1], ty_i[i-1]}
    				};			
	state_t state_2 = {
        				.x = {tx_i[i-2], ty_i[i-2]}
    				};			
	
	//la division du segment doit etre en fonction de sa longeur a faire aprés	
	x=(tx_i[i-1]-tx_i[i-2])/30;
	y=(ty_i[i-1]-ty_i[i-2])/30;
	gboolean b=0;
	state_t state_trans = state_2;
	while(!b) {
		if(optsystem_segment_on_obstacle (opttree->optsys, &state_0, &state_trans, 100)==0){
			tx_i[i-1]= state_trans.x[0];
			ty_i[i-1]= state_trans.x[1];
			b=1;
		}
		state_trans.x[0]=state_trans.x[0]+x;
		state_trans.x[1]=state_trans.x[1]+y;
	}
}

//derniere optim

	double px[300];
	double py[300];
	double d1x,d1y,d2x,d2y;
	gboolean b1,b2;
	px[0]=tx_i[0];
	py[0]=ty_i[0];
	j=1;
	int nb_final;
	int k1, k2;
	
	for(i=1;i<nb_point-1;i++){
		printf("i=%d\n",i);
		d1x=(tx_i[i-1]-tx_i[i])/5;
		d1y=(ty_i[i-1]-ty_i[i])/5;
		d2x=(tx_i[i+1]-tx_i[i])/5;
		d2y=(ty_i[i+1]-ty_i[i])/5;
		
		k2=2;
		k1=1;
		b1=1;
		b2=1;
		state_t state_0 = {
        				.x = {tx_i[i]+d1x, ty_i[i]+d1y}
    				};
			
		while((b1) && (k1<5)){
			
			
			state_t state_1 = {
				.x = {tx_i[i]+k1*d2x, ty_i[i]+k1*d2y}
			};
			
			if (optsystem_segment_on_obstacle (opttree->optsys, &state_0, &state_1, 100) == 0){
				k1++;
				//printf("(%lf,%lf),(%lf,%lf)\n",state_0.x[0]*20,state_0.x[1]*20,state_1.x[0]*20,state_1.x[1]*20);
				
			}
			else {
				printf("aywaaaah\n");
				b1=0;
				k1--;
			}
		}
		printf("k1=%d\n",k1);
		if (k1!=0){
			state_t state_1 = {
				.x = {tx_i[i]+k1*d2x, ty_i[i]+k1*d2y}
			};
			while((b2) && (k2<5)){
				//printf("%d\n",k2);
				state_t state_2 = {
        				.x = {tx_i[i]+k2*d1x, ty_i[i]+k2*d1y}
				};
			
				if(optsystem_segment_on_obstacle (opttree->optsys, &state_1, &state_2, 100)==0){
					k2++;
					//if (k2<20)
					//printf("(%lf,%lf),(%lf,%lf)\n",state_2.x[0]*20,state_2.x[1]*20,state_1.x[0]*20,state_1.x[1]*20);
				}
				else {
					//printf("%d\n",k2);
					b2=0;
					k2--;
				}
			}
		}
		if (k1!=0){
			if (j==1){
				px[j]=tx_i[i]+k2*d1x;
				py[j]=ty_i[i]+k2*d1y;
				px[j+1]=tx_i[i]+k1*d2x;
				py[j+1]=ty_i[i]+k1*d2y;
				j=j+2;
			} 
			else {
				px[j]=tx_i[i]+k2*d1x;
				py[j]=ty_i[i]+k2*d1y;
				px[j+1]=tx_i[i]+k1*d2x;
				py[j+1]=ty_i[i]+k1*d2y;
				

				if ((px[j]-px[j-1])*(tx_i[i]-tx_i[i-1])>0){
					j=j+2;
				}
				else {
					px[j-1]=px[j];
					px[j]=px[j+1];
					py[j-1]=py[j];
					py[j]=py[j+1];
					j++;
				}

			}			
		}
		else {
			px[j]=tx_i[i];
			py[j]=ty_i[i];
			j++;
		}
	}
	px[j]=tx_i[nb_point-1];
	py[j]=ty_i[nb_point-1];
	nb_final=j;
	

	double px1[300];
	double py1[300];
	//double d1x,d1y,d2x,d2y;
	//gboolean b1,b2;
	px1[0]=px[nb_final];
	py1[0]=py[nb_final];
	j=1;
	//int nb_final;
	//int k1, k2;
	for(i=nb_final-1;i>0;i--){
		
		d1x=(px[i-1]-px[i])/5;
		d1y=(py[i-1]-py[i])/5;
		d2x=(px[i+1]-px[i])/5;
		d2y=(py[i+1]-py[i])/5;
		
		k2=2;
		k1=1;
		b1=1;
		b2=1;
		state_t state_0 = {
        				.x = {px[i]+d1x, py[i]+d1y}
    				};
			
		while((b1) && (k1<5)){
			//printf("%d\n",k1);
			
			state_t state_1 = {
				.x = {px[i]+k1*d2x, py[i]+k1*d2y}
			};
			
			if (optsystem_segment_on_obstacle (opttree->optsys, &state_0, &state_1, 100) == 0){
				k1++;
				//printf("(%lf,%lf),(%lf,%lf)\n",state_0.x[0],state_0.x[1],state_1.x[0],state_1.x[1]);
				
			}
			else {
				printf("aywaaaah\n");
				b1=0;
				k1--;
			}
		}
		if (k1!=0){
			state_t state_1 = {
				.x = {px[i]+k1*d2x, py[i]+k1*d2y}
			};
			while((b2) && (k2<5)){
				state_t state_2 = {
        				.x = {px[i]+k2*d1x, py[i]+k2*d1y}
				};
			
				if(optsystem_segment_on_obstacle (opttree->optsys, &state_1, &state_2, 100)==0){
					k2++;
				}
				else {
					b2=0;
					k2--;
				}
			}
		}
		
		if (k1!=0){
			if (j==1){
				px1[j+1]=px[i]+k2*d1x;
				py1[j+1]=py[i]+k2*d1y;
				px1[j]=px[i]+k1*d2x;
				py1[j]=py[i]+k1*d2y;
				j=j+2;
			} 
			else {
				px1[j+1]=px[i]+k2*d1x;
				py1[j+1]=py[i]+k2*d1y;
				px1[j]=px[i]+k1*d2x;
				py1[j]=py[i]+k1*d2y;

				if ((px1[j]-px1[j-1])*(px[i]-px[i+1])>0){
					j=j+2;
				}
				else {
					px1[j-1]=px1[j];
					px1[j]=px1[j+1];
					py1[j-1]=py1[j];
					py1[j]=py1[j+1];
					j++;
				}

			}			
		}
		else {
			px1[j]=px[i];
			py1[j]=py[i];
			j++;
		}
	}
	px1[j]=px[0];
	py1[j]=py[0];
	int nb_final1=j;
		
		
			










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
*/	
		//le ,	
	/*printf("zaaaaaab\n");
	printf("%d\n",nb_point);
	k=nb_point-1;
	fprintf (planisphere,"[%3.5lf, %3.5lf,0]",tx_i[k]*20,ty_i[k]*20);
	while( k!=0 ){
		printf("%d\n",k);
		k--;
   		fprintf (planisphere,",[%3.5lf, %3.5lf,0]",tx_i[k]*20,ty_i[k]*20);
	} 
*/

	printf("nb_final1=%d\n",nb_final1);
	k_=nb_final1;
	fprintf (planisphere,"[%3.5lf, %3.5lf,0]",px1[k_]*k,py1[k_]*k);
	while( k_!=0 ){
		printf("%d\n",k_);
		k_--;
   		fprintf (planisphere,",[%3.5lf, %3.5lf,0]",px1[k_]*k,py1[k_]*k);
	} 

		

	fclose(f_ptr);
}
	fprintf(planisphere,"]}}\n");
	fprintf(planisphere,"]}\n\n\n");

	fclose(planisphere);

	//printf("distance: %lf\n",d);
    // 5. Destroy the opttree structure
    	opttree_destroy (opttree);
	
    printf("%5.5lf\n",((double)(ts_now() - t))/1000000.0);
    return 1;
} 
