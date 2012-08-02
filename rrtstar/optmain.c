/*
This file is a part of ``RRT(*)'', an incremental 
sampling-based optimal motion planning library.
Copyright (c) 2010 Sertac Karaman <sertac@mit.edu>, Emilio Frazzoli <frazzoli@mit.edu>

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

/*
RRT* algorithm is explained in the following paper:
http://arxiv.org/abs/1005.0416
*/

#define PUBLISH_NODES_EDGES 0

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>
#include <sys/time.h>

#include "opttree.h"

// Returns current time 
int64_t
ts_now () {
  struct timeval tv;
  gettimeofday (&tv,NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


// Extracts the optimal path and writes it into a file
int 
optmain_write_optimal_path_to_file (opttree_t *opttree) {
    
    FILE *f_ptr = fopen ("optpath.txt", "wt"); 

    GSList *optstates_list = NULL;
    {
        node_t *node_curr = opttree->lower_bound_node;
        while (node_curr) {
            optstates_list = g_slist_prepend (optstates_list, node_curr);
            node_curr = node_curr->parent;
        }
    }

    if (optstates_list) {
        GSList *optstates_ptr = optstates_list;
        while (optstates_ptr) {
            node_t *node_curr = (node_t *)(optstates_ptr->data);
            
            GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
            while (traj_from_parent_ptr) {
                state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
                  fprintf (f_ptr, "%3.5lf, %3.5lf\n", 
                     state_this->x[0], state_this->x[1]);
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
            }
            state_t *state_curr = node_curr->state;
                  fprintf (f_ptr, "%3.5lf, %3.5lf\n", 
                     state_curr->x[0], state_curr->x[1]);
            optstates_ptr = g_slist_next (optstates_ptr);
        }
    }

    fclose (f_ptr);

    return 1;
}


// Writes the whole tree into a file
int 
optmain_write_tree_to_file (opttree_t *opttree) {

    FILE *f_nodes_ptr = fopen ("nodes.txt", "wt");
    FILE *f_edges_ptr = fopen ("edges.txt", "wt");
    
    GSList *node_ptr = opttree->list_nodes; 

    while (node_ptr) {
        
        node_t *node_curr = node_ptr->data;
        fprintf (f_nodes_ptr, "%5.5lf, %5.5lf\n", 
                 node_curr->state->x[0], node_curr->state->x[1]);
        
        node_t *node_parent = node_curr->parent;        
        if (node_parent) {
            fprintf (f_edges_ptr, "%5.5lf, %5.5lf, %5.5lf, %5.5lf\n", 
                     node_curr->state->x[0], node_curr->state->x[1],
                     node_parent->state->x[0], node_parent->state->x[1]);
        }
        
        node_ptr = g_slist_next (node_ptr);
    }

    fclose (f_nodes_ptr);
    fclose (f_edges_ptr);

    return 1;
}


int main () {

    // Setup the parameters
    int num_iterations = 20000;

    // 1. Create opttree structure
    opttree_t *opttree = opttree_create ();
    if (!opttree) {
        printf ("Memory allocation error\n");
        exit (1);
    }
    
    // 2. Setup the environment
    // 2.a. create the operating region
    region_2d_t operating_region = {
        .center = {0.0, 0.0},
        .size = {20.0, 20.0}
    };
    optsystem_update_operating_region (opttree->optsys, &operating_region);
    // 2.b. create the goal region
    region_2d_t goal_region = {
        .center = {8.0, 8.0},
        .size = {1.0, 1.0}
    };
    optsystem_update_goal_region (opttree->optsys, &goal_region);
    // 2.c create obstacles
    GSList *obstacle_list = NULL;
    region_2d_t *obstacle;
    obstacle = malloc (sizeof (region_2d_t));
    obstacle->center[0] = 5.0;
    obstacle->center[1] = 5.0;
    obstacle->size[0] = 3.0;
    obstacle->size[1] = 3.0;
    obstacle_list = g_slist_prepend (obstacle_list, obstacle);
    obstacle = malloc (sizeof (region_2d_t));
    obstacle->center[0] = -5.0;
    obstacle->center[1] = -5.0;
    obstacle->size[0] = 3.0;
    obstacle->size[1] = 3.0;
    obstacle_list = g_slist_prepend (obstacle_list, obstacle);
    optsystem_update_obstacles (opttree->optsys, obstacle_list);

    // 2.d create the root state
    state_t root_state = {
        .x = {0.0, 0.0}
    };
    opttree_set_root_state (opttree, &root_state);

    opttree->run_rrtstar = 1;  // Run the RRT* algorithm
    
    // 3. Run opttree in iterations
    int64_t time_start = ts_now(); // Record the start time
    for (int i = 0; i < num_iterations; i++) {

        opttree_iteration (opttree);
        
        if ( (i != 0 ) && (i%1000 == 0) ) 
            printf ("Time: %5.5lf, Cost: %5.5lf\n", 
                    ((double)(ts_now() - time_start))/1000000.0, opttree->lower_bound); 
        
    }
    
    // 4. Save the results to the files
    optmain_write_optimal_path_to_file (opttree);
    optmain_write_tree_to_file (opttree);
        
    // 5. Destroy the opttree structure
    opttree_destroy (opttree);
    
    return 1;
}
