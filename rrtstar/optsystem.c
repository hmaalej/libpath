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

#include "optsystem.h"


// Returns 1 iff (state) is on an obstacle
gboolean 
optsystem_on_obstacle (optsystem_t *self, state_t *state);

// Returns 1 iff the line connecting (state_initial) and (state_final) lies on an obstacle
int 
optsystem_segment_on_obstacle (optsystem_t *self, state_t *state_initial, state_t *state_final, int num_steps);


// Allocates memory for and initializes an empty dynamical system
int optsystem_new_system (optsystem_t *self) {
    
    self->initial_state = (state_t *) malloc (sizeof(state_t));
    for (int i = 0; i < NUM_STATES; i++)
        self->initial_state->x[i] = 0.0;

    self->obstacle_list = NULL;
    
    return 1;
}


// Frees up the memory used by optsystem_t *self
int optsystem_free_system (optsystem_t *self) {
    
    free(self->initial_state);
    
    return 1;    
}


// Allocates memory for a new state
state_t* optsystem_new_state (optsystem_t *self) {
    state_t *state = (state_t *) malloc (sizeof (state_t)); 
    for (int i = 0; i < NUM_STATES; i++)
        state->x[i] = 0.0;
    return state;
}


// Frees up the memory used by a given state
int optsystem_free_state (optsystem_t *self, state_t *state) {
    free (state);
    return 1;
}


// Allocates memory for a new state
input_t* optsystem_new_input (optsystem_t *self) {
    input_t *input = (input_t *) malloc (sizeof (input_t));
    for (int i = 0; i < NUM_INPUTS; i++)
        input->x[i] = 0.0;
    return input;
}


// Frees up the memory used by a given state
int optsystem_free_input (optsystem_t *self, input_t *input) {
    free (input);
    return 1;
}


// Create a copy of the given state
state_t *optsystem_clone_state (optsystem_t *self, state_t *state) {
    state_t *stateNew = (state_t *) malloc (sizeof (state_t));
    for (int i = 0; i < NUM_STATES; i++)
        stateNew->x[i] = state->x[i];
    return stateNew;
}


// Sets the initial state to a particular value
int optsystem_set_initial_state (optsystem_t *self, state_t *state) {
    
    for (int i = 0; i < NUM_STATES; i++)
        self->initial_state->x[i] = state->x[i];
    return 1;
}


// Returns the initial state of the system
int  optsystem_get_initial_state (optsystem_t *self, state_t *state) {
    for (int i = 0; i < NUM_STATES; i++) 
        state->x[i] = self->initial_state->x[i];
    return 1;
}


// Returns the ``number of dimensions''
int optsystem_get_num_states (optsystem_t *self) {
    return NUM_STATES;
}


// Returns a double array (of size optsystem_get_num_states) 
//  used for storing the state in a spatial data structure
double* optsystem_get_state_key (optsystem_t *self, state_t *state) {
    return  state->x;
}


// Creates a random state
int optsystem_sample_state (optsystem_t *self, state_t *random_state) { 

    for (int i = 0; i < NUM_STATES; i++) {
        random_state->x[i] = self->operating_region.size[i]*rand()/(RAND_MAX + 1.0)
            + self->operating_region.center[i] - self->operating_region.size[i]/2.0;
    }

    if ( optsystem_on_obstacle (self, random_state) ) 
        return 0;
    
    return 1;
}


// Creates a random state
int optsystem_sample_target_state (optsystem_t *self, state_t *random_state) { 

    for (int i = 0; i < NUM_STATES; i++) {
        random_state->x[i] = self->goal_region.size[i]*rand()/(RAND_MAX + 1.0)
            + self->goal_region.center[i] - self->goal_region.size[i]/2.0;
    }

    if ( optsystem_on_obstacle (self, random_state) ) 
        return 0;
    
    return 1;
}


// Evaluates the Euclidean distance between two states -- used mainly for the Nearest and CloseNodes procedures
double optsystem_evaluate_distance (optsystem_t *self, state_t *state_from, state_t *state_to) {
    
    double dist = 0;
    for (int i = 0; i < NUM_STATES; i++) {
        double dist_this = state_to->x[i] - state_from->x[i];
        dist += dist_this * dist_this;
    }

    return sqrt (dist);
}


// Evaluates the cost when traversing a given set of inputs
double optsystem_evaluate_distance_for_cost (optsystem_t *self, GSList *inputs) {

    // Calculates the Euclidean distance to get from state_from to state_to using the intermediate trajectory traj

    double time = 0.0;

    GSList *inputs_ptr = inputs;
    while (inputs_ptr) {
        input_t *input_curr = inputs_ptr->data;
        time += input_curr->x[0];
        inputs_ptr = g_slist_next (inputs_ptr);
    }
    
    return time;
}


// Checks whether the line segment between (state_initial) and (state_final) lies on an obstacle
int optsystem_segment_on_obstacle (optsystem_t *self, state_t *state_initial, state_t *state_final, int num_steps) {

    GSList *obstacle_list_curr = self->obstacle_list;

    while (obstacle_list_curr) {
        region_2d_t *obstacle_region = (region_2d_t *) (obstacle_list_curr->data);
        double center[2] = {
            obstacle_region->center[0],
            obstacle_region->center[1]
        };
        double size[2] = {
            obstacle_region->size[0]/2.0,
            obstacle_region->size[1]/2.0
        };

        double disc[2] = {
            (state_final->x[0] - state_initial->x[0])/( (double)num_steps ),
            (state_final->x[1] - state_initial->x[1])/( (double)num_steps )
        };

        double state_curr[2] = {
            state_initial->x[0],
            state_initial->x[1]
        };

        for (int i = 0; i < num_steps; i++) {
            if ( (fabs(center[0] - state_curr[0]) <= size[0]) && (fabs(center[1] - state_curr[1]) <= size[1]) ) 
                return 1;
            
            state_curr[0] += disc[0];
            state_curr[1] += disc[1];
        }
        
        obstacle_list_curr = g_slist_next (obstacle_list_curr);
    }
    
    return 0;
}


// Extends a given state towards another state
int optsystem_extend_to (optsystem_t *self, state_t *state_from, state_t *state_towards, 
                         int *fully_extends, GSList **trajectory,
                         int *num_node_states, int **nodes_states, GSList **inputs) {

    int discretization_num_steps = 10;
    
    GSList *trajectory_curr = NULL; // Start with an empty trajectory
    GSList *inputs_curr = NULL;

    double dist_x = state_towards->x[0] - state_from->x[0];
    double dist_y = state_towards->x[1] - state_from->x[1];

    double dist = sqrt (dist_x * dist_x + dist_y * dist_y);

    if (dist < 1.0) {
        if (optsystem_segment_on_obstacle (self, state_from, state_towards, discretization_num_steps) ) {
            *fully_extends = 0;
            return 0;
        }
        state_t *state_new = optsystem_new_state (self);
        state_new->x[0] = state_towards->x[0];
        state_new->x[1] = state_towards->x[1];
        trajectory_curr = g_slist_prepend (trajectory_curr, state_new);        
        input_t *input_new = optsystem_new_input (self);
        input_new->x[0] = dist;
        inputs_curr = g_slist_prepend (inputs_curr, input_new);
        *fully_extends = 1;
    }
    else { 
        *fully_extends = 0;
        state_t *state_new = optsystem_new_state (self);  
        state_new->x[0] = (state_towards->x[0] - state_from->x[0])/dist + state_from->x[0];
        state_new->x[1] = (state_towards->x[1] - state_from->x[1])/dist + state_from->x[1];
        if (optsystem_segment_on_obstacle(self, state_from, state_new, discretization_num_steps)) {
            optsystem_free_state (self, state_new);
            return 0;
        }
        trajectory_curr = g_slist_prepend (trajectory_curr, state_new);        
        input_t *input_new = optsystem_new_input (self);
        input_new->x[0] = 1.0;
        inputs_curr = g_slist_prepend (inputs_curr, input_new);
    }

    *trajectory = trajectory_curr;
    *inputs = inputs_curr;
    *num_node_states = 0;
    *nodes_states = NULL;
    
    return 1;
}


// Checks whether a given state is on an obstacle
gboolean optsystem_on_obstacle (optsystem_t *self, state_t *state) {

    GSList *obstacle_list_curr = self->obstacle_list;

    while (obstacle_list_curr) {
        region_2d_t *obstacle_region = (region_2d_t *) (obstacle_list_curr->data);

        if ( (fabs(obstacle_region->center[0] - state->x[0]) <= obstacle_region->size[0]/2.0) &&
             (fabs(obstacle_region->center[1] - state->x[1]) <= obstacle_region->size[1]/2.0) ) 
            return 1;

        obstacle_list_curr = g_slist_next (obstacle_list_curr);
    }
    
    return 0;
}


// Checks whether a given state is inside the target region
gboolean optsystem_is_reaching_target (optsystem_t *self, state_t *state) {
    
    if ( (fabs(self->goal_region.center[0] - state->x[0]) <= self->goal_region.size[0]/2.0) &&
         (fabs(self->goal_region.center[1] - state->x[1]) <= self->goal_region.size[1]/2.0) )  {
        return 1;
    }

    return 0;
}


double optsystem_evaluate_cost_to_go (optsystem_t *self, state_t *state) {

    // If state is in the goal region then return zero
    if (optsystem_is_reaching_target (self, state)) 
        return 0.0;

    // Otherwise calculate a lower bound on the cost to go 
    // TODO: do the exact cost to go calculation

    double min_side = self->goal_region.size[0]/2.0;
    if (self->goal_region.size[1] < min_side)
        min_side = self->goal_region.size[1]/2.0;

    double dist_x = state->x[0] - self->goal_region.center[0];
    double dist_y = state->x[1] - self->goal_region.center[1];


    dist_x = fabs(dist_x);
    dist_y = fabs(dist_y);
    
    double dist = sqrt(dist_x*dist_x + dist_y*dist_y);
    if (dist_x < dist_y) {
        dist -= sqrt(1 + (dist_x/dist_y)*(dist_x/dist_y))*self->goal_region.size[0]/2.0;
    }
    else {
        dist -= sqrt(1 + (dist_y/dist_x)*(dist_y/dist_x))*self->goal_region.size[0]/2.0;
    }

    dist-= 0.5;
    if (dist < 0.0)
        dist = 0.0;

    return dist;
    
    
    double sat[4] = { 0, 0, 0, 0};

    if (state->x[0] <= self->goal_region.center[0] + self->goal_region.size[0]/2.0)
        sat[0] = 1;

    if (state->x[0] >= self->goal_region.center[0] - self->goal_region.size[0]/2.0)
        sat[1] = 1;

    if (state->x[1] <= self->goal_region.center[1] + self->goal_region.size[1]/2.0)
        sat[2] = 1;

    if (state->x[1] >= self->goal_region.center[1] - self->goal_region.size[1]/2.0)
        sat[3] = 1;

    int sum = sat[0] + sat[1] + sat[2] + sat[3];
    
    if (sum == 2) { // Check distance to corners
        double dist_min;
        double dist_this;
        double dist_x;
        double dist_y;

        dist_x = state->x[0] - (self->goal_region.center[0] + self->goal_region.size[0]/2.0);
        dist_y = state->x[1] - (self->goal_region.center[1] + self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        dist_min = dist_this;

        dist_x = state->x[0] - (self->goal_region.center[0] - self->goal_region.size[0]/2.0);
        dist_y = state->x[1] - (self->goal_region.center[1] + self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        if (dist_this < dist_min)
            dist_min = dist_this;

        dist_x = state->x[0] - (self->goal_region.center[0] - self->goal_region.size[0]/2.0);
        dist_y = state->x[1] - (self->goal_region.center[1] - self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        if (dist_this < dist_min)
            dist_min = dist_this;
        
        dist_x = state->x[0] - (self->goal_region.center[0] + self->goal_region.size[0]/2.0);
        dist_y = state->x[1] - (self->goal_region.center[1] - self->goal_region.size[1]/2.0);
        dist_this = sqrt(dist_x*dist_x + dist_y*dist_y);
        if (dist_this < dist_min)
            dist_min = dist_this;
        
        return dist_min;
    }
    else if (sum == 3) { // Check distance to edges
        
        if (sat[0] == 0)
            return fabs(state->x[0] - (self->goal_region.center[0] + self->goal_region.size[0]/2.0));

        if (sat[1] == 0)
            return fabs(state->x[0] - (self->goal_region.center[0] - self->goal_region.size[0]/2.0));
        
        if (sat[2] == 0)
            return fabs(state->x[1] - (self->goal_region.center[1] + self->goal_region.size[1]/2.0));

        if (sat[3] == 0)
            return fabs(state->x[1] - (self->goal_region.center[1] - self->goal_region.size[1]/2.0));
        
        printf ("??Sum : %d\n", sum);
        exit(1);
   
    }
    else {
        printf ("Sum : %d\n", sum);
        exit(1);
    }

}


gboolean optsystem_update_goal_region (optsystem_t *self, region_2d_t *goal_region) {

    self->goal_region.center[0] = goal_region->center[0];
    self->goal_region.center[1] = goal_region->center[1];
    self->goal_region.size[0] = goal_region->size[0];
    self->goal_region.size[1] = goal_region->size[1];

    return TRUE;
}


gboolean optsystem_update_operating_region (optsystem_t *self, region_2d_t *operating_region) {

    self->operating_region.center[0] = operating_region->center[0];
    self->operating_region.center[1] = operating_region->center[1];
    self->operating_region.size[0] = operating_region->size[0];
    self->operating_region.size[1] = operating_region->size[1];

    return TRUE;
}


gboolean optsystem_update_obstacles (optsystem_t *self, GSList *obstacle_list) {

    // Clear the previous obstacles
    while (self->obstacle_list) {
        region_2d_t *region_curr = (region_2d_t *) (self->obstacle_list->data);
        self->obstacle_list = g_slist_remove (self->obstacle_list, region_curr);
        free (region_curr);
    }
    
    // Add new obstacles
    GSList *obstacle_list_curr = obstacle_list;
    while (obstacle_list_curr) {
        region_2d_t *region_curr = (region_2d_t *) (obstacle_list_curr->data);
        region_2d_t *region_new = (region_2d_t *) malloc (sizeof (region_2d_t));
        region_new->center[0] = region_curr->center[0];
        region_new->center[1] = region_curr->center[1];
        region_new->size[0] = region_curr->size[0];
        region_new->size[1] = region_curr->size[1];

        self->obstacle_list = g_slist_prepend (self->obstacle_list, (gpointer)region_new);

        obstacle_list_curr = g_slist_next (obstacle_list_curr);
    }

    return TRUE;
}
