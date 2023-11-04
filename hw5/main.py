#!/usr/bin/env python3

import gurobipy as g
import sys
import os
import itertools
import matplotlib.pyplot as plt
import numpy as np
from itertools import combinations
import math
#instance_path = "./hw1_public_instances"
#file_path = os.path.join(instance_path,"/instances","instance1.txt")

class Task():
    
    def __init__(self,index,process_time,release_date,deadline) -> None:
        self.i = index
        self.p = process_time
        self.r = release_date
        self.d = deadline
        self.s = -1
        pass

    def __lt__(self,other):
        self.i < other.i

    def __str__(self) -> str:
        return "T{}".format(self.i)

    def __repr__(self) -> str:
        return "T{} ({}|{}|{}) (s:{})".format(self.i,self.p,self.r,self.d,self.s)

def print_Bratley_instance(scheduled,unscheduled,solution_exists,best_so_far,current_solution):
    print("Current instance (c = {}) with best value (b ={}) Solution exists? {}".format(current_solution,best_so_far,solution_exists))
    for t in scheduled:
        print(t,end="")
    print("",end="\t")
    for t in unscheduled:
        print(t,end="")
    print("")

    return None

def recursive_Bratley(scheduled,unscheduled,solution_exists,best_solution_so_far,current_solution):
    """
    @param scheduled    list of tasks which are already scheduled
    @param unscheduled  list of tasks to be yet scheduled
    @param solution_exists  bool whether there is actual solutin
    @best_solution_value    int Upper bound on solution value
    @current_solution       int 

    @return current_solution int ,scheduled_part_sequence list, can_backtrack bool
    """
    
    #print_Bratley_instance(scheduled,unscheduled,solution_exists,best_solution_so_far,current_solution)

    ##Zero case##

    #input() To debug easily
    
    if not unscheduled:
        print("Found solution!")
        return current_solution, scheduled, False
    
    c = current_solution
    min_release_time = math.inf
    min_total_time_duration = 0
    sub_upper_bound = 0
    should_not_backtrack = False
    
    ##Check bounding conditions ##
    
    
    for task in unscheduled:
        #Deadline condition (each unscheduled task)
        if task.p + max(c,task.r) > task.d:
            #print("Prune (1) {} missed deadline".format(task))
            return -1, scheduled, False

        min_release_time = min(min_release_time,task.r)
        min_total_time_duration += task.p
        sub_upper_bound = max(sub_upper_bound,task.d)
    
    #LB condition
    r_min = min_release_time
    if solution_exists:
        if max(c,r_min) + min_total_time_duration >= best_solution_so_far:
            #print("Prune (2) better solution exists".format(task))
            return -1, scheduled, False
    else:
        if max(c,r_min) + min_total_time_duration > best_solution_so_far:
            #print("Prune (2) better solution exists".format(task))
            return -1, scheduled, False


    #Do not backtrack condition - needs to be propagated above 
    if c > 0 and r_min >= c:
        print("Can restart Bratley alg!")
        sub_sol,sub_scheduled,_ = recursive_Bratley([],unscheduled,False,sub_upper_bound,0)
        return sub_sol, scheduled + sub_scheduled, True

    
    ## Follow breaching procedure ##

    
    scheduled_to_be_returned = []
    value_to_be_returned = 0
    for index,task in enumerate(unscheduled):
        newly_scheduled = scheduled + [task]
        newly_unscheduled = unscheduled[:index] + unscheduled[index+1:]
        new_sol_val = task.p + max(task.r,c)
        ret_value, ret_scheduled, can_backtrack = recursive_Bratley(newly_scheduled,newly_unscheduled,solution_exists,best_solution_so_far,new_sol_val)
        
        if can_backtrack:
            value_to_be_returned = ret_value
            scheduled_to_be_returned = ret_scheduled
            break

        # Feasible solution 
        if ret_value >= 0:     
            if ( (not solution_exists) and best_solution_so_far >= ret_value) or (solution_exists and best_solution_so_far > ret_value):
                solution_exists = True
                best_solution_so_far = ret_value
                scheduled_to_be_returned = ret_scheduled
                value_to_be_returned = ret_value
 
    
    # Feasible solution found
    if scheduled_to_be_returned:
        return value_to_be_returned,scheduled_to_be_returned, can_backtrack
    else:
        return -1, scheduled, can_backtrack

#parse input

with open(sys.argv[1],'r') as file:

    
    number_of_tasks = int(file.readline().strip())
    list_of_task =[]
    upper_bound = 0 
    print("Number of tasks: {}".format(number_of_tasks))
    for i in range(number_of_tasks):
        p,r,d = list(map(int,(file.readline().strip().split(sep=" ")))) 
        print("p: {} \t r: {}\t d: {}".format(p,r,d))
        list_of_task.append(Task(i+1,p,r,d))
        upper_bound = max(upper_bound,d) #total time of scheduled timetable must be below maximal deadline 
        

## Bratley algorithm
ret,scheduled_sequence,_ = recursive_Bratley([],list_of_task,False,upper_bound,0)
#print("Bratley algorithm found!  solution: {} ({})".format(scheduled_sequence,ret))

#Reconstruct actual time table
current_time = 0
if ret > 0:
    for task in scheduled_sequence:
        task.s = max(current_time,task.r)
        current_time = task.s + task.p

    
#print("Timetable reconstructed:")
#print(list_of_task)    


with open(sys.argv[2],'w') as f:
    print("ret: {}".format(ret))
    if ret < 0:
        print("-1",file=f)
    else:
        for task in list_of_task:
            print(task.s,file=f)
