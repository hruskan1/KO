#!/usr/bin/env python3

import gurobipy as g
import sys
import os
import itertools
import matplotlib.pyplot as plt
import numpy as np
from itertools import combinations
#instance_path = "./hw1_public_instances"
#file_path = os.path.join(instance_path,"/instances","instance1.txt")

def eliminate_subtour(model,where):
    # Heavily inspired by https://www.gurobi.com/documentation/9.1/examples/tsp_py.html

    if  where == g.GRB.Callback.MIPSOL:
        
        _vars = model.getVars()
        values = model.cbGetSolution(_vars)
        n = int(len(values)**(1/2))
        sol = np.reshape(np.array(values),(n,n))
            
        edges = g.tuplelist([(i,j) for i in range(n) for j in range(n) if sol[i,j] > 0.5])
        
        shortest_tour_nodes = find_shortest_cycle(edges,n)
        
        if len(shortest_tour_nodes) < n: #
            #add lazy constraint forbidding the cycle on those tour nodes
            t = shortest_tour_nodes
            shortest_tour_edges = g.tuplelist( ( t[k] ,t[ (k+1) % len(t) ]) for k in range(len(t)) )
            model.cbLazy(g.quicksum(_vars[i*n+j] for i,j in shortest_tour_edges) <= len(shortest_tour_nodes)-1 )


def find_shortest_cycle(edges,n):
    # Heavily inspired by https://www.gurobi.com/documentation/9.1/examples/tsp_py.html

    #each node should have exactly one out edge and one in edge, each one has to lie on cycle:
    unvisited = list(range(n))
    cycle = list(range(n+1)) #initial cycle (unfeasible)

    #find shortest one from one each
    while len(unvisited) > 0:
        current_cycle = []
        neighbours = unvisited #initialize to all possibilities
        while neighbours:
            current = neighbours[0]
            unvisited.remove(current)
            current_cycle.append(current)

            neighbours = [j for i,j in edges.select(current,"*") if j in unvisited]

        if len(current_cycle) < len(cycle):
            cycle = current_cycle
    
    return cycle


def calculate_stripe_distance(first,second):
    #Calculate distance rightmost column of first and leftmost column of second stripe (abs value)
    return np.sum(np.abs(first[-1,:,:]-second[0,:,:]))

def solveTsp(distances):
    #solve TSP (ensure loaded gurobi as g)
    #distances = numpy (n,n) matrix of distances between nodes
    #returns list of node indices forming the optimal circuit
    
    n = distances.shape[0]
    #gurobi inicitalization
    m = g.Model()
    m.Params.lazyConstraints = 1

    #gurobi adding vars (objective set)
    e = m.addVars(n,n,vtype=g.GRB.BINARY,name="e")

    #gurobi constraints (find edge cover (each vertex has to have one in degree and one outdegree))
    m.addConstrs(e.sum(i,"*") == 1 for i in range(n) )
    m.addConstrs(e.sum("*",i) == 1 for i in range(n) )

    #forbid trivial cycle
    m.addConstrs(e.sum(i,i) == 0 for i in range(n) )

    #set minimize objective 
    m.setObjective(g.quicksum( distances[i,j]*e[i,j] for i,j in itertools.product(range(n),range(n)) ),sense=g.GRB.MINIMIZE)
    
    #Optimize
    m.optimize(eliminate_subtour)

    #Retrive solution
    if m.Status == g.GRB.OPTIMAL:
        _vars = m.getVars()
        optimal_tour = []
        _from = 0
        while len(optimal_tour) < n:
            for j in range(n):
                if _vars[_from*n + j].x > 0.5:
                    optimal_tour.append(_from)
                    _from = j
                    break
        return optimal_tour
    else:
        return None



#parse input

with open(sys.argv[1],'r') as file:
    # n   number of stripes
    # w   width of stripe (number of columns in one stripe)
    # h   height of stripe (number of rows in one stripe)
    # each cell (pixel) has 3 rgb color value
    
    n,w,h = list(map(int,(file.readline().strip().split(sep=" ")))) 
    stripes = np.zeros((n,w,h,3),dtype=np.int)

    print("n: {} \t w: {}\t h: {}".format(n,w,h))
    raw_stripes = file.read().strip().split(sep="\n")
    
    
    for index,raw_stripe in enumerate(raw_stripes):
        stripe =  np.array(raw_stripe.split(sep=" "),dtype=np.int)
        reshaped_stripe =np.swapaxes(np.reshape(stripe,(h,w,3)),0,1)
        #print("reshaped stripe ({}.):\n {}".format(index+1,reshaped_stripe))
        stripes[index,:,:,:] =  reshaped_stripe

#Calculate distance matrix (full graph)
D  = np.zeros((n+1,n+1)) 
for i in range(n):
    for j in range(n):
        if i == j:
            D[i+1,j+1] = 0
        else:
            D[i+1,j+1] = calculate_stripe_distance(stripes[i,:,:,:],stripes[j,:,:,:])
print("Distance matrix:\n {}".format(D))

tour_list = solveTsp(D)
print(tour_list)
if tour_list:
    with open(sys.argv[2],'w') as f:
        print(" ".join(map(str,tour_list[1:])),end="",file=f)
