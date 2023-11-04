#!/usr/bin/env python3

import gurobipy as g
import sys
import os
import itertools
import matplotlib.pyplot as plt
import numpy as np
#instance_path = "./hw1_public_instances"
#file_path = os.path.join(instance_path,"/instances","instance1.txt")

#parse input

with open(sys.argv[1],'r') as file:
    d = list(map(int,(file.read().split(sep=" ")))) 

#gurobi inicitalization
env = g.Env()
env.setParam('LogToConsole',0)
env.setParam('OutputFlag',0)
model = g.Model(env=env) 

#gurobi adding vars

x = model.addVars(len(d),lb=0,ub=g.GRB.INFINITY,vtype=g.GRB.INTEGER,name="x")
z = model.addVars(len(d),lb=0,ub=g.GRB.INFINITY,vtype=g.GRB.INTEGER,name="z") #should be rather GRB.CONTINOUS
#gurobi constraints


#model.addConstrs([g.quicksum([x[i] for i in range((j-8)%24,j)]) for j in range(24)])
for j in range(len(d)):
    #print("Hour {}".format(j),end="\t")
    if ((j-7) // len(d) == -1): #overflow
        #print([*itertools.chain(range((j-7) % len(d),len(d)),range(j+1))])
        model.addConstr( g.quicksum([ x[i] for i in  itertools.chain(range((j-7) % len(d),len(d)),range(j+1)) ] ) - d[j] <= z[j],name = "hour_{}_pos".format(j))
        model.addConstr(d[j] - g.quicksum([ x[i] for i in  itertools.chain(range((j-7) % len(d),len(d)),range(j+1)) ] )  <= z[j],name = "hour_{}_neg".format(j))
    
    else:
        #print([*range((j-7),j+1)])
        model.addConstr( g.quicksum([ x[i] for i in range((j-7),j+1) ] ) - d[j] <= z[j], name = "hour_{}_pos".format(j) )
        model.addConstr( d[j] - g.quicksum([ x[i] for i in range((j-7),j+1) ] ) <= z[j], name = "hour_{}_neg".format(j) )

model.setObjective(g.quicksum(z),g.GRB.MINIMIZE)

model.optimize()

if model.Status == g.GRB.OPTIMAL:
    with open(sys.argv[2],'w') as f:
            
        obj = model.getObjective()
        print("{}".format(int(obj.getValue())),end="\n",file=f)
        
        list_x = [str(int(items.x)) for (ind,items) in x.items()]
        print(" ".join(list_x),end="",file=f)
