#!/usr/bin/env python3
import sys
import os
import itertools
import numpy as np
import copy

def transform_graph_to_feasible_flow_with_balances(edges,size_of_graph):
    # transform graph with positive lower bound to zero lower bound with balances
    # input: list of edges: [ ... ,[start,end,lower,upper] , ... ]
    # output: b(v), list of edges
    node_balances = [0]*size_of_graph

    new_edges = copy.deepcopy(edges)
    
    # add edge from t to s
    new_edges.append([size_of_graph-1,0,0,float('inf')])

    for edge in new_edges:
        # edge [start,end,lower,upper]
        l = edge[2]
        
        # set balance
        node_balances[edge[0]] -= l
        node_balances[edge[1]] += l
        
        # lower upper bound
        edge[3] -= l
        # set lower bound to zero
        edge[2] = 0

    return new_edges, node_balances

def transform_feasible_flow_with_balances_to_MaxFlow_with_zero_balances(edges,balance):
    
    number_of_nodes = len(balance)
    #create new nodes
    _s = number_of_nodes
    _t = number_of_nodes + 1

    new_edges = copy.deepcopy(edges)

    for i in range(number_of_nodes):
        
        if balance[i] > 0: # send flow from s into ith node
            new_edges.append([_s,i,0,balance[i]])
        
        elif balance[i] < 0: #  send flow from ith node to t
            new_edges.append([i,_t,0,-balance[i]])

    return new_edges, number_of_nodes + 2


def Ford_Fulkerson_algotihm(edge_list,size_of_graph,init_flow,s,t):
    
    # create Graph
    flow = copy.deepcopy(init_flow)
    fwd_adjacency_list = [{} for i in range(size_of_graph)] 
    bwd_adjacency_list = [{} for i in range(size_of_graph)] 

    for i in range(len(edge_list)):
        # edge [start,end,lower,upper]
        #print("Updating {} edge: {}".format(i,edge_list[i]))
        edge = edge_list[i]
        start = edge[0]
        end = edge[1]
        fwd_adjacency_list[start].update({end : i})
        bwd_adjacency_list[end].update({start: i})
        
    '''
    for i in range(size_of_graph):
        print("Node: {} has following fwd connections\n {}".format(i,fwd_adjacency_list[i]))
        print("Node: {} has following bwd connections\n {}".format(i,bwd_adjacency_list[i]))
    '''

    #label Path
    while True:
        # BFS
        P, gamma = BFS(fwd_adjacency_list,bwd_adjacency_list,edge_list,flow,s,t)
        
        if P:
            for p in P:
                edge_id = p[0]
                heading = p[1] # {-1,1}
            
                flow[edge_id] += heading*gamma
        else:
            break

    #print("Found maximal flow:",flow)

    return flow



def BFS(fwd_adjacency_list,bwd_adjacency_list,edge_list,flow,s,t):
    # s start node
    # t end node
    #print("Trying to find path from {} to {}".format(s,t))
    size_of_graph = len(fwd_adjacency_list)
    predecestor = [-2]*size_of_graph
    edge_pred = [-1]*size_of_graph
    capacity = [0]*len(edge_list)
    
    queue = [s]
    predecestor[s] = -1
    capacity[s] = float('inf')
    path_found = False

    while queue:
        parent = queue.pop(0)
        #print("Parent: {}".format(parent))
        if (parent == t):
            path_found = True
            #print("Break!")
            break
        
        fwd_neighbours = fwd_adjacency_list[parent]
        
        if fwd_neighbours: #there is child 
            for child in fwd_neighbours:
                if predecestor[child] == -2: #not yet visited
                    # edge [start,end,lower,upper]
                    edge_id = fwd_neighbours[child]
                    edge = edge_list[edge_id] 
                    f = flow[edge_id]
                    cap = edge[3] - f
                    if cap > 0:
                        predecestor[child] = parent
                        edge_pred[child] = edge_id
                        capacity[child] = cap

                        queue.append(child)
                        #print("Appeding child: {}".format(child))

        bwd_neighbours = bwd_adjacency_list[parent]

        if bwd_neighbours: #there is child 
            for child in bwd_neighbours:
                if predecestor[child] == -2:
                    # edge [start,end,lower,upper]
                    edge_id = bwd_neighbours[child]
                    edge = edge_list[edge_id] 
                    f = flow[edge_id]
                    cap = f - edge[2]
                    if cap > 0:
                        predecestor[child] = parent
                        capacity[child] = cap
                        edge_pred[child] = edge_id 
                        
                        queue.append(child)
                        #print("Appeding child: {}".format(child))
                    
    
    #retrive path 
    if path_found:
        gamma = float('inf')
        node = t
        path = []
        while predecestor[node] != -1:
            edge_id = edge_pred[node]
            edge = edge_list[edge_id]
            if edge[0] == predecestor[node]:
                heading = 1
            elif edge[1] == predecestor[node]:
                heading = -1
            else:
                raise ValueError

            path.append([edge_id,heading])
            gamma = min(gamma,capacity[node])
            node = predecestor[node]
            
    else:
        path = []
        gamma = -1

    '''
    for p in path[::-1]:
        edge_id = p[0]
        edge = edge_list[edge_id]
        if p[1] > 0:
            print("{} -> {} ({}) ||".format(edge[0],edge[1],edge[3]-flow[edge_id]),end='\t')
        elif p[1] < 0:
            print("{} -> {} ({}) ||".format(edge[1],edge[0],flow[edge_id]-edge[2]),end='\t')
    '''  
    return path, gamma    




#parse input

with open(sys.argv[1],'r') as file:
    s = list(map(int,file.readline().strip().split(" ")))
    number_of_clients = s[0]
    number_of_products = s[1]

    # create list of edges
    # [from,to,lower,upper]
    # id 0                  s-node
    # id 1 - |C|            customer nodes
    # id |C|+1 - |C|+|P|    product nodes 
    # id |C| + |P| + 1      t-node
    edges = []
    for i in range(1,number_of_clients+1):
        s = list(map(int,file.readline().strip().split(" ")))
        # i: [l_i,u_i,p_i1,p_i2,p_ij]
        
        #from s-node to ith node: l_i, u_i 
        edges.append([0,i,s[0],s[1]])
        
        #from ith customer to jth product (|C| + j th node)
        for j in range(2,len(s)):
            edges.append([i,s[j]+number_of_clients,0,1])


    # from products to t-node
    s = list(map(int,file.readline().strip().split(" ")))

    for j in range(1,number_of_products+1):
        #from jth product (|C| + j th node) to t-node
        edges.append([number_of_clients+j,number_of_clients+number_of_products+1,s[j-1],float('inf')])

print("Original edges:")
print(edges)

new_edges,node_balances = transform_graph_to_feasible_flow_with_balances(edges,number_of_products+number_of_clients+2)
print("New edges")
print(new_edges)
print("------")
print(node_balances)

alter_edges,size_of_alter_G = transform_feasible_flow_with_balances_to_MaxFlow_with_zero_balances(new_edges,node_balances)
print("Alter edges")
print(alter_edges)
print("------")

#First with alternated graph:
alter_flow = Ford_Fulkerson_algotihm(alter_edges,size_of_alter_G,[0]*len(alter_edges),size_of_alter_G-2,size_of_alter_G-1)
#print(alter_flow)


is_flow_feasible = True

for i in range(len(new_edges),len(alter_edges)):
    
    if alter_edges[i][0] == size_of_alter_G-2: # initial node in alter graph 
       if (alter_flow[i] < alter_edges[i][3]): # not sufficient flow
           is_flow_feasible = False
           break


if is_flow_feasible:

    feasible_flow = [0]*len(edges)

    for i in range(len(edges)):
        feasible_flow[i] = edges[i][2] + alter_flow[i]

    max_flow = Ford_Fulkerson_algotihm(edges,number_of_products+number_of_clients+2,feasible_flow,0,number_of_products+number_of_clients+1)



#parse output
customer_product_list = [[] for i in range(number_of_clients)]
with open(sys.argv[2],'w') as f:
        
    if not is_flow_feasible:
        print(-1,end="",file=f)
    else:
        for i in range(len(edges)):
            e = edges[i]
            if 0 < e[0] and e[0] < number_of_clients+1 and max_flow[i] == 1:
                customer_product_list[e[0]-1].append(e[1]-number_of_clients)

        for i in range(number_of_clients):
            print(" ".join(list(map(str,sorted(customer_product_list[i])))),end="",file=f)
            if i < number_of_clients - 1:
                print("",file=f)

    
    #list_x = [str(int(items.x)) for (ind,items) in x.items()]
    
