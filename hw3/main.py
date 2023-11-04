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

def transform_feasible_flow_with_balances_to_flow_with_zero_balances(edges,balance):
    # transform graph with balances to zero lower bound 
    # input: list of edges: [ ... ,[start,end,lower,upper] , ... ]
    # input: b(v)
    # output new list of edges and size (two last nodes are start and end)
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

def Cycle_canceling_alg(edge_list,flow,weights,n):
    # edge list = [e1,e2,..], e1 = [from,to,lower,upper]
    # flow [f1,f2,..] f1 is current flow in f1
    # weight [w1,w2,..] w1 is weight of e1
    
    m = len(edge_list)
    #build a initial residual graph (only add new edges) (backward arc has "index of edge + m")
    #init structs
    residual_edge_list = [[-1,-1] for i in range(len(edge_list))]
    residual_cost_list = copy.deepcopy(weights)
    residual_improving_capacity_list = [0]*len(edge_list) 
    
    
    for i,e in enumerate(edge_list): 
        # e = [from,to,lower,upper]
        #print("{}.th edge: {}".format(i,e))
        
        #rewwrite forward edge & forward capacity (forward weights already copied)
        residual_edge_list[i][0] = e[0]
        residual_edge_list[i][1] = e[1]
        residual_improving_capacity_list[i] = e[3] - flow[i]

        #append backward edge & capacity and weights
        residual_edge_list.append([e[1],e[0]])
        residual_improving_capacity_list.append(flow[i]-e[2])
        residual_cost_list.append(-weights[i])
    
    '''
    print("Residual edge list:")
    print(residual_edge_list)
    print("Residual improving capacity:")
    print(residual_improving_capacity_list)
    print("Residual weight list:")
    print(residual_cost_list)
    '''
    
    #main loop
    while True:
        Cycle,delta = find_negative_cycle(residual_edge_list,residual_improving_capacity_list,residual_cost_list,n)

        if delta > 0:
            for edge_index in Cycle:
                edge = residual_edge_list[edge_index]
                if edge[0] < edge[1]: #forward arch
                
                    flow[edge_index] += delta
                    residual_improving_capacity_list[edge_index] -= delta #fwd arc
                    residual_improving_capacity_list[edge_index+m] += delta #bwd arc
                
                else:

                    flow[edge_index-m] -= delta
                    residual_improving_capacity_list[edge_index-m] += delta #fwd arc
                    residual_improving_capacity_list[edge_index]  -= delta #bwd arc
            
        
        #print("Edge in cycle: {}".format(edge))
        #print("Another edge: {}".format(residual_edge_list[edge_index-m]))            
        
        else:         
            break



    return flow

        

        

def find_negative_cycle(edge_list,capacity_list,weights,n):
    #return negative Cycle and minimal capacity 
    
    #create new graph
    index_list = []
    edges = []
    costs = []
    nodes = set()
    fwd_adjacency_list = [{} for i in range(n)] 

    #remove all edges with 0 capacity
    for i in range(len(edge_list)):
        edge = edge_list[i]
        if capacity_list[i] > 0:
            index_list.append(i)
            edges.append(edge)
            costs.append(weights[i])
            nodes.add(edge[0])
            nodes.add(edge[1])
            fwd_adjacency_list[edge[0]].update({edge[1] : i})
            
            #print("{}th edge (with positive cap): {}".format(i,edge_list[i]))
            
    
    #auxilary new node s_ with zero weight
    for node in nodes:
        edges.append([n,node])
        costs.append(0)

    '''
    print("Reduced Graph (size: {}):".format(len(nodes)))
    print(nodes)
    print("Edges (size: {}):".format(len(edges)))
    print(edges)
    print("Fwd adjacency list:")
    print(fwd_adjacency_list)
    print("Costs (size: {}): {}".format(len(costs),costs))
    '''

    pred, dist,is_affected = Bellman_Ford(edges,costs,n,n+1,len(nodes))
    
    '''
    print("Predecessors:")
    print(pred)
    print(dist)
    '''

    #detect negative cycle:
    visited = [False]*(n+1)
    stack = []

    is_there_negative_cycle = False
    for i in range(len(dist)):
        if (is_affected[i]):
            is_there_negative_cycle = True
            
            next_ = i
            break
    
    #end if no negative cycle
    if not is_there_negative_cycle:
        return [], -1 
    

    # if there is negative cycle, reconstruct one
    while True: 

        if visited[next_] == True:
            break

        visited[next_] = True
        stack.append(next_)
        next_ = pred[next_]
    
    #print("Stack: {} ({})".format(stack,next_))
    
    # parse cycle
    Cycle = []
    
    end_node = next_
    prev_ = stack.pop()
    Cycle.append(fwd_adjacency_list[next_][prev_])

    while prev_ != end_node:
        
        next_ = prev_
        prev_ = stack.pop()
        Cycle.append(fwd_adjacency_list[next_][prev_])

    #print("Is there key {} in: {}".format(prev_,fwd_adjacency_list[next_]))
    #print("Cycle: {}".format(Cycle))
    

    min_delta = np.inf
    cost_of_cycle = 0
    for e in Cycle:
        #print("{} w: ({:2.2f}) c: ({})".format(edge_list[e],weights[e],capacity_list[e]),end="\t")
        cost_of_cycle += weights[e]
        min_delta = min(min_delta,capacity_list[e])
    #print("\nCost of cycle: {}".format(cost_of_cycle))

    return Cycle,min_delta
        

def Bellman_Ford(edge_list,weights,s,n,n_):
    #edge_list [[u,v],..]
    #weights (coresponding to the edge list) [w,..] \in \R
    #initial node s
    # n -maximal id of node
    # n_ total number of nodes (with id larger then n)
    #returns predecesors and distances
    
    m = len(edge_list)

    #initilize mappings
    predecesors = np.ones(n,dtype=np.int)*(-1)
    distances = np.ones(n)* np.inf
    is_it_affected_by_negative_cycle = np.zeros(n,dtype=np.int)

    distances[s] = 0

    for i in range(n_-1):
        for j in range(m): 
            u,v = edge_list[j]
            if distances[u] + weights[j] < distances[v]:
                distances[v] = distances[u] + weights[j]
                predecesors[v] = u

    #pre-detect negative cycle
    for i in range(n_-1):
        for j in range(m):
            u,v = edge_list[j]
            if distances[u] + weights[j] < distances[v]:
                distances[v] = distances[u] + weights[j]
                predecesors[v] = u
                is_it_affected_by_negative_cycle[v] = 1

    return predecesors,distances,is_it_affected_by_negative_cycle


#parse input

with open(sys.argv[1],'r') as file:
    s = list(map(int,file.readline().strip().split(" ")))
    number_of_objects = s[0]
    number_of_frames = s[1]
    #print("Number of objects: {}\t Number of frames: {}".format(s[0],s[1]))
    
    pos_matrix = np.zeros((number_of_frames,number_of_objects,2))
    for i in range(number_of_frames):
        s = list(map(int,file.readline().strip().split(" ")))
        for j in range(number_of_objects):
            pos_matrix[i,j,0] = s[2*j] 
            pos_matrix[i,j,1] = s[2*j+1]

    mappings = np.ones((number_of_frames-1,number_of_objects),dtype=np.int) *(-1)

#print(pos_matrix)
for frame_index in range(number_of_frames-1):
    #create graph for two consecutive frames
    size_of_graph = 2*number_of_objects + 2
    # id 0      s-node
    # id 1 - |O|+1  ith object in first frame
    # id |0|+1 - 2|O|+1 jth object in following frame
    # id 2|O| + 2  t-node

    # create list of edges with weights corresponding to euclidian metric
    # [from,to,lower,upper]
    edges = []
    weights = []
    #from s node to first frame 
    for object_index in range(1,number_of_objects+1):
        edges.append([0,object_index,1,1])
        weights.append(0)
        #zero weight


    #from first frame to following frame
    for i in range(1,number_of_objects+1):
        for j in range(number_of_objects+1, 2*number_of_objects+1):
            edges.append([i,j,0,1])
            
            #print("Points:")
            #print(pos_matrix[frame_index,i-1,:])
            #print(pos_matrix[frame_index+1,j-number_of_objects-1,:])
            x = pos_matrix[frame_index,i-1,:]
            y = pos_matrix[frame_index+1,j-number_of_objects-1,:]
            d_ij = np.linalg.norm(x-y)
            #print("Calculated distance: {}".format(d_ij))  
            weights.append(d_ij)

    #from following frame to t node:

    for j in range(number_of_objects+1, 2*number_of_objects+1):
        edges.append([j,size_of_graph-1,1,1])
        weights.append(0)




    #find initial flow
    new_edges,node_balances = transform_graph_to_feasible_flow_with_balances(edges,size_of_graph)
    
    alter_edges,size_of_alter_G = transform_feasible_flow_with_balances_to_flow_with_zero_balances(new_edges,node_balances)

    alter_flow = Ford_Fulkerson_algotihm(alter_edges,size_of_alter_G,[0]*len(alter_edges),size_of_alter_G-2,size_of_alter_G-1)

    '''
    print("Original edges (len: {}) (n = {}):".format(len(edges),size_of_graph))
    print(edges)
    print("New edges (len: {}):".format(len(new_edges)))
    print(new_edges)
    print("------")
    print(node_balances)
    print("Alter edges (len: {}) (n = {}):".format(len(alter_edges),size_of_alter_G))
    print(alter_edges)
    print("------")
    '''


    is_flow_feasible = True

    for i in range(len(new_edges),len(alter_edges)):
        
        if alter_edges[i][0] == size_of_alter_G-2: # initial node in alter graph 
            if (alter_flow[i] < alter_edges[i][3]): # not sufficient flow
                is_flow_feasible = False
                break

    #Run cycle alg
    if is_flow_feasible:
        
        feasible_flow = [0]*len(edges)

        for i in range(len(edges)):
            feasible_flow[i] = edges[i][2] + alter_flow[i]

        '''
        print("Feasible flow!")
        print("Original edges (len: {}) (n = {}):".format(len(edges),size_of_graph))
        print(edges)
        print("Feasible flow:")
        print(feasible_flow)
        print("Weights:")
        print(weights)
        '''

        min_flow = Cycle_canceling_alg(edges,feasible_flow,weights,size_of_graph)
        
        #print("Min flow:", min_flow)

        #reconstruction of mapping
        
        for i in range(number_of_objects,len(edges)-number_of_objects):
            if min_flow[i] > 0.5:
                mappings[frame_index,edges[i][0]-1] = edges[i][1]-number_of_objects

    else:
        print("Not Feasible flow!")

#parse output
with open(sys.argv[2],'w') as f:
    for i in range(number_of_frames-1):
        row = mappings[i,:]
        print(" ".join(map(str,row.tolist())),end="",file=f)
        if (i != number_of_frames-2):
            print("",file=f)
    
