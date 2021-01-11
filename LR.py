'''
Lagrangian Relaxation test
'''

import pandas as pd
import numpy as np
import math
import time
import matplotlib.pyplot as plt

g_node_list=[]
g_link_list=[]
g_demand_list=[]
g_node_Id_ListNum_dict={}
g_node_ListNum_Id_dict={}
g_link_Id_ListNum_dict={}
g_link_ListNum_Id_dict={}
g_link_FromTo_ListNum_dict={}

g_node_No=-1
g_link_No=-1
g_demand_No=-1
g_LR_IterationNum=40
time_budget=14
LR_multiplier=0
LR_multiplier_stepsize=1
LR_multiplier_list=[]
LR_ObjValue_list=[]

class Node():
    def __init__(self):
        self.node_id=0
        self.outgoing_node_list=[]
        self.incoming_node_list=[]

class Link():
    def __init__(self):
        self.link_id=0
        self.from_node_id=0
        self.to_node_id=0
        self.length=0
        self.lanes=0
        self.free_speed=0
        self.capacity=0
        self.cost=0
        self.com_cost=0

    def CompositCost(self):
        self.com_cost=self.cost+LR_multiplier*self.length

class Demand():
    def __init__(self):
        self.o_zone_id=0
        self.d_zone_id=0
        self.volume=0


def ReadData():
    print('I am reading data, take your time.')
    global g_node_No
    global g_link_No
    global g_demand_No


    with open('node.csv') as file_object:
        lines=file_object.readlines()

        for line in lines[1:]:
            line=line.strip().split(',')
            node=Node()

            try:
                node.node_id=int(line[1])
            except:
                print('Python can not read node.csv successfuly, please check!')
            else:
                g_node_No+=1
                g_node_list.append(node)
                g_node_Id_ListNum_dict[node.node_id]=g_node_No
                g_node_ListNum_Id_dict[g_node_No] = node.node_id

                if g_node_No % 100 ==0:
                    print('the number of nodes has been read is: {}'.format(g_node_No))


    with open('link.csv') as file_object:
        lines=file_object.readlines()
        for line in lines[1:]:
            line=line.strip().split(',')
            link=Link()

            try:
                link.link_id=int(line[1])
                link.from_node_id=int(line[3])
                link.to_node_id=int(line[4])
                link.length=float(line[6])
                link.lanes=int(line[7])
                link.free_speed=float(line[8])
                link.capacity=float(line[9])
                link.cost=float(line[15])
            except:
                print('Python can not read link.csv successfuly, please check!')
            else:
                g_link_No+=1
                g_link_list.append(link)
                g_link_Id_ListNum_dict[link.link_id] = g_link_No
                g_link_ListNum_Id_dict[g_link_No] = link.link_id
                g_link_FromTo_ListNum_dict[(link.from_node_id,link.to_node_id)]=g_link_No
                from_node_ListNumb=g_node_Id_ListNum_dict[link.from_node_id]
                to_node_ListNumb = g_node_Id_ListNum_dict[link.to_node_id]
                g_node_list[from_node_ListNumb].outgoing_node_list.append(link.to_node_id)
                g_node_list[to_node_ListNumb].incoming_node_list.append(link.from_node_id)


                if g_link_No % 100 ==0:
                    print('the number of links has been read is: {}'.format(g_link_No))

    with open('demand.csv') as file_object:
        lines=file_object.readlines()
        for line in lines[1:]:
            line=line.strip().split(',')
            demand=Demand()

            try:
                demand.o_zone_id=int(line[0])
                demand.d_zone_id=int(line[1])
                demand.volume=float(line[2])
            except:
                print('Python can not read demand.csv successfuly, please check!')
            else:
                g_demand_No += 1
                g_demand_list.append(demand)
                if g_demand_No%100==0:
                    print('the number of demand has been read is: {}'.format(g_demand_No))

def Dijkstra(origin):

    s_permanent=[origin]
    s_temporary=[]
    node_id_list=[]
    for node in g_node_list:
        s_temporary.append(node.node_id)

    s_temporary.remove(origin)

    distance=[9999]*(g_node_No+1)
    predecessor=[0]*(g_node_No+1)
    distance[g_node_Id_ListNum_dict[origin]]=0
    current_node=origin

    while len(s_permanent)<g_node_No+1:

        current_node_ListNum=g_node_Id_ListNum_dict[current_node]
        for to_node in g_node_list[current_node_ListNum].outgoing_node_list:
            to_node_ListNum=g_node_Id_ListNum_dict[to_node]

            link=(current_node,to_node)
            link_ListNum=g_link_FromTo_ListNum_dict[link]
            if distance[to_node_ListNum]>distance[current_node_ListNum]+g_link_list[link_ListNum].com_cost:
                distance[to_node_ListNum]=distance[current_node_ListNum]+g_link_list[link_ListNum].com_cost
                predecessor[to_node_ListNum]=current_node

        s_temporary_ListNum_list=[g_node_Id_ListNum_dict[i] for i in s_temporary]
        min_distance=9999
        for node_ListNum in s_temporary_ListNum_list:
            if distance[node_ListNum]<min_distance:
                min_distance=distance[node_ListNum]
                current_node=g_node_ListNum_Id_dict[node_ListNum]

        s_permanent.append(current_node)
        s_temporary.remove(current_node)

    return distance,predecessor

def LagangianRelaxation(origin,destination):

    global LR_multiplier
    destination_ListNum=g_node_Id_ListNum_dict[destination]
    i=1
    while i<=g_LR_IterationNum:

        for link in g_link_list:
            link.CompositCost()

        distance, predecessor=Dijkstra(origin)
        min_path_composit_cost=distance[destination_ListNum]

        path_list=[]
        path_list.insert(0,destination)
        pre_node=destination
        while pre_node!=origin:

            pre_node_ListNum=g_node_Id_ListNum_dict[pre_node]
            pre_node=predecessor[pre_node_ListNum]
            path_list.insert(0, pre_node)

        path_length=0
        for j in range(len(path_list)-1):
            link=(path_list[j],path_list[j+1])
            link_ListNum=g_link_FromTo_ListNum_dict[link]
            path_length+=g_link_list[link_ListNum].length

        LR_multiplier_list.append(LR_multiplier)
        LR_obj_value = min_path_composit_cost-LR_multiplier*time_budget

        LR_ObjValue_list.append(LR_obj_value)
        LR_multiplier_stepsize=1/i
        LR_multiplier=max([0,LR_multiplier+LR_multiplier_stepsize*(path_length-time_budget)])
        i+=1

    return LR_multiplier_list,LR_ObjValue_list,path_list

if __name__=='__main__':

    ReadData()
    LR_multiplier_list,LR_ObjValue_list,path_list=LagangianRelaxation(0,5)
    print('The optimal path from origin to destination is:')
    print(path_list)



    plt.figure()
    plt.plot(np.arange(1,g_LR_IterationNum+1),LR_multiplier_list)
    plt.xlabel('Iteration number')
    plt.ylabel('Lagrangian multiplier value')
    new_ticks=range(1,g_LR_IterationNum,3)
    plt.xticks(new_ticks)

    plt.figure()
    plt.plot(np.arange(1, g_LR_IterationNum + 1), LR_ObjValue_list)
    plt.xlabel('Iteration number')
    plt.ylabel('Lagrangian objective function value')
    new_ticks = range(1, g_LR_IterationNum, 3)
    plt.xticks(new_ticks)

    plt.show()
