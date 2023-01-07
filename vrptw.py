from gurobipy import *
import csv
import matplotlib.pyplot as plt
import folium
from gurobipy import GRB,Model,quicksum,tupledict,tuplelist

"""
:param N: 所有节点集合,其中N[0]为车场
:param Q: 节点需求集合
:param TT: 旅行时间
:param ET: 节点最早开始服务时间
:param LT：节点最晚结束服务时间
:param ST: 节点服务时间
:param CAP: 车辆容量
:param Cost: 旅行费用
:param K: 车队数量 10
:return:
"""

N=[]  #所有节点
Q={}  #节点需求
TT={} #节点旅行时间
ET={} #节点最早开始服务时间
LT={} #节点最晚结束服务时间
ST={} #节点服务时间
Cost={}

with open('node_for_gurobi.csv','r') as f:
    node_reader = csv.DictReader(f)
    for row in node_reader:
        node_id = row['id']
        demand = float(row['demand'])
        start_time = float(row['start_time'])
        end_time = float(row['end_time'])
        service_time = float(row['service_time'])
        N.append(node_id)
        Q[node_id] = demand
        ET[node_id] = start_time
        LT[node_id] = end_time
        ST[node_id] = service_time
with open('link_for_gurobi.csv','r') as f:
    link_reader = csv.DictReader(f)
    for row in link_reader:
        link_node_id = row['from_node_id']
        to_node_id = row['to_node_id']        
        travel_time = float(row['travel_time'])
        travel_cost = float(row['link_cost'])
        TT[link_node_id,to_node_id] = travel_time
        Cost[link_node_id,to_node_id] = travel_cost

CAP = 80           #容量约束
K = 15             #车队数量
C=tuplelist(N[1:]) #需求节点
N=tuplelist(N)
Q=tupledict(Q)
TT=tupledict(TT)
E=tupledict(ET)
L=tupledict(LT)
S=tupledict(ST)
K=tuplelist([f'v'+str(i) for i in range(K)])
M=10**5
depot = N[0]

m = Model('cvrptw')

x = m.addVars(N,N,K, vtype=GRB.BINARY, name='X(i,j,k)')
t = m.addVars(N,K, vtype=GRB.CONTINUOUS, lb=0, name='T[i,k]')

#设置目标函数
z1 = quicksum(Cost[i,j]*x[i,j,k] for i in N for j in N for k in K if i!=j)
m.setObjective(z1, GRB.MINIMIZE)

#车辆起点约束:让车辆驶出仓库(depot)
m.addConstrs(quicksum(x[depot, j, k] for j in N) == 1 for k in K)

#车辆路径连续约束:流平衡(除去depot之外的点)
m.addConstrs(quicksum(x[i,j,k] for j in N if j!=i) == quicksum(x[j,i,k] for j in N if j!=i) for i in C for k in K)

#车辆终点约束:让车辆驶回depot
m.addConstrs(quicksum(x[j, depot, k] for j in N) == 1 for k in K)

#需求服务约束:保证每个客户点都被服务
m.addConstrs(quicksum(x[i,j,k] for k in K for j in N if j!=i) == 1 for i in C)

#车辆容量约束:不超过CAP=80
m.addConstrs(quicksum(Q[i]*x[i,j,k] for i in C for j in N if i!=j) <= CAP for k in K )

#时间窗约束:
#保证被服务的相邻节点开始服务时间的大小关系(去回路)
m.addConstrs(t[i,k]+S[i]+TT[i,j]-(1-x[i,j,k])*M <= t[j,k] for i in C for j in C for k in K if i!=j )
#不违反客户的时间窗
m.addConstrs(t[i,k] >= E[i] for i in N for k in K)
m.addConstrs(t[i,k] <= L[i] for i in N for k in K)

m.optimize()

def saveFile(data):
    outfile = open('resultstest.csv', 'w', newline='', errors='ignore')
    write = csv.writer(outfile)
    write.writerow(['from_node_id', 'to_node_id', 'vehicle','Ti','Tj'])
    for v in data:
        write.writerow(v)
    outfile.close()

if m.status == GRB.OPTIMAL:
    print('obj={}'.format(m.objVal))
    res=[]
    for k in K:
        for i in N:
            for j in N:
                if i!=j:
                    if x[i,j,k].x>0:
                        print("X[{},{},{}]=1".format(i,j,k))
                        res.append([i,j,k,t[i,k].x,t[j,k].x])
    saveFile(res)
else:
    print("no solution")