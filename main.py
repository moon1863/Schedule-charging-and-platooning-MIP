#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 21:45:40 2022

@author: rakibulalam
"""

# define timer
import time
tic = time.perf_counter()

# uncomment argument values when above argument parser is uncommented    
routeLength = 595#args.l # Mile
numberOfCS =16#args.cs
csID = range(1,numberOfCS+1)
csIDwithO = range(0, numberOfCS+1)
csIDwithOD = range(0, numberOfCS+2) 
vehicleID = range(0,20)#args.v)
platoonID = range(0,20)#args.p)
TS = list(range(1,49)) #timestep

#d = dict() # distance between station
#for i in csIDwithOD:
#    for j in csIDwithOD:
#        if i==j:
#            d[i,j] = 0
#        else:
#            d[i,j] = abs((routeLength/(numberOfCS+1))*(i-j))

d = dict()#distance between stations including OD


distance_data=[51.57,62.19,22.74,59.36,78.32,21.12,43.30,35.53,24.80,5.53,29.61,6.77,7.31,9.73,4.49,86.49,46.14] #total=595 miles

for i in csIDwithOD:
    for j in csIDwithOD:
        if (j,i) in d.keys():
            d[i,j] = d[j,i]
        else:
            d[i,j] = sum(distance_data[x] for x in range(i,j))
#
# Model
from docplex.mp.model import Model
model = Model(name="fev_Scheduling")

# Sets
S = list(csID)
J = list(vehicleID)#,doc="set of vehicles")
I = list(platoonID)#,doc="set of platoons")
swithO = list(csIDwithO)#,doc="s union O") 
swithOD = list(csIDwithOD)#,doc="s union OD") 

# Variables
y = model.binary_var_cube(I, J, swithO,name="y")
t_jdep = model.continuous_var_matrix(J, swithOD, lb=0, name="t_jdep")
t_jarr = model.continuous_var_matrix(J, swithOD, lb=0, name="t_jarr") # doc = "time of vehicle j arrive at station s")
t_idep = model.continuous_var_matrix(I, swithOD, lb=0, name="t_idep")#doc = "time of platoon i depart from station s") #EMPTY
t_iarr = model.continuous_var_matrix(I, swithOD, lb=0, name="t_iarr")#doc = "time of platoon i arrive at station s") #EMPTY
soc_arr = model.continuous_var_matrix(J,swithOD, lb=0, name="soc_arr")# doc = "soc of vehicle j at station s when arrive")
soc_dep = model.continuous_var_matrix(J,swithOD, lb=0, name="soc_dep")# doc = "soc of vehicle j at station s when departs")
t_cha = model.continuous_var_matrix(J,S, lb=0, name="t_cha")#doc = "charging time of vehicle j at station s")
t_wai = model.continuous_var_matrix(J,S, lb=0, name="t_wai")#doc = "waiting time (after charge) of vehicle j at station s")
dummy = model.continuous_var_list(J, lb=0, name="dummy")#doc = "dummy var for delay cost calc")
w = model.binary_var_matrix(I,swithO, name="w") #doc = "indicator variables")
c1_jstau = model.binary_var_cube(J,S,TS, name="c1_jstau") #indicator varible sub1 for charging capacity formulation
c2_jstau = model.binary_var_cube(J,S,TS, name="c2_jstau") #indicator varible sub2 for charging capacity formulation
c_jstau = model.binary_var_cube(J,S,TS, name="c_jstau") #indicator varible for charging capacity formulation


# Parameters
M=25
N_max = len(vehicleID)#doc = "maximum vehicle in a platoon")
t_tau=dict()
n_s={1:8,2:4,3:8,4:4,5:12,6:6,7:4,8:8,9:1,10:8,11:8,12:6,13:2,14:1,15:1,16:1}

for tau in TS:
    t_tau[tau]=tau*30/60 # timestep*15 minute/60 ==> fractional hr

m=dict()
v_j=dict()
soc_depmin=dict()
soc_arrmin=dict()
soc_min=dict()
soc_max=dict()
t_arrmax=dict()
t_depmin=dict()
c_delay=dict()
c_energy=dict()
b=dict()
t_dep=dict()

diffArrTime=dict()
for j in J:
    diffArrTime[j]=17#diffDepTime[i]+11.16 
    
# eCascadia: 475kwh, 1.9kwh/mile  https://www.autoweek.com/news/green-cars/a36506185/electric-big-rig-semi-trucks/
# eM2: 315kwh, 1.4kwh/mile 
# MT50e: 220kwh, 1.76kwh/mile
efficiencyVehicle = 1.5#{k:random.uniform(1.4,1.9) for k in vehicleID}
batteryCapVehicle = 300#{k:random.randrange(220,400) for k in vehicleID}

for j in J:
    v_j[j] = 50
    soc_depmin[j]=1
    soc_arrmin[j]=0.2
    soc_min[j]=0.2
    soc_max[j]=1
    t_arrmax[j]=diffArrTime[j]
    t_depmin[j]=0
    c_delay[j]=50
    c_energy[j]=.1165
    b[j]=batteryCapVehicle
    #t_dep[j]=diffDepTime
    m[j]=efficiencyVehicle
       
v_i=dict()
for i in I:
    v_i[i] = 50

r_cha=dict()
c_cha=dict()
 
for s in S:
    r_cha[s]=100
    c_cha[s]=.28
  
delta=dict()
for i in I:
    for swithOele in swithO:
        delta[i,swithOele]=.13

# Constraints
# SOC 
for j in J:
    model.add_constraint(soc_dep[j,0]==soc_depmin[j]) #doc = "initial SOC: init depart SOC = depat soc from origin"
    model.add_constraint(soc_arr[j, swithOD[-1]]>=soc_arrmin[j]) #lastArrivalSOC

for j in J:
    for s in S:
        model.add_constraint(soc_dep[j,s] == soc_arr[j,s] + (r_cha[s]/b[j])*t_cha[j,s]) #departSOCfromCS
        model.add_constraint(soc_arr[j,s] >= soc_min[j]) #arriveSOCMin
        model.add_constraint(soc_dep[j,s] <= soc_max[j]) #departSOCMax

for j in J:
    for s in swithO:
        model.add_constraint(soc_arr[j,s+1] == soc_dep[j,s]- ((d[s,s+1]*m[j])/b[j])*(1 - sum(delta[i,s]*y[i,j,s] for i in I))) #arriveSOCatCS

# Time
for j in J:
    model.add_constraint(t_jdep[j,0] >= t_depmin[j]) #initialDepartTime
    #return model.t_jdep[j,0] == model.t_dep[j]

for j in J:
    for s in S:
        model.add_constraint(t_jdep[j,s] == t_jarr[j,s] + t_cha[j,s] + t_wai[j,s]) #departTime

for j in J:
    for s in swithO:
        model.add_constraint(t_jarr[j,s+1] == t_jdep[j,s] + d[s,s+1]/v_j[j]) #arriveTime
for i in I:
    for s in S:
        model.add_constraint(t_idep[i,s] == t_iarr[i,s]) #platoonDepart
        
for i in I:
    for s in swithO:
        model.add_constraint(t_iarr[i,s+1] == t_idep[i,s] + d[s,s+1]/v_i[i]) #platoonArriveTime
        model.add_constraint(sum(y[i,j,s] for j in J) >= w[i,s]*2)

for i in I:
    for j in J:
        for s in swithO:
            model.add_constraint(-M*(1-y[i,j,s]) <= t_jdep[j,s] - t_idep[i,s])  #platoonVehicleDep1
            model.add_constraint(t_jdep[j,s] - t_idep[i,s] <= M*(1-y[i,j,s])) #platoonVehicleDep2
            # Energy
            model.add_constraint(sum(y[i,j,s] for i in I) <= 1) #joinPlatoon1
            # reduntant: sum (y[i,j,s] for j in model.J) >= 0 #joinPlatoon2
            model.add_constraint(sum(y[i,j,s] for j in J) <= w[i,s]*N_max) #platoonNoLimit1

for j in J:
    model.add_constraint(dummy[j] >= t_jarr[j,swithOD[-1]]-t_arrmax[j]) #nonNegativeDelay1

for j in J:
    for s in S:
        for tau in TS:
            model.add_constraint(t_tau[tau]-t_jarr[j,s]>=(-M)*(1-c1_jstau[j,s,tau]))
            model.add_constraint(t_tau[tau]-t_jarr[j,s]<=M*c1_jstau[j,s,tau])
            model.add_constraint(t_jarr[j,s]+t_cha[j,s]-t_tau[tau]>=(-M)*(1-c2_jstau[j,s,tau]))
            model.add_constraint(t_jarr[j,s]+t_cha[j,s]-t_tau[tau]<=M*c2_jstau[j,s,tau])
            model.add_constraint(c1_jstau[j,s,tau]+c2_jstau[j,s,tau]-1.5>=(-M)*(1-c_jstau[j,s,tau]))
            model.add_constraint(c1_jstau[j,s,tau]+c2_jstau[j,s,tau]-1.5<=M*c_jstau[j,s,tau])


for s in S:
    for tau in TS:
        model.add_constraint(sum(c_jstau[j,s,tau] for j in J)<=n_s[s])


# Objective
enRtChargingCost = sum(c_cha[s]*r_cha[s]*t_cha[j,s] for j in J for s in S)
deliveryDelayCost = sum(c_delay[j]*dummy[j] for j in J)
hubChargingCost  = sum(c_energy[j]*d[s,s+1]*m[j]*(1-sum(delta[i,s]*y[i,j,s] for i in I)) for j in J for s in swithO)\
    -sum(c_energy[j]*r_cha[s]*t_cha[j,s] for j in J for s in S)
totalCost=enRtChargingCost+deliveryDelayCost+hubChargingCost
model.add_kpi(enRtChargingCost, 'enRtChargingCost')
model.add_kpi(deliveryDelayCost, 'deliveryDelayCost')
model.add_kpi(hubChargingCost, 'hubChargingCost')
model.add_kpi(totalCost, 'totalCost')

# finally the objective
model.minimize(totalCost)

model.parameters.mip.strategy.search=1
model.parameters.mip.tolerances.mipgap=0.01
# =============================================================================
# model.parameters.mip.tolerances.mipgap=0.05
# 
# model.parameters.mip.strategy.heuristicfreq=-1
# model.parameters.mip.limits.cutpasses=-1
# =============================================================================


#warm start
#values = { y[i,j,s]:0 for i in I for j in J for s in swithO}#(0  if (i==1) else 0) for i in r}
#warmstart=model.new_solution()
#warmstart.update(values)
#model.add_mip_start(warmstart)


modelSol=model.solve(log_output=True)
assert modelSol

model.report()

#tuning 
#cpx=model.get_engine().get_cplex()
#
#cpx.parameters.timelimit = 1200
#cpx.parameters.tune.timelimit.set(300.0)
#status=cpx.parameters.tune_problem()
#
#if status==cpx.parameters.tuning_status.completed:
# print("tuned parameters:")
# for param,value in cpx.parameters.get_changed():
#     print("{0}:{1}".format(repr(param), value))
#else:
# print("tuning status  was : {0}".format(cpx.parameters.tuning_status[status]))
#
#model.print_solution()

col_vid=[]
col_jp_vid=[]
col_jp_plid=[]
col_tvp_plid=[]
col_csid=[]
col_jp_csid=[]
col_tvp_csid=[]
col_depTime=[]
col_arrTime=[]
col_chaTime=[]
col_joinPlatoon=[]
col_totVehPlatoon=[]
for j in J:
    for s in S:
        col_vid.append(j)
        col_csid.append(s)
        col_depTime.append(modelSol[t_jdep[j,s]])
        col_arrTime.append(modelSol[t_jarr[j,s]])
        col_chaTime.append(modelSol[t_cha[j,s]])

for i in I:
    for j in J:
        for s in swithO:
            col_jp_plid.append(i)
            col_jp_vid.append(j)
            col_jp_csid.append(s)
            col_joinPlatoon.append(modelSol[y[i,j,s]])

for i in I:
    for s in swithO:
        col_tvp_plid.append(i)
        col_tvp_csid.append(s)
        col_totVehPlatoon.append(sum(modelSol[y[i,j,s]] for j in J))
        
with open("chargingTimeSA_capa_10vehgap5.csv","w")as file:
    file.write("j"+"\t"+"s"+"\t"+"t_jdep"+"\t"+"t_jarr"+"\t"+"t_cha"+"\n")
    for elem1,elem2,elem3,elem4,elem5 in zip(col_vid,col_csid,col_depTime,col_arrTime,col_chaTime):
        file.write(str(elem1)+"\t"+str(elem2)+"\t"+str(elem3)+"\t"+str(elem4)+"\t"+str(elem5)+"\n")

with open("joinPlatoonSA_capa_10vehgap5.csv","w")as file:
    file.write("i"+"\t"+"j"+"\t"+"s"+"\t"+"y"+"\n")
    for elem1,elem2,elem3,elem4 in zip(col_jp_plid,col_jp_vid,col_jp_csid,col_joinPlatoon):
        file.write(str(elem1)+"\t"+str(elem2)+"\t"+str(elem3)+"\t"+str(elem4)+"\n")

with open("totalVehicleInPlatoonSA_capa_10vehgap5.csv","w")as file:
    file.write("i"+"\t"+"s"+"\t"+"n"+"\n")
    for elem1,elem2,elem3 in zip(col_tvp_plid,col_tvp_csid,col_totVehPlatoon):
        file.write(str(elem1)+"\t"+str(elem2)+"\t"+str(elem3)+"\n")
##
