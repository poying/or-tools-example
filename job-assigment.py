# goal: minimize the total moving distance
# rules have to follow:
#   1. every da can only accept one job
#   2. every job has to be assigned to one da
#   3. check pracel size

import math
from ortools.sat.python import cp_model

def moving_distance(da, job):
    return distance(da["x"], da["y"], job["px"], job["py"]) + distance(job["px"], job["py"], job["dx"], job["dy"])

def distance(x1, y1, x2, y2):
    return math.floor(math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2)) * 1000)

model = cp_model.CpModel()

# data
jobs = [
    { "size": 10, "px": 1, "py": 10, "dx": 10, "dy": 1 },
    { "size": 13, "px": 4, "py": 1, "dx": 8, "dy": 11 },
]

das = [
    { "space": 15, "x": 2, "y": 10 },
    { "space": 10, "x": 11, "y": 11 },
    { "space": 20, "x": 1, "y": 11 },
]

# variables
result = []
for i in range(len(das)):
    da_result = []
    result.append(da_result)
    for j in range(len(jobs)):
        da_result.append(model.NewIntVar(0, 1, "result[%i,%i]" % (i, j)))

cost = []
for i in range(len(das)):
    da_cost = []
    cost.append(da_cost)
    for j in range(len(jobs)):
        da_cost.append(moving_distance(das[i], jobs[j]))

# constraint functions
# check parcel size
for i in range(len(das)):
    for j in range(len(jobs)):
        model.Add(result[i][j] * (das[i]["space"] - jobs[j]["size"]) >= 0)

# every da can only accept 1 job
for i in range(len(das)):
    model.Add(sum(result[i]) <= 1)

# every job has to be assigned to 1 da
for j in range(len(jobs)):
    model.Add(sum([ result[i][j] for i in range(len(das)) ]) == 1)

# objective function
# min result[1][1] * moving_distance(da[1], job[1]) + result[1][2] * moving_distance(da[1], job[2]) + ... + result[m][n] * moving_distance(da[m], job[n])
model.Minimize(sum([ result[i][j] * cost[i][j] for i in range(len(das)) for j in range(len(jobs)) ]))

# solve the problem
solver = cp_model.CpSolver()
status = solver.Solve(model)

if status == cp_model.OPTIMAL:
    print('Solution:')
    for i in range(len(das)):
        for j in range(len(jobs)):
            print("result[%i,%i] = %i" % (i, j, solver.Value(result[i][j])))
            # print(solver.Value(result[i][j]) * (das[i]["space"] - jobs[j]["size"] + 1))
else:
    print('The problem does not have an optimal solution.')