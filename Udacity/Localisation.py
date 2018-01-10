'''
#uniform distribution

p=[0.2,0.2,0.2,0.2,0.2]

#  Modify your code to create probability vectors, p, of arbitrary 
#  size, n. Use n=5 to verify that your new solution matches 
#  the previous one.

p = []
n = 5

for i in range(n):
    p.append(1./n) #to get fp

print p

#Write code that outputs p after multiplying each entry 
#by pHit or pMiss at the appropriate places. Remember that
#the red cells 1 and 2 are hits and the other green cells
#are misses.

p=[0.2,0.2,0.2,0.2,0.2]
pHit = 0.6
pMiss = 0.2
greenSquare = [0,3,4]
redSquare = [1,2]

for i in greenSquare:
    p[i] = p[i]*pMiss
for i in redSquare:
    p[i] = p[i]*pHit

print p

#Modify the program to find and print the sum of all 
#the entries in the list p.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
pHit = 0.6
pMiss = 0.2

p[0]=p[0]*pMiss
p[1]=p[1]*pHit
p[2]=p[2]*pHit
p[3]=p[3]*pMiss
p[4]=p[4]*pMiss

sum = 0
for i in range(5):
    sum += p[i]

print sum
'''

#sense function

#Modify the code below so that the function sense, which 
#takes p and Z as inputs, will output the NON-normalized 
#probability distribution, q, after multiplying the entries 
#in p by pHit or pMiss according to the color in the 
#corresponding cell in world.

#Modify your code so that it normalizes the output for 
#the function sense. This means that the entries in q 
#should sum to one.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
Z = 'red'
pHit = 0.6
pMiss = 0.2
q = []

def sense(p, Z):

    for i in range(5):
        if world[i] == 'green':
            p[i] = p[i]*pMiss
        else:
            p[i] = p[i]*pHit
        q.append(p[i])

    sum = 0
    for k in range(len(p)):
        sum += q[k]
    print sum #0.36
    
    for b in range(len(p)):
        q[b] = q[b]/sum
  
    return q

    for k in range(len(measurements)): #for multiple measurements
    p = sense(p,measurements[k])

print p

#EXACT MOTION

#Program a function that returns a new distribution 
#q, shifted to the right by U units. If U=0, q should 
#be the same as p.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
Z = 'red'
pHit = 0.6
pMiss = 0.2
q = []

def sense(p, Z):

    for i in range(5):
        if world[i] == 'green':
            p[i] = p[i]*pMiss
        else:
            p[i] = p[i]*pHit
        q.append(p[i])

    sum = 0
    for k in range(len(p)):
        sum += q[k]
    print sum #0.36
    
    for b in range(len(p)):
        q[b] = q[b]/sum
  
    return q

def move(p, U):
    t = []
    for i in range(len(p)):
        t.append(p[(i-U]%len(p))   
    return t
                 
print move(p, 1)

#INEXACT MOTION

#Modify the move function to accommodate the added 
#probabilities of overshooting or undershooting 
#the intended destination.

p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact*p[i-U%len(p)]
        s = s + pOvershoot*p[i-U+1%len(p)]
        s = s + pUndershoot*p[i-U-1%len(p)]
        q.append(s)
    return q

#makes robot move twice
p = move(p,1)
p = move(p,1)

#makes robot move 1000 times
j = 0
while (j < 1000):
    p = move(p,1)
    j=j+1

print p

#Given the list motions=[1,1] which means the robot 
#moves right and then right again, compute the posterior 
#distribution if the robot first senses red, then moves 
#right one, then senses green, then moves right again, 
#starting with a uniform prior distribution.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):

    for i in range(5):
        if world[i] == 'green':
            p[i] = p[i]*pMiss
        else: #red is detected
            p[i] = p[i]*pHit
        q.append(p[i])

    sum = 0
    for k in range(len(p)):
        sum += q[k]
    print sum #0.36
    
    for b in range(len(p)):
        q[b] = q[b]/sum
  
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q

p = sense(p,measurements[0])
p = move(p,motions[0])
p = sense(p,measurements[1])
p = move(p,motions[1])

print p
#[0.21157894736842103, 0.1515789473684211,
#0.08105263157894739, 0.16842105263157897,
#0.3873684210526316]

#[G,R,R,G,G]. Max likelihood at far right G



