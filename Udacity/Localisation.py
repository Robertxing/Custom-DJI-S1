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
