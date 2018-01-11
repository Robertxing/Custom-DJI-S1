#Kalman Filter
#(Combining information in the presence of uncertainty)

#kalman Filters
'''
#maximise gaussian

from math import *

def f(mu, sigma2,x)
    return 1/sqrt(2.*pi*sigma)*exp(-.5*(x-mu)**2 / sigma2)

printf f(10.0,4.0,8.0) #maximise by changing x value to 10 

# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.

def update(mean1, var1, mean2, var2):
    new_mean = ((var2*mean1) + (var1*mean2))/(var2 + var1)
    new_var = 1/((1/var2)+(1/var1))
    return [new_mean, new_var]

print update(10.,8.,13., 2.)

# Write a program that will predict your new mean
# and variance given the mean and variance of your 
# prior belief and the mean and variance of your 
# motion. 

def update(mean1, var1, mean2, var2): #multiplication = measurement update
    new_mean = ((var2*mean1) + (var1*mean2))/(var2 + var1)
    new_var = 1/((1/var2)+(1/var1))
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2): #motion update = addition
    new_mean = mean1 + mean2
    new_var =var1 + var2
    return [new_mean, new_var]

print predict(10.0, 4.0, 12.0, 4.0)
'''
# Write a program that will iteratively update and
# predict based on the location measurements 
# and inferred motions shown below. 

def update(mean1, var1, mean2, var2):
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1./(1./var1 + 1./var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000

#Please print out ONLY the final values of the mean
#and the variance in a list [mu, sig].

for i in range(len(measurements)):
    [mu,sig] = update(mu,sig,measurements[i],measurement_sig)
    #print 'update: ',[mu,sig]
    [mu,sig] = predict(mu,sig,motion[i],motion_sig)
    #print 'update: ', [mu,sig]

print [mu, sig]


