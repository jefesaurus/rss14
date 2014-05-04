from numpy import *
import string

points = []
count = 100000
with open('points.txt', 'r') as f:
  for i in xrange(count):
    points.append([float(a) for a in f.readline()[:-1].split(',')])


A = mat(points)
u,s,vh = linalg.linalg.svd(A)
v = vh.conj().transpose()
print v[:,-1] 


#p = [(0,0,0), (1,1,0), (1,0,0), (0,1,0)]

#print planeFit(p)
