q_initial=Quaternion(degrees=1,axis=[1,0,0])
i=0
for yaw in (range(degree_u, degree_t)):
    q = to_quaternion(yaw +1, 0,  0)
    distance[i] = Quaternion.distance(q, q_initial)
    i=i+1

dlist=[]
for cd in d:
    dlist.append(cd['distance'])
