import time

t0 = time.time()
desired_frq = 5 

desired_dt = 2 #  = 1/desired_frq 
i = 0

while i < 10:
    t = time.time()
    dt0 = t - t0
    if dt0 < desired_dt:
        continue
    t1 = time.time()
    time.sleep(1)
    dt = t-t0
    f = 1/dt
    print("-- %s seconds" % dt)
    print("-- %s Hz" % f)
    i += 1
    t0 = t1
