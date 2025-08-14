import time

def do_every(period,f,*args):
    def g_tick():
        t = time.time()
        count = 0
        while True:
            count += 1
            yield max(t + count*period - time.time(),0)
    g = g_tick()
    while True:
        time.sleep(next(g))
        f(*args)

def hello(s):
    print('hello {} ({:.4f})'.format(s,time.time()))
    time.sleep(.3)

do_every(1,hello,'foo')
