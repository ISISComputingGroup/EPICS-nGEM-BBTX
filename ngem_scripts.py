import time
from genie_python import genie as g

## is_local=True will prefix with local instrument
def start_ngem():
    g.set_pv("NGEM_01:UPDATETOF:SP",1, is_local=True)
    time.sleep(0.1)
    g.set_pv("NGEM_01:START:SP",1, is_local=True)

def stop_ngem():
    g.set_pv("NGEM_01:STOP:SP",1, is_local=True)

