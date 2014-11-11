#! /usr/bin/env python


import sixaxis
import pprint

sixaxis.init("/dev/input/js1")
pp = pprint.PrettyPrinter(indent=4)
while(1):
    state = sixaxis.get_state()
    if(state['triangle'] == True):
        break
    pp.pprint(state)
