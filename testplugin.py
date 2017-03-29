#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/rrt_connect')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    rrt_module = RaveCreateModule(env,'rrt_module')
    # print rrt_module.SendCommand('help')
    print rrt_module.SendCommand('GetPath lel');
finally:
    RaveDestroy()
