#!/usr/bin/env python
#motion planning project
from __future__ import division
from __future__ import with_statement  # for python 2.5

import time
import openravepy as rave
import argparse
from quadrotor.quadrotor_openrave import parser
from quadrotor.quadrotor_openrave import bound
from quadrotor.quadrotor_openrave import state
#import navigation
#import test
import time
import openravepy
from openravepy import *
import numpy as np

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def parse_args():
    ap = argparse.ArgumentParser()

    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--test', action='store_true')
    ap.add_argument('--params', default='quadrotor/quadrotor_openrave/params/quadrotor.yaml')

    return ap.parse_args()

def get_bounds(robot, dof):
    set_dof(robot, dof)

    env = robot.GetEnv()
    with env:
        envmin, envmax = [], []
        for b in env.GetBodies():
            ab = b.ComputeAABB()
            envmin.append(ab.pos() - ab.extents())
            envmax.append(ab.pos() + ab.extents())
        abrobot = robot.ComputeAABB()
        envmin = np.min(np.array(envmin), 0.0) + abrobot.extents()
        envmax = np.max(np.array(envmax), 0.0) - abrobot.extents()
        envmin -= np.ones_like(envmin)
        envmax += np.ones_like(envmax)
        envmin[2] = max(envmin[2], 0.0)

        robot.SetAffineTranslationLimits(envmin, envmax)
        robot.SetAffineTranslationMaxVels([0.5, 0.5, 0.5, 0.5])
        robot.SetAffineRotationAxisMaxVels(np.ones(4))

        if dof == 6:
            bounds = np.array(((envmin[0], envmin[1], envmin[2], 0.0, 0.0, -np.pi),
                               (envmax[0], envmax[1], envmax[2], 0.0, 0.0, np.pi)))
        elif dof == 4:
            bounds = np.array(((envmin[0], envmin[1], envmin[2], -np.pi),
                               (envmax[0], envmax[1], envmax[2], np.pi)))
        elif dof == 3:
        	bounds = np.array(((envmin[0], envmin[1], envmin[2]),
                               (envmax[0], envmax[1], envmax[2])))
        else:
            raise NotImplementedError('dof == 4 || dof == 6')

        return bounds

def set_dof(robot, dof):
    if dof == 6:
        robot.SetActiveDOFs(
            [], rave.DOFAffine.X | rave.DOFAffine.Y | rave.DOFAffine.Z | rave.DOFAffine.Rotation3D)
    elif dof == 4:
        robot.SetActiveDOFs(
            [], rave.DOFAffine.X | rave.DOFAffine.Y | rave.DOFAffine.Z | rave.DOFAffine.RotationAxis, [0, 0, 1])
    elif dof == 3:
    	robot.SetActiveDOFs(
            [], rave.DOFAffine.X | rave.DOFAffine.Y | rave.DOFAffine.Z)
    else:
        raise NotImplementedError('dof == 4 || dof == 6 || dof == 3')




@rave.with_destroy
def run():

    args = parse_args()
    params = parser.Yaml(file_name=args.params)
    env = rave.Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load(params.scene)
    env.UpdatePublishedBodies()
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # using import bound to define DOF's
    bounds = get_bounds(robot,3)
    # start = robot.GetActiveDOFValues()
    # print start
    robot_state = state.State(env, verbose=False)

    # Initializing the plugin
    RaveInitialize()
    RaveLoadPlugin('build/2phase_planning')
    RRTModule = RaveCreateModule(env, 'rrt_module')

    startConfig = [0,0,5]
    robot.SetActiveDOFValues(startConfig)

    with env:
    	# Phase-I
        goalConfig = [3,3,1]
        RRTModule.SendCommand("set_step_size 0.4")
        RRTModule.SendCommand("setbias 00")

        sinput1 = 'arastar ' + str(goalConfig)

        # sinput = 'help'
        print '---------START-------------'

        print "Now Running : " , sinput1
        cmdout = RRTModule.SendCommand(sinput1)

		# Phase-II
        startConfig = [0, 0, 5, 0]
        goalConfig = [3, 3, 1, 0]
        bounds = get_bounds(robot,4)
        robot.SetActiveDOFValues(startConfig)
        RRTModule.SendCommand("set_step_size 0.2")
        sinput2 = 'rrtconnect ' + str(goalConfig)
        print "Now Running : " , sinput2
        cmdout = RRTModule.SendCommand(sinput2)

        print '---------FINISH------------'

    waitrobot(robot)

    raw_input("Press enter to exit...")

if __name__ == "__main__":
    rave.RaveSetDebugLevel(rave.DebugLevel.Verbose)
    run()
