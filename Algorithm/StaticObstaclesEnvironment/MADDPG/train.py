#!/usr/bin/python
# -*- coding: utf-8 -*-

# Core training func

from Algorithm.StaticObstaclesEnvironment.MADDPG.environmentSetup.APF import APF


def train(arglist):

    """step1: create the enviroment"""
    apf = APF()
