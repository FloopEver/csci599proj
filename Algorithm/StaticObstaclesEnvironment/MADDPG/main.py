#!/usr/bin/python
# -*- coding: utf-8 -*-

# MADDPG
from Algorithm.StaticObstaclesEnvironment.MADDPG.environmentSetup.argument import parse_args
from Algorithm.StaticObstaclesEnvironment.MADDPG.environmentSetup.randomSeed import setup_seed
from Algorithm.StaticObstaclesEnvironment.MADDPG.train import train

if __name__ == '__main__':
    setup_seed(10)
    arglist = parse_args()
    train(arglist)