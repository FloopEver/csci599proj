#!/usr/bin/python
# -*- coding: utf-8 -*-

# tool methods
import numpy as np


def distanceCost(point1, point2):
    return np.sqrt(np.sum((point1 - point2) ** 2))


def angleVec(vec1, vec2):
    temp = np.dot(vec1, vec2) / np.sqrt(np.sum(vec1 ** 2)) / np.sqrt(np.sum(vec2 ** 2))
    temp = np.clip(temp, -1, 1)
    theta = np.arccos(temp)
    return theta


def getUnitVec(vec):
    unit_vec = vec / np.sqrt(np.sum(vec ** 2))
    return unit_vec

