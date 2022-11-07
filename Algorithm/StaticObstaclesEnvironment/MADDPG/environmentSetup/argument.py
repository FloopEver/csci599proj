#!/usr/bin/python
# -*- coding: utf-8 -*-

# Argument parser

import argparse
import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def parse_args():
    parser = argparse.ArgumentParser("MADDPG")
    # Environment
    parser.add_argument(
        "--device",
        default=device,
        help="torch device type")
    parser.add_argument(
        "--num_units_actor",
        type=int,
        default=128,
        help="number of units in actor")
    parser.add_argument(
        "--lr_a",
        type=float,
        default=1e-3,
        help="learning rate for actor")
    parser.add_argument(
        "--num_units_critic",
        type=int,
        default=128,
        help="number of units in critic")
    parser.add_argument(
        "--lr_c",
        type=float,
        default=1e-3,
        help="learning rate for critic")
    parser.add_argument(
        "--max_grad_norm",
        type=float,
        default=0.5,
        help="max gradient norm for clip")
    parser.add_argument(
        "--learning_fre",
        type=int,
        default=5,
        help="learning frequency")
    parser.add_argument(
        "--t_nn_dep",
        type=int,
        default=0.01,
        help="soft update nn depth")
    parser.add_argument(
        "--gamma",
        type=float,
        default=0.95,
        help="discount factor")
    parser.add_argument(
        "--batch_size",
        type=int,
        default=1024,
        help="size of batch")
    parser.add_argument(
        "--memory_size",
        type=int,
        default=1e6,
        help="size memory")
    parser.add_argument(
        "--initial_step",
        type=int,
        default=10000,
        help="initial steps")
    parser.add_argument(
        "--actor_episode_start",
        type=int,
        default=60,
        help="the actor begin to work after this episode")
    parser.add_argument(
        "--max_episode",
        type=int,
        default=500,
        help="maximum episode length")
    parser.add_argument(
        "--per_episode_max_len",
        type=int,
        default=250,
        help="maximum length per episode")
    parser.add_argument(
        "--action_limit_min",
        type=float,
        default=0.1,
        help="the minimum action value")
    parser.add_argument(
        "--action_limit_max",
        type=float,
        default=3.0,
        help="the maximum action value")

    return parser.parse_args()