#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 21 09:46:25 2022

@author: nvilla
"""

from Identification_tools import identify_from_data, load_data
import argparse
import numpy as np


def getArgs():

    parser = argparse.ArgumentParser(
        description="Produce a contour of c-p to identify the best values of stiffness"
    )

    parser.add_argument(
        "-left_bag_path", help="path to the bag obtained with left support"
    )
    parser.add_argument(
        "-right_bag_path", help="path to the bag obtained with right support"
    )
    parser.add_argument("-axis", default="y", help="axis of measured errors to use.")
    parser.add_argument(
        "-LT_range",
        default=None,
        nargs="+",
        type=float,
        help="range of times of the leg support experiment where the measurements must be taken",
    )
    parser.add_argument(
        "-RT_range",
        default=None,
        nargs="+",
        type=float,
        help="range of times of the right support experiment where the measurements must be taken",
    )
    parser.add_argument(
        "-rm_LK",
        default=None,
        nargs="+",
        type=float,
        help="values of stiffness to ignore on the left support experiment",
    )
    parser.add_argument(
        "-rm_RK",
        default=None,
        nargs="+",
        type=float,
        help="values of stiffness to ignore on the right support experiment",
    )
    parser.add_argument("-map_name", help="name of the generated identification map")

    parser.add_argument(
        "-check_timming",
        default=False,
        help="plot the used stiffness as a function of time to see and set by hand what is the range of time of the experiment",
    )

    return parser.parse_args()


if __name__ == "__main__":

    args = getArgs()

    left_Data = load_data(args.left_bag_path)
    right_Data = load_data(args.right_bag_path)

    LT_range = args.LT_range if args.LT_range is None else np.array(args.LT_range)
    RT_range = args.RT_range if args.RT_range is None else np.array(args.RT_range)
    rm_LK = args.rm_LK if args.rm_LK is None else np.array(args.rm_LK)
    rm_RK = args.rm_RK if args.rm_RK is None else np.array(args.rm_RK)
    map_name = "id_map" if args.map_name is None else args.map_name

    identify_from_data(
        left_Data=left_Data,
        right_Data=right_Data,
        axis=args.axis,
        LT_range=LT_range,
        RT_range=RT_range,
        rm_LK=rm_LK,
        rm_RK=rm_RK,
    )
