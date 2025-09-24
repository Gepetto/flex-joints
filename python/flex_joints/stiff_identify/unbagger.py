#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 12:06:43 2022

@author: nvilla
"""
import rosbag
import warnings
import numpy as np
import pandas as pd


class Unbagger:
    def __init__(self, file_path, unbag_names=[]):

        self.file_path = file_path
        self.bag = rosbag.Bag(file_path)

        contents = self.bag.read_messages()

        for items in contents:
            if hasattr(items[1], "names"):
                self.topic_name, stat_names, self.time0 = items
                break

        self.names_type = type(stat_names)
        self.names = stat_names.names

        if type(unbag_names) is str:
            unbag_names = [unbag_names]

        unbag_dictionary = {}
        if unbag_names:
            unbag_dictionary = {name: [] for name in unbag_names}

        time = []
        # i = 0
        for _, stat_value, T in contents:
            # i += 1
            if type(stat_value) is self.names_type:
                # print(len(stat_names.names), len(self.names))
                # print("This happended at ", i)
                self.names = stat_value.names
                continue

            time.append((T - self.time0).to_sec())
            for k in unbag_dictionary.keys():
                try:
                    k_id = self.names.index(k)
                    unbag_dictionary[k].append(stat_value.values[k_id])
                except ValueError:
                    warnings.warn("Nothing in this bag is called " + k)
                    unbag_dictionary[k].append(np.nan)

        self.unbagged_Frame = pd.DataFrame(
            {**unbag_dictionary, "time": time}
        ).set_index("time")

    def look_for(self, unbag_names=[]):

        if type(unbag_names) is str:
            unbag_names = [unbag_names]

        contents = self.bag.read_messages()
        next(contents)

        unbag_dictionary = {}
        if unbag_names:
            unbag_dictionary = {name: [] for name in unbag_names}

        for _, stat_value, _ in contents:
            if type(stat_value) is self.names_type:
                self.names = stat_value.names
                continue
            for k in unbag_dictionary.keys():
                try:
                    k_id = self.names.index(k)
                    unbag_dictionary[k].append(stat_value.values[k_id])
                except ValueError:
                    warnings.warn("Nothing in this bag is called " + k)
                    unbag_dictionary[k].append(np.nan)

        for name in unbag_names:
            self.unbagged_Frame[name] = unbag_dictionary[name]

    def find(self, name_fragment):

        for name in self.names:
            if name_fragment.lower() in name.lower():
                print(name)


if __name__ == "__main__":

    explorer = Unbagger(
        "data/stiffness_estimation_left_first_dcm_nobase_2022-07-11-14-28-41.bag"
    )
