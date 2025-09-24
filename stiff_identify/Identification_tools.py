#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 13:21:34 2022

@author: nvilla
"""
from unbagger import Unbagger
from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
from scipy.interpolate import interp2d


plt.rcParams.update(
    {
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Palatino"],
    }
)

### PARAMETERS ###
left_leg_mass = 17.57468  # kg
right_leg_mass = 17.57468  # kg
robot_mass = 90.272192  # kg
gravity = 9.81  # m/s2
S = np.array([[0, -1], [1, 0]])


dt = 0.002  # seconds time between estimations of deflection
MA_duration = 0.01  # window length for the fex MA
left_hip_indices = np.array([0, 1, 2])
right_hip_indices = np.array([6, 7, 8])
filtered = True

### FUNCTIONS ###


def load_data(bag_file_name):
    looking_for = []
    looking_for.append("HipLeftFlexibilityStiffness_roll")
    looking_for.append("HipLeftFlexibilityStiffness_pitch")
    looking_for.append("HipRightFlexibilityStiffness_roll")
    looking_for.append("HipRightFlexibilityStiffness_pitch")

    looking_for.append("leftHipPitchDeflection")
    looking_for.append("leftHipRollDeflection")
    looking_for.append("rightHipPitchDeflection")
    looking_for.append("rightHipRollDeflection")

    looking_for.append("actualCOMPosition_X")
    looking_for.append("actualCOMPosition_Y")
    looking_for.append("actualCOMPosition_Z")

    looking_for.append("actual_sensors_cop_X")
    looking_for.append("actual_sensors_cop_Y")
    looking_for.append("actual_sensors_cop_Z")

    bg_Stiff = Unbagger(bag_file_name, looking_for)
    data = bg_Stiff.unbagged_Frame

    return data


def get_com_cop(bg, support):
    """
    the support can be 'left' or 'right'
    """

    ## Unpackaging:
    LH_stiff_x = bg["HipLeftFlexibilityStiffness_roll"].to_numpy()
    LH_stiff_y = bg["HipLeftFlexibilityStiffness_pitch"].to_numpy()
    RH_stiff_x = bg["HipRightFlexibilityStiffness_roll"].to_numpy()
    RH_stiff_y = bg["HipRightFlexibilityStiffness_pitch"].to_numpy()

    com_x = bg["actualCOMPosition_X"].to_numpy()
    com_y = bg["actualCOMPosition_Y"].to_numpy()
    com_z = bg["actualCOMPosition_Z"].to_numpy()

    cop_x = bg["actual_sensors_cop_X"].to_numpy()
    cop_y = bg["actual_sensors_cop_Y"].to_numpy()
    cop_z = bg["actual_sensors_cop_Z"].to_numpy()

    time = bg.index.to_numpy()

    ## Correcting the rigid model error:

    Nocorrected_com = []
    Nocorrected_cop = []
    for i in range(time.size):

        com = np.array([com_x[i], com_y[i], com_z[i]])
        Nocorrected_com.append(com)

        cop = np.array([cop_x[i], cop_y[i], cop_z[i]])
        Nocorrected_cop.append(cop)

    corrected_com = np.array(list(zip(*Nocorrected_com)))
    corrected_cop = np.array(list(zip(*Nocorrected_cop)))

    corrected_data = {
        "LH_stiffness_x": LH_stiff_x,
        "LH_stiffness_y": LH_stiff_y,
        "RH_stiffness_x": RH_stiff_x,
        "RH_stiffness_y": RH_stiff_y,
        "mismatch_x": corrected_com[0] - corrected_cop[0],
        "mismatch_y": corrected_com[1] - corrected_cop[1],
        "time": time,
    }
    print("Corrected CoM and CoP measures.")
    return pd.DataFrame(corrected_data)


def load_corrector(settings=None):

    from flex_joints import Flex

    H_stiff = np.array([1, 1, 1, 1])
    H_damp = 2 * np.sqrt(H_stiff)
    flexToJoint = np.array([0, 0, 0.09])

    settings = dict(
        left_stiffness=H_stiff[:2],
        left_damping=H_damp[:2],
        right_stiffness=H_stiff[:2],
        right_damping=H_damp[:2],
        flexToJoint=flexToJoint,
        dt=dt,
        MA_duration=MA_duration,
        left_hip_indices=left_hip_indices,
        right_hip_indices=right_hip_indices,
        filtered=filtered,
    )
    flex = Flex()
    flex.initialize(settings)

    return flex


def get_com_cop_corrected(bg, support, corrector_settings=None):
    """
    the support can be 'left' or 'right'
    """
    flex = load_corrector(corrector_settings)
    flexToJoint = flex.Settings()["flexToJoint"]

    ## Unpackaging:
    LH_stiff_x = bg["HipLeftFlexibilityStiffness_roll"].to_numpy()
    LH_stiff_y = bg["HipLeftFlexibilityStiffness_pitch"].to_numpy()
    RH_stiff_x = bg["HipRightFlexibilityStiffness_roll"].to_numpy()
    RH_stiff_y = bg["HipRightFlexibilityStiffness_pitch"].to_numpy()

    leftHipPitchDelta = bg["leftHipPitchDeflection"].to_numpy()
    leftHipRollDelta = bg["leftHipRollDeflection"].to_numpy()
    rightHipPitchDelta = bg["rightHipPitchDeflection"].to_numpy()
    rightHipRollDelta = bg["rightHipRollDeflection"].to_numpy()

    com_x = bg["actualCOMPosition_X"].to_numpy()
    com_y = bg["actualCOMPosition_Y"].to_numpy()
    com_z = bg["actualCOMPosition_Z"].to_numpy()

    cop_x = bg["actual_sensors_cop_X"].to_numpy()
    cop_y = bg["actual_sensors_cop_Y"].to_numpy()
    cop_z = bg["actual_sensors_cop_Z"].to_numpy()

    time = bg.index.to_numpy()

    ## Correcting the rigid model error:

    corrected_com = []
    corrected_cop = []
    for i in range(time.size):
        delta_LH = np.array([leftHipPitchDelta[i], leftHipRollDelta[i]])
        currentFlexToJoint_LH = flex.currentFlexToJoint(delta_LH)

        delta_RH = np.array([rightHipPitchDelta[i], rightHipRollDelta[i]])
        currentFlexToJoint_RH = flex.currentFlexToJoint(delta_RH)

        error_LH = currentFlexToJoint_LH - flexToJoint
        error_RH = currentFlexToJoint_RH - flexToJoint
        error_com = (error_LH * left_leg_mass + error_RH * right_leg_mass) / robot_mass
        error_support = error_LH if support == "left" else error_RH

        com = np.array([com_x[i], com_y[i], com_z[i]])
        corrected_com.append(com + error_com)

        cop = np.array([cop_x[i], cop_y[i], cop_z[i]])
        corrected_cop.append(cop + error_support)

    corrected_com = np.array(list(zip(*corrected_com)))
    corrected_cop = np.array(list(zip(*corrected_cop)))

    corrected_data = {
        "LH_stiffness_x": LH_stiff_x,
        "LH_stiffness_y": LH_stiff_y,
        "RH_stiffness_x": RH_stiff_x,
        "RH_stiffness_y": RH_stiff_y,
        "mismatch_x": corrected_com[0] - corrected_cop[0],
        "mismatch_y": corrected_com[1] - corrected_cop[1],
        "time": time,
    }
    print("Corrected CoM and CoP measures.")
    return pd.DataFrame(corrected_data)


def arrange_data(
    frame, support, axis, t_extremes=None, sp_remove_st=None, sw_remove_st=None
):
    """axis refers to the stiffness that we are identifying:
    use 'x' for roll and 'y' for pitch.

    t_extremes is a list with [t_min, t_max] that define the experiment time.
    """

    other_axis = "y" if axis == "x" else "x"
    swing = "right" if support == "left" else "left"
    supp = support[0].capitalize()
    swin = swing[0].capitalize()

    supp_stiff = frame[supp + "H_stiffness_" + other_axis]
    time = frame.time

    if t_extremes is None:
        t_min = 0
        t_max = time[time.size - 1]

        start_stiff = supp_stiff[0]
        final_stiff = supp_stiff[supp_stiff.size - 1]

        for i, t in enumerate(time):
            if supp_stiff[i] != start_stiff:
                t_min = t
                break
        for i, t in enumerate(reversed(time)):
            if supp_stiff[supp_stiff.size - 1 - i] != final_stiff:
                t_max = t
                break
    else:
        t_min = t_extremes[0]
        t_max = t_extremes[1]

    frame = frame[frame["time"] > t_min][frame["time"] < t_max]

    supp_stiff_values = frame[supp + "H_stiffness_" + other_axis].unique()

    wait_stabilization = 0.2  # 20% of time discarted waiting after set each stiffnes
    n_std_devs = 2  # for 95% of probability
    supp_stiffness = []
    swin_stiffness = []
    other_supp_stiffness = []
    other_swin_stiffness = []
    mean_mismatch_x = []
    mean_mismatch_y = []
    error_bars_x = []
    error_bars_y = []

    for support_st in supp_stiff_values:
        local_frame = frame[frame[supp + "H_stiffness_" + other_axis] == support_st]
        swin_stiff_values = local_frame[swin + "H_stiffness_" + other_axis].unique()

        for swing_st in swin_stiff_values:
            current_swing_IDs = (
                local_frame[local_frame[swin + "H_stiffness_" + other_axis] == swing_st]
            ).index

            N = current_swing_IDs.size
            IDs = current_swing_IDs[round(wait_stabilization * N) :]

            supp_stiffness.append(support_st)
            swin_stiffness.append(swing_st)
            other_supp_stiffness.append(
                local_frame[supp + "H_stiffness_" + axis][IDs[0]]
            )
            other_swin_stiffness.append(
                local_frame[swin + "H_stiffness_" + axis][IDs[0]]
            )

            mean_mismatch_x.append(local_frame["mismatch_x"][IDs].mean())
            mean_mismatch_y.append(local_frame["mismatch_y"][IDs].mean())
            error_bars_x.append(local_frame["mismatch_x"][IDs].std() * n_std_devs)
            error_bars_y.append(local_frame["mismatch_y"][IDs].std() * n_std_devs)

    processed = {
        supp + "H_stiffness_" + other_axis: supp_stiffness,
        supp + "H_stiffness_" + axis: other_supp_stiffness,
        swin + "H_stiffness_" + other_axis: swin_stiffness,
        swin + "H_stiffness_" + axis: other_swin_stiffness,
        "mean_mismatch_x": mean_mismatch_x,
        "mean_mismatch_y": mean_mismatch_y,
        "error_bars_x": error_bars_x,
        "error_bars_y": error_bars_y,
    }

    processed_df = pd.DataFrame(processed).sort_values(
        [supp + "H_stiffness_" + other_axis, swin + "H_stiffness_" + other_axis]
    )

    ### Removing undesired stiffness
    if sp_remove_st is not None:
        for st in sp_remove_st:
            processed_df = processed_df[
                processed_df[supp + "H_stiffness_" + other_axis] != st
            ]
            processed_df = processed_df[
                processed_df[supp + "H_stiffness_" + axis] != st
            ]
    if sw_remove_st is not None:
        for st in sw_remove_st:
            processed_df = processed_df[
                processed_df[swin + "H_stiffness_" + other_axis] != st
            ]
            processed_df = processed_df[
                processed_df[swin + "H_stiffness_" + axis] != st
            ]

    swings = processed_df[swin + "H_stiffness_x"].unique()
    for sw in swings:
        ids = processed_df[processed_df[swin + "H_stiffness_" + axis] == sw].index
        if ids.size < 4:
            processed_df.drop(ids, inplace=True)
    print("Cleaned data.")
    return processed_df


def get_contour_data(data, support, axis, case="mean"):

    other_axis = "y" if axis == "x" else "x"
    m_to_mm = 1000

    X, Y = np.meshgrid(
        data["RH_stiffness_" + other_axis].unique(),
        data["LH_stiffness_" + other_axis].unique(),
    )

    if case == "mean":
        level = data["mean_mismatch_" + axis].to_numpy()
    elif case == "lower":
        level = (data["mean_mismatch_" + axis] - data["error_bars_" + axis]).to_numpy()
    elif case == "higher":
        level = (data["mean_mismatch_" + axis] + data["error_bars_" + axis]).to_numpy()

    if support == "right":
        Z = level.reshape(X.shape, order="F") * m_to_mm
    elif support == "left":
        Z = level.reshape(X.shape) * m_to_mm

    return Z, X, Y


def identify_possible_cross(X, Y, lZ_low, lZ_high, rZ_low, rZ_high, points):

    LL = interp2d(X, Y, lZ_low)
    LH = interp2d(X, Y, lZ_high)
    RL = interp2d(X, Y, rZ_low)
    RH = interp2d(X, Y, rZ_high)

    possible_cross = []
    for point in points:

        if LL(*point) > 0:
            continue
        if LH(*point) < 0:
            continue
        if RL(*point) > 0:
            continue
        if RH(*point) < 0:
            continue

        possible_cross.append(point)

    return np.vstack(possible_cross)


def identify(
    left_supp_data,
    right_supp_data,
    axis,
    LT_range=None,
    RT_range=None,
    rm_LK=None,
    rm_RK=None,
    correct_measurements=True,
):
    """

    Parameters
    ----------
    left_supp_data : pd.DataFrame
        frame generated by `load_data` with the rosbag obtained with the left support
    right_sup_data : pd.DataFrame
        frame generated by `load_data` with the rosbag obtained with the right support
    axis : string
        it is either "x" or "y"
    LT_range : list, optional
        [t_min, t_max] that identify the left experiment measurement. The default is None.
    RT_range : TYPE, optional
        [t_min, t_max] that identify the right experiment measurement. The default is None.
    rm_LK : list, optional
        list of left stiffness values to remove from the both frames.
    rm_RK : list, optional
        list of right stiffness values to remove from the both frames.

    Returns
    -------
    Possible cross points, plotting data.

    """
    if correct_measurements:
        left_frame = get_com_cop_corrected(left_supp_data, "left")
        right_frame = get_com_cop_corrected(right_supp_data, "right")
    else:
        left_frame = get_com_cop(left_supp_data, "left")
        right_frame = get_com_cop(right_supp_data, "right")
    left_arranged = arrange_data(left_frame, "left", axis, LT_range, rm_LK, rm_RK)
    right_arranged = arrange_data(right_frame, "right", axis, RT_range, rm_RK, rm_LK)

    LZ_mean, _, _ = get_contour_data(left_arranged, "left", axis, "mean")
    LZ_low, _, _ = get_contour_data(left_arranged, "left", axis, "lower")
    LZ_high, _, _ = get_contour_data(left_arranged, "left", axis, "higher")

    RZ_mean, X, Y = get_contour_data(right_arranged, "right", axis, "mean")
    RZ_low, _, _ = get_contour_data(right_arranged, "right", axis, "lower")
    RZ_high, _, _ = get_contour_data(right_arranged, "right", axis, "higher")

    xs = np.linspace(X[0, 0], X[-1, -1], 90)
    ys = np.linspace(Y[0, 0], Y[-1, -1], 500)
    test_points = []
    for x in xs:
        for y in ys:
            test_points.append(np.array([x, y]))

    points = np.vstack(test_points)
    may_cross = identify_possible_cross(X, Y, LZ_low, LZ_high, RZ_low, RZ_high, points)

    return {
        "cross_patch": may_cross,
        "X": X,
        "Y": Y,
        "LZ_mean": LZ_mean,
        "LZ_low": LZ_low,
        "LZ_high": LZ_high,
        "RZ_mean": RZ_mean,
        "RZ_low": RZ_low,
        "RZ_high": RZ_high,
    }


### PLOTS ###


def check_time_stiffness(data, legend=False, plot_name=None):

    figure, ax = plt.subplots()
    if "time" in data.columns:
        time = data["time"].to_numpy()
    else:
        time = data.index.to_numpy()

    names = [
        "HipLeftFlexibilityStiffness_roll",
        "HipLeftFlexibilityStiffness_pitch",
        "HipRightFlexibilityStiffness_roll",
        "HipRightFlexibilityStiffness_pitch",
        "LH_stiffness_x",
        "LH_stiffness_y",
        "RH_stiffness_x",
        "RH_stiffness_y",
    ]

    for name in names:
        if name in data.columns:
            ax.plot(time, data[name], label="$" + name + "$")
    if legend:
        figure.legend()

    ax.grid(color="0.86")

    if plot_name:
        figure.savefig("./figures/" + plot_name + ".png", format="png")


def check_time_centroidal(data, legend=False):

    figure, ax = plt.subplots()
    if "time" in data.columns:
        time = data["time"].to_numpy()
    else:
        time = data.index.to_numpy()

    names = [
        "actualCOMPosition_X",
        "actualCOMPosition_Y",
        "actual_sensors_cop_X",
        "actual_sensors_cop_Y",
        "mismatch_x",
        "mismatch_y",
    ]

    for name in names:
        if name in data.columns:
            ax.plot(time, data[name], label="$" + name + "$")
    if legend:
        figure.legend()

    ax.grid(color="0.86")


def contour_mismatch(X, Y, Z, axis):
    other_axis = "y" if axis == "x" else "x"

    fig, ax = plt.subplots()

    # nn = colors.CenteredNorm()
    nn = colors.TwoSlopeNorm(0)  # , -9, 3.6  # -0.75, 4.0
    # nn = colors.SymLogNorm(linthresh=0.03, linscale=0.03, vmin=-0.15, vmax=2, base=10)
    colormap = cm.get_cmap("RdYlBu_r")

    cp = ax.contourf(X, Y, Z, 20, norm=nn, cmap=colormap)  # , cmap="binary"
    # zc = plt.contour(X, Y, Z, [0])
    # ax.contour(zc, levels=[0])
    clb = fig.colorbar(cp)
    clb.set_label(
        r"error ($c^{" + axis + r"}\texttt{-} p^{" + axis + r"}$) [mm]",
        fontsize=14,
        rotation=270,
        labelpad=10,
    )

    ax.set_xlabel(r"right hip stiffness$_" + other_axis + r"$ [Nm/rad]")
    ax.set_ylabel(r"left hip stiffness$_" + other_axis + r"$ [Nm/rad]")

    return fig, ax


def add_zero_curves(ax, X, Y, Z, color="k"):

    zc = plt.contour(X, Y, Z, [0])
    ax.contour(zc, levels=[0], colors=color)

    return zc


def identification_map(
    X, Y, BG_mean, OTH_mean, cross_patch, axis, save=False, map_name="iden_map"
):
    bg_color = "k"
    other_color = "#4DE82E"

    fig, ax = contour_mismatch(X, Y, BG_mean, axis)
    add_zero_curves(ax, X, Y, BG_mean, bg_color)
    add_zero_curves(ax, X, Y, OTH_mean, other_color)

    ax.set_xlabel(r"right hip stiffness [Nm/rad]", fontsize=14)
    ax.set_ylabel(r"left hip stiffness [Nm/rad]", fontsize=14)

    ax.plot(cross_patch[:, 0], cross_patch[:, 1], ",g")

    if save:
        fig.savefig("./figures/" + map_name + ".png", format="png")

    return fig, ax


def check_stiffness_grid(data, support, axis):

    other_axis = "y" if axis == "x" else "x"
    swing = "right" if support == "left" else "left"
    supp = support[0].capitalize()
    swin = swing[0].capitalize()

    fig0, ax0 = plt.subplots()
    ax0.plot(
        data[supp + "H_stiffness_" + axis],
        data[swin + "H_stiffness_" + axis],
        label=swin + "\_stiff\_" + axis,
    )
    fig0.legend()
    ax0.set_title("variation of stiffness in " + axis)
    ax0.set_xlabel(r"stiffness\_support [Nm/rad]", fontsize=14)
    ax0.set_ylabel(r"stiffness\_swing [Nm/rad]", fontsize=14)

    fig1, ax1 = plt.subplots()
    ax1.plot(
        data[supp + "H_stiffness_" + axis],
        data[swin + "H_stiffness_" + other_axis],
        label=swin + "\_stiff\_" + other_axis,
    )
    ax1.plot(
        data[supp + "H_stiffness_" + axis],
        data[supp + "H_stiffness_" + other_axis],
        label=supp + "\_stiff\_" + other_axis,
    )
    fig1.legend()
    ax1.set_title("variation of stiffness in " + other_axis)
    ax1.set_xlabel(r"stiffness\_support$_" + axis + r"$ [Nm/rad]", fontsize=14)
    ax1.set_ylabel(r"stiffness [Nm/rad]", fontsize=14)


def identify_from_data(
    left_Data,
    right_Data,
    axis,
    LT_range=None,
    RT_range=None,
    rm_LK=None,
    rm_RK=None,
    map_name="id_map",
):

    D = identify(left_Data, right_Data, axis, LT_range, RT_range, rm_LK, rm_RK)

    identification_map(
        D["X"],
        D["Y"],
        D["RZ_mean"],
        D["LZ_mean"],
        D["cross_patch"],
        axis,
        save=True,
        map_name=map_name,
    )


if __name__ == "__main__":

    leftBag_name = "second_pass_LEFT_2022-10-20-15-53-41.bag"
    LT_range = [18, 1476]
    rm_LK = None

    rightBag_name = "second_pass_RIGHT_2022-10-20-15-18-46.bag"
    RT_range = [10, 1465]
    rm_RK = None

    axis = "y"

    rosbag_path_root = Path(__file__).resolve().parent / "data"
    left_bag_path = rosbag_path_root / leftBag_name
    right_bag_path = rosbag_path_root / rightBag_name

    try:
        left_Data
    except NameError:
        left_Data = load_data(left_bag_path)

    try:
        right_Data
    except NameError:
        right_Data = load_data(right_bag_path)

    D = identify(left_Data, right_Data, axis, LT_range, RT_range, rm_LK, rm_RK)

    identification_map(
        D["X"], D["Y"], D["RZ_mean"], D["LZ_mean"], D["cross_patch"], axis, save=True
    )
