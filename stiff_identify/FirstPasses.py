"""
Created on Thu Oct 20 12:37:09 2022

@author: nvilla
"""

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt


from Identification_tools import load_data, get_com_cop_corrected


def clean_first_pass(frame, support, axis, t_extremes=None):
    other_axis = "y" if axis == "x" else "x"
    supp = support[0].capitalize()

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
    wait_stabilization = 0.4  # 15%

    supp_stiffness = []
    # swin_stiffness = []
    other_supp_stiffness = []
    # other_swin_stiffness = []
    mean_mismatch_x = []
    mean_mismatch_y = []
    error_bars_x = []
    error_bars_y = []

    for support_st in supp_stiff_values:

        local_ids = frame[frame[supp + "H_stiffness_" + other_axis] == support_st].index
        N = local_ids.size
        IDs = local_ids[round(wait_stabilization * N) :]

        supp_stiffness.append(support_st)
        other_supp_stiffness.append(frame[supp + "H_stiffness_" + axis][IDs[0]])

        mean_mismatch_x.append(frame["mismatch_x"][IDs].mean())
        mean_mismatch_y.append(frame["mismatch_y"][IDs].mean())
        # for 95% of probability
        error_bars_x.append(frame["mismatch_x"][IDs].std() * 2)
        error_bars_y.append(frame["mismatch_y"][IDs].std() * 2)

    processed = {
        supp + "H_stiffness_" + other_axis: supp_stiffness,
        supp + "H_stiffness_" + axis: other_supp_stiffness,
        "mean_mismatch_x": mean_mismatch_x,
        "mean_mismatch_y": mean_mismatch_y,
        "error_bars_x": error_bars_x,
        "error_bars_y": error_bars_y,
    }

    processed_df = pd.DataFrame(processed).sort_values(
        [supp + "H_stiffness_" + other_axis]
    )

    print("Cleaned data.")
    return processed_df


def show_measurements(data, support, axis):

    other_axis = "y" if axis == "x" else "x"
    supp = support[0].capitalize()
    m_to_mm = 1000

    fig, ax = plt.subplots()

    ax.errorbar(
        data[supp + "H_stiffness_" + other_axis],
        data["mean_mismatch_" + axis] * m_to_mm,
        data["error_bars_" + axis] * m_to_mm,
        c="#22B5D3",
        marker=".",
        ls="",
        capsize=3,
        zorder=2,
    )

    min_st = data[supp + "H_stiffness_" + other_axis].min()
    max_st = data[supp + "H_stiffness_" + other_axis].max()
    ax.plot([min_st, max_st], [0, 0], "r", lw=0.5, zorder=2)

    ax.grid(color="0.86")
    ax.set_xlabel(r"stiffness$_" + other_axis + r"$ [Nm/rad]", fontsize=14)
    ax.set_ylabel(
        r"CoM error ($c^{" + axis + r"}\texttt{-} p^{" + axis + r"}$) [mm]", fontsize=14
    )


if __name__ == "__main__":
    rosbag_path_root = Path(__file__).resolve().parent / "data"

    leftBag_name = "first_pass.bag"
    left_bag_path = rosbag_path_root / leftBag_name
    l_times = [10, 385]

    rightBag_name = "first_pass.bag"
    right_bag_path = rosbag_path_root / rightBag_name
    r_times = [0, 375]

    try:
        left_Data_first
    except NameError:
        left_Data_first = load_data(left_bag_path)

    try:
        right_Data_first
    except NameError:
        right_Data_first = load_data(right_bag_path)

    left_corrected_frame = get_com_cop_corrected(left_Data_first, "left")

    left_cleaned = clean_first_pass(left_corrected_frame, "left", "y", l_times)
    show_measurements(left_cleaned, "left", "y")

    right_corrected_frame = get_com_cop_corrected(right_Data_first, "right")
    right_cleaned = clean_first_pass(right_corrected_frame, "right", "y", r_times)
    show_measurements(right_cleaned, "right", "y")
