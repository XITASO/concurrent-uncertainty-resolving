import os
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
# from system_interfaces.msg import AdaptationStatus, StrategyStatus
import numpy as np
from typing import Dict


def filter_logs_by_timestamp(
    df: pd.DataFrame, lower_threshold: float, upper_threshold: float
) -> pd.DataFrame:
    """
    Filter the DataFrame based on the timestamp column.

    Parameters:
    df (pd.DataFrame): The DataFrame containing log data.
    lower_threshold (float): The lower threshold for the timestamp.
    upper_threshold (float): The upper threshold for the timestamp.

    Returns:
    pd.DataFrame: The filtered DataFrame.
    """
    filtered_df = df[
        (df["timestamp"] > lower_threshold) & (df["timestamp"] < upper_threshold)
    ]
    return filtered_df


def calculate_adaptations(df: pd.DataFrame) -> int:
    """
    Calculate the adpatations needed for a given run.
    """
    adaptation_rows = df[df["source"].str.contains("execution_", na=False)]
    adaaptation_triggered = adaptation_rows[
        adaptation_rows["adaptation_status"]
        == 1
    ]
    return len(adaaptation_triggered)

def calculate_redeploys(df: pd.DataFrame) -> int:
    """
    Calculate the adpatations needed for a given run.
    """
    num_redeploys = df[df["source"].str.contains("execution_redeploy", na=False)][df["success"]==True]
    return len(num_redeploys)


def calculate_mean_reaction_time(df: pd.DataFrame, filename: str) -> list[float] | None:
    """
    Calculates the mean reaction time from rule trigger to strategy finished successfully.
    """
    def find_true_negative_successful_strat(df:pd.DataFrame, rule_rows: pd.DataFrame, rule_name: str) -> float | None:
        if not "Dead" in rule_name:
            return None

        relevant_eval_logs = df[(df['source'] == '/evaluator_log') & (df['timestamp'] > rule['timestamp'])].iloc[0]['timestamp']
        fallback_strategy = rule_rows[
            (rule_rows["strategy_status"] == (4))
            & (rule_rows["timestamp"] > rule["timestamp"])
            & (rule_rows["timestamp"] < relevant_eval_logs)
        ]

        return fallback_strategy.iloc[0]['timestamp']
    reaction_times = []
    #{
    #    "ok": [],
    #    "warning": [],
    #    "error": []
    #}
    skipped_rules = 0
    analyzer_rows = df[df["source"] == "analyzer"]
    rule_triggered_rows = analyzer_rows[
        analyzer_rows["strategy_name"].isna() &
        (analyzer_rows["strategy_status"] != 5)
    ]
    if len(rule_triggered_rows) == 0:
        print(
            f"No rule triggered rows found in {filename}, skipping reaction time calculation."
        )
        return None
    for _, rule in rule_triggered_rows.iterrows():
        rule_name: str = rule["rule_name"]
        if "Focus" in rule_name:
            criticality_level = 'ok'
        elif "Bad" in rule_name:
            criticality_level = "warning"
        elif "Dead" in rule_name:
            criticality_level = "error"
        else:
            print('This should not happen')
        rule_rows = analyzer_rows[analyzer_rows["rule_name"] == rule_name]
        strategy_finished = rule_rows[
            ((rule_rows["strategy_status"] == (3 ))
            #| (rule_rows["strategy_status"] == (5 )))
            & (rule_rows["timestamp"] > rule["timestamp"]))
        ].sort_values("timestamp")

        if len(strategy_finished) <= 0:
            skipped_rules += 1
            continue
        name_successful_strategy = strategy_finished.iloc[0]['strategy_name']
        if type(name_successful_strategy) is float:
            continue
        #try:
        correct_strategy_start = df[(df['source']=='planning') & (df['strategy_name']==name_successful_strategy) & (df["timestamp"] > rule["timestamp"])].iloc[0]
        time_correct_strat_selected = correct_strategy_start['timestamp']
        #except:
        #    time_correct_strat_selected = find_true_negative_successful_strat(df, rule_rows, rule_name)
        #    if time_correct_strat_selected is None:
        #        print("No successful strategy found")
        #        continue
        reaction_times.append(
            time_correct_strat_selected - rule["timestamp"]
        )

    if len(reaction_times) == 0:
        print(
            f"No valid reaction times found in {filename}, skipping mean calculation."
        )
        return None
    if skipped_rules > 0:
        print(
            f"Skipped {skipped_rules} rules in {filename} due to no successful strategy finish."
        )
    return reaction_times

def calculate_mean_reaction_time_v2(df: pd.DataFrame, filename: str) -> list[float] | None:
    """
    Calculates the mean reaction time from rule trigger to strategy finished successfully.
    """
    # idea staore each triggered rule and wait until we find the resolving strat
    analyzer_logs = df[(df['source'] == 'analyzer')].sort_values("timestamp").values
    current_triggered_rules = {}
    react_times = []
    # if no adaptation given and status is 0 its the first time, add rule, but do check
    for log in analyzer_logs:
        # print(filename)
        # print(current_triggered_rules, log[3], log[4])
        if log[4] is np.nan and log[6] == 0:
            assert(log[3] not in current_triggered_rules)
            # save rule trigger time
            current_triggered_rules[log[3]] = log[0]
        else:
            # we should know that strat already
            assert(log[3] in current_triggered_rules)
            # if we resolved the strat, i.e. status 3 or 5, safe the time diff
            if log[6] == 4:
                pass
            elif log[6] == 3:
                react_times.append(log[0] - current_triggered_rules[log[3]])
                del current_triggered_rules[log[3]]
            elif log[6] == 5:
                react_times.append(log[0] - current_triggered_rules[log[3]])
                del current_triggered_rules[log[3]]

    if len(current_triggered_rules) != 0:
        print("WARNING:, %d, unresolves rules" %len(current_triggered_rules))
        # print(current_triggered_rules)
    print(react_times)

    return react_times
    

def calculate_automatically_resolved_strategies(df: pd.DataFrame) -> int:
    return len(df[df['strategy_status'] == 5])

def calculate_system_down_time(df: pd.DataFrame, filename: str) -> float | None:
    """
    Calcualte the time from when a failure case is produced until the strategy finished successfully.
    """
    segmentation_msg_timestamps = df[df["source"] == "/evaluator_log"].sort_values("timestamp")["timestamp"].values
    # if len(segmentation_msg_timestamps) 
    # .sort_values("timestamp")
    diffs = np.diff(segmentation_msg_timestamps)
    # missing at least one frame
    threshold = 200000000
    diffs_masked = diffs[diffs > threshold]
    down_time = diffs_masked.sum()      
    
    return down_time

def calculate_relative_strategy_success(df: pd.DataFrame) -> float | None:
    triggered_strategies = len(df[(df["source"] == "planning") & (df["strategy_status"] == 1)])
    successful_strategies = len(df[(df["source"] == "analyzer") & (df["strategy_status"] == 3)])
    if triggered_strategies == 0:
        return None
    if triggered_strategies < successful_strategies:
        raise Exception("More successful strategies than triggered strategies, data inconsistency.")

    return (successful_strategies) / triggered_strategies

def calculate_down_time_periods(df: pd.DataFrame):
    """Return all (start, end) periods between where no iou is logged and the timestamp diff > 0.

    Returns
    -------
    list[tuple[float, float]]
        Each tuple is (start_timestamp, end_timestamp). Empty list if not
        enough rows or no positive gaps.
    """
    exec_rows = df[df["iou"] == 0.0].sort_values("timestamp")
    ts = exec_rows["timestamp"]
    diffs = ts.diff()
    mask = diffs > 0
    starts = ts.shift(1)[mask]
    ends = ts[mask]
    return list(zip(starts.tolist(), ends.tolist()))

def caluculate_error_amounts(df: pd.DataFrame) -> Dict[str, int]:
    """Calculate the amount of different errors from the log DataFrame."""
    error_counts: Dict[str, int] = {}
    error_rows = df[df["source"] == "scenario_executor"]
    for _, row in error_rows.iterrows():
        error_name = row["gt_failure_name"]
        if error_name not in error_counts:
            error_counts[error_name] = 0
        error_counts[error_name] += 1
    return error_counts

def calculate_new_success_metric(df: pd.DataFrame):
    try:
        triggered_strategies = len(df[(df["source"] == "planning") & (df["strategy_status"] == 1)])
        successful_strategies = len(df[(df["source"] == "analyzer") & (df["strategy_status"] == 3)])
        resolved_strategies = len(df[(df["source"] == "analyzer") & (df["strategy_status"] == 5)])
        return (resolved_strategies + successful_strategies) / triggered_strategies
    except:
        return 0


def mean_Iou(df: pd.DataFrame) -> list[float]:
    """Calculate the mean IoU from evaluator logs."""
    rows = df[df["source"] == "/evaluator_log"]
    if len(rows) == 0:
        return None
    ious = rows["iou"].astype(float)
    return ious.tolist()

def round_to_significant_figures(num: float, sig_figs: int = 3) -> str:
    """
    Round a number to a specific number of significant figures.

    Parameters:
    num (float): The number to be rounded.
    sig_figs (int): The number of significant figures to retain.

    Returns:
    str: The rounded number as a string.
    """
    if num == 0 or np.isnan(num):
        return str(0)
    else:
        # Determine the number of digits before the decimal point
        num_digits = int(f"{num:e}".split('e')[1])  # Get exponent from scientific notation
        # Compute the shift needed to maintain significant figures
        shift = sig_figs - num_digits - 1
        str_num = str(round(num, shift))
        exp_chars = sig_figs if shift == 0 else sig_figs + 1
        if len(str_num) < exp_chars:
            str_num = str_num + "0"
        return str_num[:exp_chars]

def calculate_unnecessary_redeploys(df: pd.DataFrame):
    try:
        redeploys_necessary = len(df[(df["source"] == "scenario_executor") & (df["gt_failure_name"].str.contains("hard"))])
        redeploys_performed = len(df[(df["source"] == "planning") & (df["strategy_name"].str.contains("redeploy"))])
        return redeploys_performed - redeploys_necessary
    except:
        return 0

def is_valid_log_file(df: pd.DataFrame) -> bool:
    timestamp_col = df["timestamp"].tolist()

    if len(timestamp_col) == 0:
        return False

    if 0 in timestamp_col:
        return False

    for timestamp in timestamp_col:
        stamp = str(timestamp)
        if stamp.startswith("176") and len(stamp) >= 19:
            return False

    evaluator_rows = df[df["source"] == "/evaluator_log"]
    if len(evaluator_rows) < 10:
        return False

    return True

def main() -> None:
    log_file_folder = "/home/code/mapek-bt/log_dump"
    scenarios = []
    # scenarios to evaluate
    for s in [
        # "scenario_image_degradation_",
        # "scenario_message_drop_",
        # "scenario_F1+S1_",
        "",
    ]:
        # rule set
        for r in ["settings"]:
            for m in [
                "",
                "_cost",
                "_deps",
                "_crit",
                "_crit_cost",
                "_deps_cost",
                "_deps_crit",
                "_deps_crit_cost",
            ]:
                scenarios.append(s + r + m)

    overall_result = {}
    for scenario_folder in scenarios:
        scenarion_down_times = []
        scenario_ok_reaction_times = []
        scenario_warning_reaction_times = []
        scenario_error_reaction_times = []
        scenario_reaction_times = []
        scenario_strategy_success = []
        scenario_error = {}
        scenario_adaptations = []
        num_redeploys = []
        auto_resolved_rules = []
        unnecessary_redeploys = []
        # scenario_mIou = []
        for single_run in sorted(
            os.listdir(os.path.join(log_file_folder, scenario_folder))
        ):
            full_path = os.path.join(log_file_folder, scenario_folder, single_run)
            df = pd.read_csv(full_path)
            unn_redeploys = calculate_unnecessary_redeploys(df)
            unnecessary_redeploys.append(unn_redeploys)

            if scenario_folder == "settings_deps" and unn_redeploys > 2:
               continue
            log_file_valid = is_valid_log_file(df)

            if not log_file_valid:
                print(f"{full_path} doesn't seem to be a valid logfile")
                #os.remove(full_path)
                #continue

            adap = calculate_adaptations(df)
            scenario_adaptations.append(adap)
            num_redeploys.append(calculate_redeploys(df))
            relative_strategy_success = calculate_new_success_metric(df)
            if relative_strategy_success is not None:
                scenario_strategy_success.append(relative_strategy_success)
            reaction_time = calculate_mean_reaction_time_v2(df, full_path)
            if reaction_time is not None:
                #scenario_ok_reaction_times.extend(reaction_time['ok'])
                #scenario_warning_reaction_times.extend(reaction_time['warning'])
                #scenario_error_reaction_times.extend(reaction_time['error'])
                scenario_reaction_times.extend(reaction_time)
            down_time = calculate_system_down_time(df, full_path)
            if down_time is not None:
             scenarion_down_times.append(down_time)
            auto_resolved_rules.append(calculate_automatically_resolved_strategies(df))
            # mean_iou = mean_Iou(df)
            # if mean_iou is not None:
            #     scenario_mIou.extend(mean_iou)

            errors = caluculate_error_amounts(df)
            for key, value in errors.items():
                if key not in scenario_error:
                    scenario_error[key] = 0
                scenario_error[key] += value

        overall_result[scenario_folder] = {}

        overall_result[scenario_folder]["adaptations"] = {
            "mean": np.mean(scenario_adaptations),
            "std": np.std(scenario_adaptations),
        }

        overall_result[scenario_folder]["unnecessary_redeploys"] = {
            "mean": np.mean(unnecessary_redeploys),
            "std": np.std(unnecessary_redeploys),
        }

        overall_result[scenario_folder]["auto_resolved"] = {
            "mean": np.mean(auto_resolved_rules),
            "std": np.std(auto_resolved_rules),
        }

        overall_result[scenario_folder]["error_counts"] = scenario_error
        overall_result[scenario_folder]["num_redeploys"] = {
            "mean": np.mean(num_redeploys),
            "std": np.std(num_redeploys),
        }

        overall_result[scenario_folder]["strategy_success"] = {
            "mean": np.mean(scenario_strategy_success) if len(scenario_strategy_success) > 0 else np.nan,
            "std": np.std(scenario_strategy_success) if len(scenario_strategy_success) > 0 else np.nan,
        }

        overall_result[scenario_folder]["reaction_time"] = {
           "mean": np.mean(scenario_reaction_times) / 1e9 if len(scenario_reaction_times) > 0 else np.nan,
            "std": np.std(scenario_reaction_times) / 1e9 if len(scenario_reaction_times) > 0 else np.nan,
        }

#        overall_result[scenario_folder]["warning_reaction_time"] = {
#           "mean": np.mean(scenario_warning_reaction_times) / 1e9 if len(scenario_warning_reaction_times) > 0 else np.nan,
#            "std": np.std(scenario_warning_reaction_times) / 1e9 if len(scenario_warning_reaction_times) > 0 else np.nan,
#        }
#
#        overall_result[scenario_folder]["error_reaction_time"] = {
#           "mean": np.mean(scenario_error_reaction_times) / 1e9 if len(scenario_error_reaction_times) > 0 else np.nan,
#            "std": np.std(scenario_error_reaction_times) / 1e9 if len(scenario_error_reaction_times) > 0 else np.nan,
#        }

        overall_result[scenario_folder]["down_time"] = {
            "mean": np.mean(scenarion_down_times) / 1e9 if len(scenarion_down_times) > 0 else np.nan,
            "std": np.std(scenarion_down_times) / 1e9 if len(scenarion_down_times) > 0 else np.nan,
        }

        # overall_result[scenario_folder]["mIoU"] = {
        #     "mean": np.mean(scenario_mIou) if len(scenario_mIou) > 0 else np.nan,
        #     "std": np.std(scenario_mIou) if len(scenario_mIou) > 0 else np.nan,
        # }

    print_latex(result_dict=overall_result)

def print_latex(result_dict: Dict[str, Dict[str, Dict[str, float]]]) -> None:
    """Print a LaTeX table summarizing managing system evaluation.

    Expected structure per scenario key:
    result_dict[scenario] = {
        'adaptations': {'mean': float, 'std': float},
        'reaction_time': {'mean': float, 'std': float},
        'down_time': {'mean': float, 'std': float},
        'mIoU': {'mean': float, 'std': float}
    }
    Scenario key naming is assumed to encode flags via suffixes _deps, _crit, _cost.
    True is rendered as 'x', False as '\\checkmark'.
    """
    def flag(active: bool) -> str:
        return '\\checkmark' if active else 'x'
    
    print("\n\n\n")

    # Print LaTeX table header
    print("\\begin{table*}")
    print("\\begin{tabular}{ccccccc}")
    print("\\toprule")
    print("Dependencies & \makecell{Criticality\\\\ Level} & \makecell{System \\\\ Impact} & \\frac{$Strat_{resolved}$}{$Strat_{exec}$} & Reaction Time (s) & System Downtime (s) & \# $redeploys_{unneccessary}$ \\\\")
#    print("Dependencies & Criticality Level & System Impact & Stragies Successful & Reaction Time (s) & System Downtime (s) & Adaptations & Hard Drop Segmentation & Image Shift & Autofocus needed & Drop RBG Camera & Hard Drop Sensor Fusion & Image Degredation & Drop Segmentation & Hard Drop RGB Camera \\\\")
    print("\\midrule")

    latex_table = []

    for scenario in sorted(result_dict.keys()):
        deps = '_deps' in scenario
        crit = '_crit' in scenario
        cost = '_cost' in scenario
        data = result_dict[scenario]

        adap_mean = data['adaptations']['mean']
        adap_std = data['adaptations']['std']

        redeploy_mean = data['num_redeploys']['mean']
        redeploy_std = data['num_redeploys']['std']

        unn_redeploy_mean = data['unnecessary_redeploys']['mean']
        unn_redeploy_std = data['unnecessary_redeploys']['std']

        auto_resolved_mean = data['auto_resolved']['mean']
        auto_resolved_std = data['auto_resolved']['std']

        strat_success_mean = data['strategy_success']['mean']
        strat_success_std = data['strategy_success']['std']

        rt_mean = data['reaction_time']['mean']
        rt_std = data['reaction_time']['std']

        dt_mean = data['down_time']['mean']
        dt_std = data['down_time']['std']

        # miou_mean = data['mIoU']['mean']
        # miou_std = data['mIoU']['std']

        if np.isnan(adap_mean) and np.isnan(adap_std):
            adaptations = "N/A"
        else:
            adaptations = f"{round(adap_mean)} $\\pm$ {round_to_significant_figures(adap_std)}"

        if np.isnan(redeploy_mean) and np.isnan(redeploy_std):
            redeploys = "N/A"
        else:
            redeploys = f"{round_to_significant_figures(redeploy_mean)} $\\pm$ {round_to_significant_figures(redeploy_std)}"

        if np.isnan(unn_redeploy_mean) and np.isnan(unn_redeploy_std):
            unn_redeploys = "N/A"
        else:
            unn_redeploys = f"{round_to_significant_figures(redeploy_mean)} $\\pm$ {round_to_significant_figures(redeploy_std)}"

        if np.isnan(auto_resolved_mean) and np.isnan(auto_resolved_std):
            auto_resolved = "N/A"
        else:
            auto_resolved = f"{round_to_significant_figures(auto_resolved_mean)} $\\pm$ {round_to_significant_figures(auto_resolved_std)}"

        if np.isnan(strat_success_mean) and np.isnan(strat_success_std):
            strat_success = "N/A"
        else:
            strat_success = f"{round_to_significant_figures(strat_success_mean )} $\pm$ {round_to_significant_figures(strat_success_std )}"

        if np.isnan(rt_mean) and np.isnan(rt_std):
            reaction_time = "N/A"
        else:
            reaction_time = f"{round_to_significant_figures(rt_mean)} $\pm$ {round_to_significant_figures(rt_std)}"

        if np.isnan(dt_mean) and np.isnan(dt_std):
            down_time = "N/A"
        else:
            down_time = f"{round_to_significant_figures(dt_mean)} $\pm$ {round_to_significant_figures(dt_std)}"

        # if np.isnan(miou_mean) and np.isnan(miou_std):
        #     miou = "N/A"
        # else:
        #     miou = f"{round_to_significant_figures(miou_mean)} $\pm$ {round_to_significant_figures(miou_std)}"

        #hard_drop_segmentation = data["error_counts"]["do_hard_drop_segmentation"]
        #image_shift = data["error_counts"]["image_shift"]
        #autofocus_needed = data["error_counts"]["autofocus_needed"]
        #drop_rbg_camera = data["error_counts"]["do_drop_rgb_camera"]
        #hard_drop_sensor_fusion = data["error_counts"]["do_hard_drop_sensor_fusion"]
        #image_degredation = data["error_counts"]["image_degradation"]
        #drop_segmentation = data["error_counts"]["do_drop_segmentation"]
        #drop_sensor_fusion = data["error_counts"]["do_drop_sensor_fusion"]
        #hard_drop_rgb_camera = data["error_counts"]["do_hard_drop_rgb_camera"]

        latex_table.append(f"{flag(deps)} & {flag(crit)} & {flag(cost)} & {strat_success} & {reaction_time} & {down_time} & {unn_redeploys} \\\\") #& {hard_drop_segmentation} & {image_shift} & {autofocus_needed} & {drop_rbg_camera} & {hard_drop_sensor_fusion} & {image_degredation} & {drop_segmentation} & {hard_drop_rgb_camera}\\\\")

    # Print table rows and footer
    print("\n".join(latex_table))
    print("\\bottomrule")
    print("\\end{tabular}")
    print("\\begin{table*}")


if __name__ == "__main__":
    main()