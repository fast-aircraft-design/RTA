import os.path as pth
import fastoad.api as oad

DATA_FOLDER_PATH = "data"

WORK_FOLDER_PATH = pth.join("workdir", "TPAD_baseline_ref_L1")

CONFIGURATION_FILE = pth.join(WORK_FOLDER_PATH, "sizing", "TPAD_sizing.toml")
SOURCE_FILE = pth.join(DATA_FOLDER_PATH, "ATR72.xml")
#oad.generate_inputs(CONFIGURATION_FILE, SOURCE_FILE, overwrite=True)

oad.write_n2(CONFIGURATION_FILE)
# For having log messages on screen
# eval_problem = oad.evaluate_problem(CONFIGURATION_FILE, overwrite=True)
