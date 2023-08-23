import os.path as pth
import fastoad.cmd.api as oad
import warnings
warnings.filterwarnings(action='ignore', category=UserWarning)
warnings.filterwarnings(action='ignore', category=FutureWarning)

WORK_FOLDER_PATH = pth.join("workdir", "test_dir")

CONFIGURATION_FILE = pth.join(WORK_FOLDER_PATH, "oad_sizing_with_perfo_TO.toml")
SOURCE_FILE = pth.join(WORK_FOLDER_PATH, "problem_inputs_data_TO.xml")
oad.generate_inputs(CONFIGURATION_FILE, SOURCE_FILE, overwrite=True)

# oad.write_n2(CONFIGURATION_FILE, overwrite=True)

eval_problem = oad.evaluate_problem(CONFIGURATION_FILE, overwrite=True)
