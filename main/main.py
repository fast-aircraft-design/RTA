import os.path as pth
import openmdao.api as om
import logging
import shutil
import fastoad.api as oad


DATA_FOLDER_PATH = "data"

WORK_FOLDER_PATH = "workdir"

CONFIGURATION_FILE = pth.join(WORK_FOLDER_PATH, "oad_process.yml")
SOURCE_FILE = pth.join(DATA_FOLDER_PATH, "ATR72.xml")
#oad.generate_inputs(CONFIGURATION_FILE, SOURCE_FILE, overwrite=True)
# For having log messages on screen
eval_problem = oad.evaluate_problem(CONFIGURATION_FILE, overwrite=True)