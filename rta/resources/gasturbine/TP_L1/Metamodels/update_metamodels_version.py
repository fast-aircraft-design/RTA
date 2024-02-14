""" This file updates metamodels version from previous to actual scikit-learn to remove UserWarnings"""
import os
import os.path as pth
import joblib
import sklearn

# WARNING: COMPATIBILITY SHOULD BE ENSURED BEFORE EXECUTING THIS SCRIPT
sklearn_version = sklearn.__version__
sklearn_version = sklearn_version.replace(".", "_")
print(
    "WARNING: scikit-learn version compatibility check must be done before rewritting the models."
)
print(
    "Should the metamodels be rewritten with sklearn version {} (y/n)?".format(
        sklearn_version
    )
)
user_answer = input()

if user_answer == "y":
    # Simply load and save the modules with actual scikit-learn version.
    model_list = os.listdir(pth.dirname(__file__))
    model_list.remove(pth.split(__file__)[-1])
    for model in model_list:
        loaded_model = joblib.load(open(pth.join(pth.dirname(__file__), model), "rb"))
        joblib.dump(loaded_model, pth.join(pth.dirname(__file__), model))
    print("Model replaced")

else:
    print("Stopping")
