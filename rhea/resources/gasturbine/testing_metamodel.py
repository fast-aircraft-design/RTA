import os
import sklearn
from sklearn.preprocessing import PolynomialFeatures
import os.path as pth
import numpy as np
import joblib

Altitude = np.linspace(1000,18001,3)
Mach = np.linspace(0.05,0.51,3)
Power_rate = np.linspace(0.01,1,3)
Phase = np.arange(0,9,1)

def get_metamodel_prediction(phase=1.0,altitude=0.0,mach=0.01, power_rate=1.0):
    filename = pth.join(pth.dirname(__file__), "TP_L1\Metamodels")
    if phase == 1:
        filename = pth.join(filename , 'TP_L1_NTO_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach], dtype=object).reshape(1, -1))

    elif phase == 2:
        filename = pth.join(filename , 'TP_L1_MCL_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach], dtype=object).reshape(1, -1))

    elif phase == 3:
        filename = pth.join(filename , 'TP_L1_CRZ_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach, power_rate], dtype=object).reshape(1, -1))


    elif phase == 5:
        filename = pth.join(filename , 'TP_L1_CRZ_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach, power_rate], dtype=object).reshape(1, -1))

    elif phase == 8:
        filename = pth.join(filename , 'TP_L1_RTO_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach], dtype=object).reshape(1, -1))

    elif phase == 9:
        filename = pth.join(filename , 'TP_L1_MCT_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach], dtype=object).reshape(1, -1))

    else:
        filename = pth.join(filename , 'TP_L1_MCL_PSFC.sav')
        poly = PolynomialFeatures(degree=3)
        x = poly.fit_transform(np.array([altitude, mach], dtype=object).reshape(1, -1))

    loaded_model = joblib.load(open(filename, 'rb'))
    psfc = loaded_model.predict(x)[0]

    return psfc

# Generate data for different version
sklearn_version =sklearn.__version__
sklearn_version = sklearn_version.replace('.','_')
# data_list = []
# for phase in Phase:
#     for alti in Altitude:
#         for mach in Mach:
#             for power_rate in Power_rate:
#                 psfc = get_metamodel_prediction(phase=phase, altitude=alti, mach=mach, power_rate=power_rate)
#                 data_list.append([phase, alti, mach, power_rate, float(psfc)])
#
# outfilename = pth.join(pth.dirname(__file__), "TP_L1/version_validation", "data_sklearn_"+ sklearn_version + '.csv')
# np.savetxt(outfilename, np.array(data_list), delimiter=',', header='Phase,Altitude(ft),Mach,Power_rate,PSFC')


# Verify the data
file_list = os.listdir(pth.join(pth.dirname(__file__), "TP_L1/version_validation"))
# Give the name of the truth data file
base_filename = "data_sklearn_"
truth_data_version = "0_23_2"
truth_data = np.loadtxt(pth.join(pth.dirname(__file__), "TP_L1/version_validation", base_filename+truth_data_version+'.csv'), skiprows=1, delimiter=',')
for file in file_list:
    data = np.loadtxt(pth.join(pth.dirname(__file__), "TP_L1/version_validation", file), skiprows=1, delimiter=',')
    if np.any(data-truth_data):
        print('The file version {0} is not compatible with sklearn version {1}'.format(file, truth_data_version))
        break
    else:
        print('The file version {0} is compatible with sklearn version {1}'.format(file, truth_data_version))

# If compatbility is ensured, run the following for upgrading models to latest sklearn version
print("Compatibility is ensured, should the models be rewritten with sklearn version {} (y/n)?".format(sklearn_version))
user_answer = input()
if user_answer=='y':

    model_list = os.listdir(pth.join(pth.dirname(__file__), "TP_L1/Metamodels"))
    for model in model_list:
        loaded_model = joblib.load(open(pth.join(pth.dirname(__file__), "TP_L1/Metamodels", model), 'rb'))
        joblib.dump(loaded_model,pth.join(pth.dirname(__file__), "TP_L1/Metamodels", model))
    print('Model replaced')

else:
    print("Stopping")