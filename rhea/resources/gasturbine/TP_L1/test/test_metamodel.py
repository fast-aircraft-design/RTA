# This file tsts the compatibility of scikit-learn version with the metamodels#

from sklearn.preprocessing import PolynomialFeatures
import os.path as pth
import numpy as np
import joblib
from numpy.testing import assert_allclose

def get_metamodel_prediction(phase=1.0,altitude=0.0,mach=0.01, power_rate=1.0):
    filename = pth.join(pth.dirname(__file__), "../Metamodels")
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

def compute_metamodels_data(Phase, Altitude, Mach, Power_rate):
    # Generate data
    data_list = []
    for phase in Phase:
        for alti in Altitude:
            for mach in Mach:
                for power_rate in Power_rate:
                    psfc = get_metamodel_prediction(phase=phase, altitude=alti, mach=mach, power_rate=power_rate)
                    data_list.append([phase, alti, mach, power_rate, float(psfc)])
    return np.array(data_list)

def test_metamodel_compatibility():
    """
    This function test the metamodels compatibility with current scikit-learn version.

    Originally metamodels were created with scikit-learn=0.23.2.

    If compatibility is ensured, the metamodels maybe updated to the latest
    scikit-version to remove the UserWarning.
    If the test fails, the scikit-learn package should be downgraded.

    Use the python script, update_metamodels_version.py for this purpose.

    """
    Altitude = np.linspace(1000, 18001, 3)
    Mach = np.linspace(0.05, 0.51, 3)
    Power_rate = np.linspace(0.01, 1, 3)
    Phase = np.arange(0, 9, 1)

    data = compute_metamodels_data(Phase, Altitude, Mach, Power_rate)

    # The truth data file is the following:
    base_filename = "data_sklearn_"
    truth_data_version = "0_23_2"
    truth_data = np.loadtxt(pth.join(pth.dirname(__file__), "version_validation", base_filename + truth_data_version + '.csv'), skiprows=1, delimiter=',')

    assert_allclose(data, truth_data, rtol=1e-3)
