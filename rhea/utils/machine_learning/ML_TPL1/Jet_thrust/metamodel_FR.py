# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 13:50:40 2020

@author: LA202059
"""


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import pickle



###########
#Import experimental polars as Dataframe
#############
data_ref= pd.read_excel("C:/Users/LA202059/Desktop/RHEA/rhea/resources/gasturbine/TP_L1/Datapacks/jet_thrust.csv")

############
#Interpolate Polars for all Reynolds numbers (Altitudes) using a linear model
#############
LR = LinearRegression()
poly = PolynomialFeatures(degree=3)
x_train_pol=poly.fit_transform(data_ref.drop('Newton',axis=1).values)
LR.fit(x_train_pol, data_ref.Newton.values)
FR_predicted=LR.predict(x_train_pol)
data_ref['FR_predicted']=FR_predicted

# save the model to disk
filename = 'PW100_FR_3rd_degree.sav'
pickle.dump(LR, open(filename, 'wb'))
 
# some time later...

# load the model from disk
'''loaded_model = pickle.load(open(filename, 'rb'))
result = loaded_model.score(X_test, Y_test)
print(result)'''
ytest= data_ref.Newton.values
y= data_ref.FR_predicted.values

fig = plt.figure()
plt.plot(ytest, ytest, '-', label='$y_{true}$')
plt.plot(ytest, y, 'r.', label='$\hat{y}$')
   
plt.xlabel('$y_{true}$')
plt.ylabel('$\hat{y}$')

plt.legend(loc='upper left')



