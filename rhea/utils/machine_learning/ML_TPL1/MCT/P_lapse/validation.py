# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 10:16:44 2020

@author: LA202059
"""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os 

data=pd.read_csv('validation.csv')

ytest= data.P_lapse.values
y= data.P_lapse_predicted.values

fig = plt.figure()
plt.plot(ytest, ytest, '-', label='$P_lapse_{true}$')
plt.plot(ytest, y, 'r.', label='$\hat{P_lapse}$')
   
plt.xlabel('$P_lapse_{true}$')
plt.ylabel('$\hat{P_lapse}$')

plt.legend(loc='upper left')
#plt.title('Kriging model ff: validation of the prediction model')           
plt.savefig('TP_L1 MCT P_lapse validation'+'.pdf')


