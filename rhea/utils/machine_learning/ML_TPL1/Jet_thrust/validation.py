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

ytest= data.Ratio.values
y= data.Ratio_predicted.values

fig = plt.figure()
plt.plot(ytest, ytest, '-', label='$Ratio_{true}$')
plt.plot(ytest, y, 'r.', label='$\hat{Ratio}$')
   
plt.xlabel('$Ratio_{true}$')
plt.ylabel('$\hat{Ratio}$')

plt.legend(loc='upper left')
#plt.title('Kriging model ff: validation of the prediction model')           
plt.savefig('TP_L1 FR_T_prop ratio validation'+'.pdf')


