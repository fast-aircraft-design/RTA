# -*- coding: utf-8 -*-
"""
Created on Thu Jul  2 17:58:27 2020

@author: LA202059
"""




from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from sklearn.linear_model import LinearRegression
import random
from sklearn.model_selection import cross_val_score
from sklearn.preprocessing import PolynomialFeatures
import pickle
from scipy import constants
###################################################
#Visualize data in 4D
###################################################
file_name = "C:/Users/LA202059/Desktop/RHEA/rhea/resources/gasturbine/TP_L1/TP_L1_deck_RTO.txt"
data = pd.read_csv(file_name, sep="\t")
data['P_lapse']= data.SHP/2684.166655
data.drop(['PSFC','Throttle','SHP'],axis=1,inplace=True)
n_rows= data.shape[0]
indeces_train_test= random.sample(range(n_rows),int(0.75*n_rows))
indeces_val=[i for i in range(n_rows)if i not in indeces_train_test]
df_val=data.iloc[indeces_val]
df_train_test = data.iloc[indeces_train_test]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = data.Altitude.values
y = data.Mach.values
z = data.P_lapse.values
#c1=data.ETA.values
img = ax.scatter(x, y, z)
#fig.colorbar(img)
ax.set_xlabel('Altitude (ft)')
ax.set_ylabel('Mach')
ax.set_zlabel('P_lapse')

plt.show()
fig.savefig('01.png')


###################################################
#Polinomial REGRESSION
###################################################

####################PLOT LEARNING CURVES##############################
samples=[30,40,50,60,70,80]
learning_curve_list=[]
plot_df=pd.DataFrame(columns=['sample size','sample number','degree','scores', 'mean score'])
i=0
n_times=10

for sample in samples:  #learning curve
    x=0
    while x <n_times:
        n_rows = df_train_test.shape[0]
        indeces_train_test=random.sample(range(n_rows),sample ) 
        df_train_test_sample= df_train_test.iloc[indeces_train_test]   
        

        n_degree=[1,2,3,4,5,6]
        Bias_variance_list=[]
        for n in n_degree: #bias variance dilemma
            poly = PolynomialFeatures(degree=n)
            LR = LinearRegression() 
            x_train_test_pol=poly.fit_transform(df_train_test_sample.drop('P_lapse',axis=1).values)

            
            scores=cross_val_score(LR,x_train_test_pol, df_train_test_sample.P_lapse.values,cv=10)            
            Bias_variance_list.append((n,np.mean(scores)))

            plot_df.loc[i]= [sample,x,n,scores,np.mean(scores)]
            i+=1
        learning_curve_list.append((sample,x,Bias_variance_list))
        x+=1
       
#plot_df.to_csv('plot_df.csv')

fig, axs = plt.subplots(2,3, figsize=(10, 10), facecolor='w', edgecolor='k')
fig.subplots_adjust(hspace = .5, wspace=.2)
fig.suptitle('Bias Variance Dilemma', fontsize=16)
fig1, axs1 = plt.subplots(2,3, figsize=(10, 10), facecolor='w', edgecolor='k')
fig1.subplots_adjust(hspace = .5, wspace=.2)
fig1.suptitle('Learning curves', fontsize=16)
axs = axs.ravel()
axs1=axs1.ravel()
for i in range(6):
    degree=i+1
    sample=samples[i]
    axs[i].scatter(plot_df[plot_df['sample size']==sample].degree.values, plot_df[plot_df['sample size']==sample]['mean score'].values)
    axs[i].set_title('sample size'+str(sample)) 
    axs[i].set_ylim(0., 1) 
    axs[i].set_ylabel('R2 score') 
    axs[i].set_xlabel('degree') 
    axs1[i].scatter(plot_df[plot_df['degree']==degree]['sample size'].values, plot_df[plot_df['degree']==degree]['mean score'].values)
    axs1[i].set_title('degree'+str(degree))
    axs1[i].set_ylim(0., 1) 
    axs1[i].set_ylabel('R2 score') 
    axs1[i].set_xlabel('sample size') 
fig.savefig('02.png')
fig1.savefig('03.png')
#################### Validation SCORE USING ALL AVAILABLE DATA##############################
#CONCLUSION:degree 3 seems to be the best. 
#Regarding the learning curves, 300 points seems to be already enough.
#However, we will use all the points we have.
LR = LinearRegression()
poly = PolynomialFeatures(degree=3)
x_train_pol=poly.fit_transform(df_train_test.drop('P_lapse',axis=1).values)
x_val_pol=poly.fit_transform(df_val.drop('P_lapse',axis=1).values)
LR.fit(x_train_pol, df_train_test.P_lapse.values)
P_lapse_predicted=LR.predict(x_val_pol)
deltas=[]
for i in range(len(P_lapse_predicted)):
    deltas.append((df_val.P_lapse.values[i] -P_lapse_predicted[i] )/df_val.P_lapse.values[i]*100)
    
val_score=LR.score(x_val_pol, df_val.P_lapse.values)
print('Validation score is:',val_score)

training_score= LR.score(x_train_pol, df_train_test.P_lapse.values)
print('Training score is:',training_score)

df_val_results = df_val
df_val_results['P_lapse_predicted']=P_lapse_predicted
df_val_results['delta %']=(df_val_results.P_lapse-df_val_results.P_lapse_predicted)/df_val_results.P_lapse*100
df_val_results.to_csv('validation.csv')

# save the model to disk
filename = 'TP_L1_RTO_P_lapse.sav'
pickle.dump(LR, open(filename, 'wb'))
 
# some time later...'''

# load the model from disk
'''loaded_model = pickle.load(open(filename, 'rb'))
result = loaded_model.score(X_test, Y_test)
print(result)'''
