# -*- coding: utf-8 -*-
"""
Created on Tue Jul 27 16:46:28 2021

@author: LA202059
"""
import os
pth =os.path

def generate_configuration_file(SIZING_FOLDER_PATH,CONFIGURATION_FILE):


    a_file = open(CONFIGURATION_FILE, "r")
    list_of_lines = a_file.readlines()
    idx=0
    for line in list_of_lines:    
        if 'workdir' in line:        
            list_of_lines[idx]=list_of_lines[idx].replace('workdir/',SIZING_FOLDER_PATH+'/') 
        idx+=1
    a_file.close()
    conf = pth.split(CONFIGURATION_FILE)[1]
    a_file = open(pth.join(SIZING_FOLDER_PATH,conf), "w")
    a_file.writelines(list_of_lines)
    a_file.close()
    return pth.join(SIZING_FOLDER_PATH,conf)
    
def perform_DOC_mission(pth):
    import os
    from fastoad import api


    # some handy functions to use along widgets


    matricule= input("Your matricule: ")
    folder_name= input("Choose folder name (e.g. tutorial): ")
    aircraft= input("Choose aircraft (ATR72_NEO or ATR72_baseline): ")

    results_folder=aircraft+'_'+folder_name
    #pth =os.path


    DATA_FOLDER_PATH = 'data'
    os.mkdir('workdir/'+results_folder)
    os.mkdir('workdir/'+results_folder+'/sizing')
    os.mkdir('workdir/'+results_folder+'/DOC')
    
    WORK_FOLDER_PATH = 'workdir'
    SIZING_FOLDER_PATH = 'workdir'+'/'+results_folder +'/'+'sizing' 
    DOC_FOLDER_PATH = 'workdir'+'/'+results_folder +'/'+'DOC'
    DATA_FOLDER_PATH = 'data'
    pth =os.path
    
    if aircraft=='ATR72_NEO':
        SOURCE_FILE = pth.join(DATA_FOLDER_PATH, 'ATR72_NEO.xml') 
        change_matricule(pth.join(WORK_FOLDER_PATH, 'ATR_NEO_DOC_mission.toml'),matricule)
        CONFIGURATION_FILE = generate_configuration_file(DOC_FOLDER_PATH, pth.join(WORK_FOLDER_PATH, 'ATR_NEO_DOC_mission.toml'))
        
        api.generate_inputs(CONFIGURATION_FILE, SOURCE_FILE, overwrite=True)
    elif aircraft=='ATR72_baseline':
        SOURCE_FILE = pth.join(DATA_FOLDER_PATH, 'ATR72_baseline.xml') 
        change_matricule(pth.join(WORK_FOLDER_PATH, 'ATR_DOC_mission.toml'),matricule)
        CONFIGURATION_FILE = generate_configuration_file(DOC_FOLDER_PATH, pth.join(WORK_FOLDER_PATH, 'ATR_DOC_mission.toml'))
        api.generate_inputs(CONFIGURATION_FILE, SOURCE_FILE, overwrite=True)    
    return SOURCE_FILE, DOC_FOLDER_PATH


def change_matricule(CONFIGURATION_FILE,matricule):
    a_file = open(CONFIGURATION_FILE, "r")
    list_of_lines = a_file.readlines()
    idx=0
    for line in list_of_lines:    
        if 'LA202059' in line:        
            list_of_lines[idx]=list_of_lines[idx].replace('LA202059',matricule) 
        idx+=1
    a_file.close()
    a_file = open(CONFIGURATION_FILE, "w")
    a_file.writelines(list_of_lines)
    a_file.close()