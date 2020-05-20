
# -*- coding: utf-8 -*-

#######################################
# Created on May 2020                 #
#                                     #
# @author: olivier.moustier           #
#######################################

##############################################################################
# External function importation :

from os import chdir
chdir("C:/Users/olivier.moustier/Documents/Python Scripts") #fixe le répertoire courant
from tkinter import * 
import serial
import pickle  
import threading

serialBufferSize = 256

##############################################################################
# Local function definition :
   
def savingData() :
    "Acquisition et enregistrement en fichier binaire"
    def run():
        "Méthode run pour threading"
        ser=serial.Serial('COM21')
        ser.flushInput() 
        print("run")
        while switch:
            #mainWindow.mainloop()
            if switch == False:
                break
            try:
                ser_bytes=ser.read(serialBufferSize)
                print("Read data from Arduino DUE")
                print(ser_bytes)
                f = open("Fichier_Test_ADC", "ab")
                pickle.dump(ser_bytes, f)
                print("Save data from Arduino DUE")
                f.close()    
            except:
                print("Error")
                break
            
    thread = threading.Thread(target=run, daemon=True)  
    thread.start()
        
       
def switchOn():   
    "Démarrage de l'acquisition"
    global switch  
    switch = True  
    print ("Data saving on")  
    savingData()    
        
def switchOff():   
    "Arrêt de l'acquisition"
    print ("Data saving off") 
    global switch  
    switch = False     
    
def quitter():
    "Quitter le programme"
    # destruction (fermeture) de la fenêtre aprés sortie du réceptionnaire d'événements
    mainWindow.destroy()
    
    
##############################################################################
# Main code body :
    
# création de la fenêtre principale
mainWindow=Tk()

titre = Label(mainWindow, text='ADC7656 data saving', fg='red') 
titre.pack() 

quitter = Button(mainWindow,text='Quit',command=quitter) 
quitter.pack(side=BOTTOM, padx =100, pady =3)

stop=Button(mainWindow,text='Stop',command=switchOff)
stop.pack(side=BOTTOM, padx =100, pady =3)

start=Button(mainWindow,text='Start',command=switchOn)
start.pack(side=BOTTOM, padx =100, pady =3)

       
# démarrage du réceptionnaire d’événements
mainWindow.mainloop()

            
##############################################################################
 
