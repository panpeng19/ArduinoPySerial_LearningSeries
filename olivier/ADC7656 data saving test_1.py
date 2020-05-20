
# -*- coding: utf-8 -*-

#######################################
# Created on May 2020                 #
#                                     #
# @author: olivier.moustier           #
#######################################


##############################################################################
# External function importation :

from os import chdir
from tkinter import * 
import serial
import pickle  
import threading


##############################################################################
######### Variables modifiables par l'utilisateur ici
# taille du buffer de lecture usb
serialBufferSize = 256
# numéro de port de la carte Arduino DUE
portNumber = 21
# chemin du répertoire courant par défaut
savePath = 'C:/Users/olivier.moustier/Documents/Python Scripts'


##############################################################################
######### Variables systèmes
# compteur de sauvegarde depuis le démarrage du programme
saveCounter = 0


##############################################################################
# Local function definition :
   
def savingData() :
    "Acquisition et enregistrement en fichier binaire"
    def run():
        "Méthode run pour threading"
        ser=serial.Serial('COM' + str(portNumber))
        ser.flushInput() 
       #print("run")
        while switch:
            #mainWindow.mainloop()
            if switch == False:
                break
            # code de Lionel : lecture, désérialisation et enregistrement
            try:
                ser_bytes=ser.read(serialBufferSize)
                #print("Read data from Arduino DUE")
                #print(ser_bytes)
                f = open("ADC7656_" + str(saveCounter), "ab")
                pickle.dump(ser_bytes, f)
                #print("Save data from Arduino DUE")
                f.close()    
            except:
                print("Error")
                break
    # création d'un thread pour réger la lecture du port USB et l'enregistrement en fichier binaire      
    thread = threading.Thread(target=run, daemon=True)  
    thread.start()
        
       
def switchOn():   
    "Démarrage de l'acquisition"
    #print ("Data saving on")
    # change la valeur de la variable globale switch
    global switch  
    switch = True  
    # incrémente le compteur de sauvegarde
    global saveCounter
    saveCounter += 1      
    savingData()    
        
def switchOff():   
    "Arrêt de l'acquisition"
    #print ("Data saving off") 
    # change la valeur de la variable globale switch
    global switch  
    switch = False  
    
    
def quitter():
    "Quitter le programme"
    # destruction (fermeture) de la fenêtre aprés sortie du réceptionnaire d'événements
    mainWindow.destroy()
    
def pathSave(event):  
    "Sauvegarde le nouveau chemin du répertoire courant"
    # récupère le nouveau chemin à la pression de la touche Enter (event)
    savePath = chemin.get()
    # affecte le nouveau chemin comme répertoire courant
    chdir(savePath)
    
    
    
##############################################################################
# Main code body :
    
# création de la fenêtre principale
mainWindow=Tk()
mainWindow.title("ADC7656 DATA SAVING")

# création et placement du bouton quitter
quitter = Button(mainWindow,text='Quit',command=quitter) 
quitter.pack(side=BOTTOM, padx =100, pady =3)

# création et placement du bouton stop sauvegarde
stop=Button(mainWindow,text='Stop',command=switchOff)
stop.pack(side=BOTTOM, padx =100, pady =3)

# création et placement du bouton start sauvegarde
start=Button(mainWindow,text='Start',command=switchOn)
start.pack(side=BOTTOM, padx =100, pady =3)

# création et placement du titre du champs d'entré texte
titre1 = Label(mainWindow, text='Saving path :')
titre1.pack()

# création et placement du champ d'entré texte pour changer le chemin du répertoire courant
chemin = Entry(mainWindow, width = 50)
chemin.bind("<Return>", pathSave)
chemin.delete(0, END) 
chemin.insert(0, savePath)
chemin.pack()

#fixe le répertoire courant
chdir(savePath)

# démarrage du réceptionnaire d’événements
mainWindow.mainloop()

            
##############################################################################
 
