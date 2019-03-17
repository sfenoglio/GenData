#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## @package separarKaryo
# Archivo que define y utiliza las funciones para extraer los cromosomas de los cariogramas para la posterior generación de datos

"""
Created on Tue Oct  9 10:38:22 2018

@author: sebastian

MODIFICADO PARA QUE SEPARE TODAS LOS CARIOGRAMAS DISPONIBLES, VERIFICANDO QUE TENGA 46 CROMOSOMAS Y MEZCLANDO EL ORDEN
EN QUE VIENEN DADOS.

"""

import numpy as np
import cv2 as cv
import preprocesamiento as pre
import os
from random import randint
from random import shuffle
from operator import itemgetter, attrgetter


#%%
## Función que, dada dos listas de contornos, compara cuál de las dos está más arriba y a la izquierda tomando como referecia el punto más inferior y a la derecha de cada componente conexa.
# Se utiliza para ordenar espacialmente los contornos obtenidos del cariograma de arriba hacia abajo y de izquierda a derecha con la función 'sorted' que viene integrada en Python.
#@param c1 Lista de puntos obtenidos con la función 'findContours()' de OpenCV: es un arreglo de enteros de 3 dimensiones.
#@param c2 Lista de puntos obtenidos con la función 'findContours()' de OpenCV: es un arreglo de enteros de 3 dimensiones.
#@return Un entero que es 0 si son iguales, mayor a 0 si 'c1' es mayor y menor a 0 si 'c2' es mayor.
def compare(c1,c2):
    #maximos
    y1= max(c1[:,0,1])
    y2= max(c2[:,0,1])
    #tolerancia 
    if(abs(y1-y2)>20):
        return y1 - y2
    else:
        x1= min(c1[:,0,0])
        x2= min(c2[:,0,0])
        return x1 - x2

## Función que transforma la función de comparación en una 'key' aceptada por la función 'sorted' que viene por defecto en Python.
#@param mycmp Función de comparación.
#@return Clase compatible con el parámetro 'key' de 'sorted'.
def cmp_to_key(mycmp):
    'Convert a cmp= function into a key= function'
    class K:
        def __init__(self, obj, *args):
            self.obj = obj
        def __lt__(self, other):
            return mycmp(self.obj, other.obj) < 0
        def __gt__(self, other):
            return mycmp(self.obj, other.obj) > 0
        def __eq__(self, other):
            return mycmp(self.obj, other.obj) == 0
        def __le__(self, other):
            return mycmp(self.obj, other.obj) <= 0
        def __ge__(self, other):
            return mycmp(self.obj, other.obj) >= 0
        def __ne__(self, other):
            return mycmp(self.obj, other.obj) != 0
    return K

#%% 
## Función que dada una imagen que es un cariograma con fondo blanco, guarda una subimagen de cada cromosoma 
# en salida. Les agrega numero y extension '.tiff' para evitar repetidos.
# Además las ordena de acuerdo a su posicion espacial, empezando por la esquina superior izquierda y continuando
# hacia la derecha.
# Para ello, aplica un umbral de 254 sabiendo que el fondo de los mismos es blanco.
# A la máscara obtenida, se rellenan los agujeros con la función 'rellenoAgujeros' del módulo de 'preprocesamiento' y se detectan las componentes conexas con 'findContours()' de OpenCV.
# Para cada componente conexa, si es mayor al tamaño indicado 'minTam', la imagen se agrega a la lista que se devuelve posteriormente.
#@param img Cadena de texto que indica el directorio del cariograma.
#@return Lista de imágenes que contienen los cromosomas extraídos del cariograma.
def dividirCariograma(img,minTam=200):
    #1. cargar imagen
    data= cv.imread(img,0)
    #2. sacar fondo que es todo 255 umbralizando
    _,umbralizada= cv.threshold(data,254,255,cv.THRESH_BINARY_INV)
    #3. rellenar agujeros que quedan por haber pixeles 255 dentro de los cromosomas
    mask= pre.rellenoAgujeros(umbralizada)
    #4. contornos
    _, contours, _= cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    #5. ordenar
    contours= sorted(contours, key= cmp_to_key(compare))
    #6. subimagenes
    separadas= pre.dividirROIs(data,mask,contours)
    #7- tamaño minimo
    salida= []
    for imagen in separadas:
        H,W= imagen.shape
        if(H*W>minTam):
            salida.append(imagen)
    #8. devolver
    return salida
    

#%% guardar
## Función que se encarga de guardar los cromosomas extraídos en la carpeta de salida.
# Se guardan de la forma "cco.png", donde 'cc' es la clase del cromosoma y 'o' es igual a 'a' o a 'b', para guardar las dos ocurrencias de cada clase en el cariograma.
# Se utiliza la función 'imwrite()' de OpenCV.
#@param separados Lista de imágenes de dos dimensiones con formato arreglo de NumPy.
#@param dirSalida Cadena de texto que indica la carpeta de salida.
def guardarSeparados(separados,dirSalida):
    contador= 1;
    estado= "a"
    for imagen in separados:
        cv.imwrite(dirSalida+str(contador).zfill(2)+estado+".tiff",imagen)
        if(estado=="a"):
            estado= "b"
        else:
            estado= "a"
            contador= contador+1    


#%%
## Funcion que toma tantos cariogramas como se le indique para extraer los cromosomas individuales.
# Primero se obtienen todos los archivos del directorio pasado como parámetro y luego se van leyendo una a una en un bucle.
# Para cada una se crea una carpeta dentro del directorio de salida con nombre "rawN", donde N es el número de cariograma a procesar.
# Luego se extraen los cromosomas con 'dividirCariograma()' y se guardan con 'guardarSeparados()' si es que tienen la cantidad de cromosomas indicada en 'cuantos'.
# Además, en la carpeta de salida se guarda el cariograma original para saber de donde provienen los cromosomas extraídos.
#@param directorio Cadena de texto que indica la carpeta en la que se encuentran los cariogramas.
#@param cuantos Cantidad de cromosomas que debe tener el cariograma para procesarlos. Usado para obtener sólo los que poseían 46 cromosomas.
#@param salida Cadena de texto que indica la carpeta de salida.
def procesarCariogramas(directorio,cuantos,salida):
    #1. obtengo imagenes de la carpeta
    imagenes= os.listdir(directorio)
    total= len(imagenes)-1
    
    #1,5 las mezclo por las dudas
    shuffle(imagenes)
    
    #2. las recorro
    contador= 1
    for i,img in enumerate(imagenes):
        
        #2.1 definir directorio de salida
        dirSalida= salida + "raw" + str(contador) +'/' #para sacar el .tiff

        
        #2.2 dividir esa imagen
        separados= dividirCariograma(directorio+img)
        
        #2.3 si cumple condicion
        if(len(separados)==cuantos):
            #2.3.1 crear directorio
            try:
                os.stat(dirSalida)
            except:
                os.mkdir(dirSalida)
            
            #2.3.2 guardar
            guardarSeparados(separados,dirSalida)
        
            #2.3.3 guardar ese cariograma ahi
            cv.imwrite(dirSalida+img,cv.imread(directorio+img,0))
            
            #2.3.4 actualizar contador si es que guardo
            contador += 1
            
        #2.4 ver progreso
        print(i/total)
        
        
#%% Ejemplo que procesa todos los cariogramas
procesarCariogramas("../karyograms_pki-3_612/",46,"./")
