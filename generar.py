#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## @package generar
# Script que aplica las funciones definidas en 'generarAux.py' para la generación de solapamientos sintéticos a partir de los parámetros definidos en 'generar_params.py'.

"""
Created on Tue Sep 25 20:01:28 2018

@author: sebastian

Objetivo: combinar cromosomas provenientes de los cariogramas, generando para cada combinacion varias
imagenes con desplazamientos y rotaciones.

Salida: archivos .npz de entrenamiento y test por separado:
    - imagenes generadas    ["data"]
    - mascaras en una sola imagen con un entero en cada píxel indicando la clase ["maskLineal"]

"""

#%%
import numpy as np
import cv2 as cv
import generarAux as aux



#%% PARAMETROS
from generar_params import *
#

#%% DATOS
cantidades= []
for r in range(cuantos_raw):        
    #para el for: (nombre, cantidad, [cuantos cromosomas solapados], [carpetas raw])
    cantidades.append(("train"+str(r+1),
                      train_cant,
                      [c for c in range(cant_crom_min,cant_crom_max+1)],
                      [r for r in range(r*intervalo,(r+1)*intervalo)]))
# test
cantidades.append(("test",
                  test_cant,
                  [c for c in range(cant_crom_min,cant_crom_max+1)],
                  [r for r in range(intervalo*cuantos_raw,total_cario)]))



#%% PROC
contador_fallos= 0
blurness= []
blurness_porTipo= [[] for i in range(23)]
total= (np.array([cantidades[i][1]*len(cantidades[i][2])*len(cantidades[i][3]) for i in range(len(cantidades))]).sum())*cant_angles*cantDespl
contador= 0
for nombre,cantidad,cuantos,raw_dir in cantidades:
    salida= []
    for r in raw_dir: #para cada directorio
        for c in cuantos: #para cada cantidad de cromosomas solapados
            for i in range(cantidad): # genero la cantidad deseada
                
                #1. inicializo con algun cromosoma al azar
                numAlAzar= getRandomNum()
                img= cv.imread(directorio +
                               str(r+1) + "/" +
                               str(numAlAzar//2).zfill(2) +
                               ("a" if numAlAzar%2==0 else "b") +
                               ".tiff", 0)
                indices_procesados= [numAlAzar//2]
                
                #%%
                data,maskAux= aux.generarSolap((np.ones_like(img)*255).astype(np.uint8), 
                                               np.zeros_like(img), 
                                               img, porcMinSolap=0, porcMaxSolap=100, minNewEP=0)
                maskAcum= maskAux[1]
                maskSolap= np.zeros_like(maskAcum)
                masksCanales= [maskAux[1]]
                
                
                
                #2. voy agregando hasta generar el solapamiento con los cuantos cromosomas que quiera. -1 porque ya agregue uno
                agregados= 0
                while(agregados < c-1):
                    #print("\n \n while agregando: "+str(agregados) +"\n \n")
                    #2.0 muevo un poco lo que tengo para tener mas posibilidades de solapamiento
                    desplMov= ( randint(0,int(0.4*data.shape[0])-1), randint(0,int(0.4*data.shape[1])-1) )
                    tamMov= (int(data.shape[0]*1.4),int(data.shape[1]*1.4))
                    dataMov, masksCanalesMov= aux.desplazarResize(data, masksCanales,
                                                                 tamImagenSalida= tamMov, despl= desplMov)
                    maskAcumMov, maskSolapMov= aux.desplazarResize(maskAcum, maskSolap, fondo= 0,
                                                                   tamImagenSalida= tamMov, despl= desplMov)
                    
                    #2.1 elijo algun cromosoma al azar de otra clase al anterior
                    numAlAzar= getRandomNum()    
                    while(numAlAzar//2 in indices_procesados):
                        numAlAzar= getRandomNum()
                    img= cv.imread(directorio +
                               str(r+1) + "/" +
                               str(numAlAzar//2).zfill(2) +
                               ("a" if numAlAzar%2==0 else "b") +
                               ".tiff", 0)
                    
                    
                    #2.2 genero un solapamiento al azar con un minimo de solapamiento/endpoints verificando que no toque las zonas de solapamiento anterior
                    #control de intentos maximos
                    intentos= 0
                    excedio= False
                    cantZeros= 1
                    while(cantZeros>0):
                        #print("while intentos: " + str(intentos))
                        intentos+= 1
                        if(intentos>maxIntentos):
                            excedio= True
                            break
                        
                        solapados, msolapados= aux.generarSolap(dataMov, maskAcumMov,img,
                                                                porcMinSolap, porcMaxSolap, minNewEP[randint(0,len(minNewEP)-1)], maxIntentosAux)
                        _,maskAuxSolapMov= aux.desplazarResize(np.zeros_like(maskSolapMov),maskSolapMov,
                                                               msolapados[1].shape, despl= (0,0))
                        #ver si devolvio todos unos
                        if(cv.countNonZero(msolapados[2])==msolapados[2].shape[0]*msolapados[2].shape[1]):
                            cantZeros= 1
                        else:
                            cantZeros= cv.countNonZero(cv.bitwise_and(msolapados[2],maskAuxSolapMov))
                    
                    #ver si se excedio 
                    if(excedio==True):
                        continue
    
                    
                    
                    #-----------------------EXITO--------------------
                    
                    
                    #2.3 cambiar tamaño de todas las mascaras anteriores segun el nuevo tamaño
                    _, masksCanales= aux.desplazarResize(np.zeros_like(dataMov), masksCanalesMov, 
                                                         msolapados[1].shape, despl= (0,0))
                    
                    #2.4 corregir mascara anterior que puede cambiar y actualizo: maskAcum, maskSolap
                    masksCanales[len(masksCanales)-1]= cv.bitwise_and(masksCanales[len(masksCanales)-1], msolapados[0])
                    _, maskAcum= aux.desplazarResize(np.zeros_like(maskAcumMov),maskAcumMov, 
                                                       msolapados[1].shape, despl= (0,0))
                    maskAcum= cv.bitwise_or(maskAcum, msolapados[1])
                    _, maskSolap= aux.desplazarResize(np.zeros_like(maskSolapMov),maskSolapMov, 
                                                       msolapados[1].shape, despl= (0,0))
                    maskSolap += msolapados[2]
                    
                    #2.5 imagen definitiva y agregar la mascara
                    data= solapados.copy()
                    masksCanales.append(msolapados[1].copy())
                    
                    #2.6 actualizo agregados y los indices procesados
                    agregados+= 1
                    indices_procesados.append(numAlAzar//2)
                    
                
                               
                #3. a ese solapamiento lo roto y lo desplazo para cada angulo
                #3.1 defino angulos al azar segun la cantidad
                step= 360//cant_angles
                angles= [randint(i*step,(i+1)*step) for i in range(0,cant_angles)]
                #3.2recorto antes de empezar
                cortado, mcortado= aux.recortar(data,masksCanales,margen=4)
                
                for angle in angles:
                    for _ in range(cantDespl):
                        #3.1 rotar
                        rotado, mrotado= aux.rotar(cortado, mcortado, angle,maxTam=tamImagen)
                        
                        #3.2 desplazo + llevo a todos al tamaño de la imagen
                        if(type(rotado)==np.ndarray):
                            sigma,std= getRandomBlurAndStd()
#%%
                            #2.1,5 NORMALIZAR
                            macumm= mrotado[0].copy()
                            for m in mrotado:
                                macumm= cv.bitwise_or(macumm,m)
                            indImg= macumm==255
                            rotado[indImg]= aux.histGauss(rotado[indImg], std)
                            
                            #blurness_porTipo[numAlAzar//2].append(cv.Laplacian(img, -1).var())
                        #%%

                            #BLUR
                            rotado= cv.GaussianBlur(rotado,(3,3), sigma)

#%%

                            #solo si pudo rotar sin superar el tamanio maximo
                            salida.append(aux.desplazarResize(rotado,mrotado,tamImagen,despl=-2) + (indices_procesados,))
                            blurness.append( cv.Laplacian(rotado, -1).var() )
                            
                        else:
                            contador_fallos +=1
                            
                        #3.3 contador
                        contador += 1
                        print("Porcentaje: {:.4f}%, Blurness: {:.4f}".format(contador/total*100, blurness[len(blurness)-1]))
                        
    #4. guardo lo generado
    aux.guardar(salida,tamImagen,nombre,folder=out_path)
    
#%% prueba leer
#loaded= np.load("./test.npz")
#data= loaded['data']
#bordes= loaded['maskBordes']
#mascaras= loaded['maskCanales']
#maskLineal= loaded['maskLineal']
#for i in [0,500, 999]:
#    util.verImagen([255-data[i,0],bordes[i,0], cv.GaussianBlur(255-data[i,0],(3,3),1), cv.GaussianBlur(255-data[i,0],(3,3), .7), cv.GaussianBlur#(255-data[i,0],(3,3),.4),
#                    cv.normalize(maskLineal[i],np.zeros_like(maskLineal[i]),255,0,cv.NORM_MINMAX)])
    
