#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## @package generarAux
#  Archivo que incluye funciones auxiliares utilizadas por 'generar.py' tales como rotación, recortar, agregar cromosoma, guardar imágenes, etc.

"""
Created on Tue Sep 25 20:47:13 2018

@author: sebastian

Objetivo: funciones auxiliares de generar.py que pueden ser reutilizadas.
"""

#%%
import numpy as np
import cv2 as cv
from random import randint
from skimage.transform import rotate
import preprocesamiento as pre


#%% devuelve -1 si no puede rotarlo sin superar el tamanio maximo de la imagen. Si maxTam es (0,0) no verifica
## Función que dada una imagen y su mascara, los rota una cantidad de angulos dada la función 'rotate()' de scikit-image, rellenando con blanco los píxeles extra de la imagen y con negro los de las máscaras.
# Luego aplica una apertura morfológica en las máscaras ya que la rotación provoca una interpolación en los bordes de la imagen que produce una difuminación del mismo.
# Así, estas máscaras se aplica en la imagen para evitar dicha interpolación.
# Por último se recorta la imagen con 'recortar()' porque la rotación puede agrandar la imagen por demás.
# Antes de devolverla, se controla que al rotar la imagen no supere el tamaño máximo especificado si no es (0,0).
#@param img Imagen 2D en formato arreglo de NumPy.
#@param mascara Imagen binaria 2D en formato arreglo de NumPy o lista que contenga imágenes de ese tipo, ya que pueden ser las máscaras de los cromosomas que se fueron agregando.
#@param angle Ángulo que se giran las imágenes.
#@param maxTam Tupla que indica el tamaño máximo que puede tener la imagen. Si es (0,0) no se controla.
#@return Tupla con imagen y máscara/lista de máscaras 2D en formato de arreglo de NumPy. Devuelve -1 si la imagen rotada es mayor a 'maxTam'.
def rotar(img,mascara,angle,maxTam=(0,0)):
    data= img.copy()
    mask= mascara.copy()
    
    #1. roto, verificando que mascara no sea una lista
    data= rotate(data,angle,resize=True,mode="constant",cval=255,order=1,preserve_range=True).astype(np.uint8)
    if(type(mask)==list):
        #roto cada mascara de la lista
#        mcombinadas= np.zeros_like(mascara[0])
        for i in range(len(mask)):
#            mcombinadas += mask[i]
            mask[i]= rotate(mask[i],angle,resize=True,mode="constant",cval=0,order=1,preserve_range=True).astype(np.uint8)
#        mcombinadas= rotate(mcombinadas,angle,resize=True,mode="constant",cval=0,order=1,preserve_range=True).astype(np.uint8)
    else:
        mask= rotate(mask,angle,resize=True,mode="constant",cval=0,order=1,preserve_range=True).astype(np.uint8)
    
    #2. elimino de la mascara los que no son 255 y la erosiono -------------> Y APLICO MASCARA
    ee= (np.ones((3,3))*255).astype(np.uint8)
    if(type(mask)==list):
        #cuando es lista no erosiono nada y todo el que sea mayor a 0 es mascara
        # hago cierre para arreglar un poco los bordes
        for i in range(len(mask)):
            mask[i][mask[i]>0]= 255
            mask[i]= cv.morphologyEx(mask[i], cv.MORPH_CLOSE, ee)
            #y le saco todo lo que pueda tener de las mascaras anteriores
            enComun= np.zeros_like(mask[i])
            for j in range(i-1,-1,-1):
                enComun += cv.bitwise_and(mask[i], mask[j])
            mask[i] = mask[i] - enComun
        #las aplico
        maskAux= np.zeros_like(data)
        for m in mask:
            maskAux+= m
        data= cv.bitwise_and(data,maskAux) +(255-maskAux)
    else:
        mask[mask<255]= 0
        ee= cv.getStructuringElement(cv.MORPH_CROSS,(3,3))*255
#        mask= cv.erode(mask,ee,iterations=1)
        mask= cv.morphologyEx(mask,cv.MORPH_OPEN,ee)
        data= cv.bitwise_and(mask,data) + (255-mask)
        
    
    #2,5 recortar
    data,mask= recortar(data,mask,margen=4)
    if(cv.countNonZero(data)==0):
        ver=1

    #3. verificar segun maxTam
    H,W= data.shape
    if(maxTam!=(0,0) and (maxTam[0]<H or maxTam[1]<W)):
        return (-1,-1)
    
    #4. devuelvo como tuple
    return (data,mask)


#%% si despl es -1: se desplaza al azar segun las posibilidades.si despl es -2: lo centra
## Función que redimensiona una imagen y su máscara, dando la posibilidad de desplazar la original dentro de la nueva más grande.
# Primero se genera una imagen con color de fondo igual a 'fondo' y máscaras de fondo negro.
# Luego, según 'despl' se genera desplazamientos al azar (-1), se centra (-2) o se aplica (si fuera una tupla que indique el desplazamiento en x y en y).
# Finalmente se aplica dicho desplazamiento.
#@param img Imagen 2D en formato arreglo de NumPy.
#@param mascara Imagen binaria 2D en formato arreglo de NumPy o lista que contenga imágenes de ese tipo, ya que pueden ser las máscaras de los cromosomas que se fueron agregando.
#@param tamImagenSalida Tupla que indica el tamaño que debe tener la imagen de salida.
#@param despl Tupla que indica el desplazamiento en x y en y. Si es un entero igual a -1, se calcula un desplazamiento al azar que no supere el tamaño de la imagen. Si es -2, la imagen original se centra dentro el nuevo lienzo.
#@param fondo Color de fondo con el que se rellena la imagen redimensionada.
#@return Tupla con imagen y máscara/lista de máscaras 2D en formato de arreglo de NumPy. 
def desplazarResize(img,mascara,tamImagenSalida,despl=-1,fondo=255):
    #1. inicializo
    salida= (fondo*np.ones(tamImagenSalida)).astype(np.uint8)
    H,W= img.shape
    if(type(mascara)==list):
        msalida= []
        for _ in range(len(mascara)):
            msalida.append(np.zeros(tamImagenSalida,dtype= np.uint8))
    else:
        msalida= np.zeros(tamImagenSalida,dtype= np.uint8)
    
    #2. calculo desplazamientos
    if(despl==-1):
        despl_h= randint(0,tamImagenSalida[0]-H)
        despl_w= randint(0,tamImagenSalida[1]-W)
    else:
        if(despl==-2):
            despl_h= (tamImagenSalida[0]-H)//2
            despl_w= (tamImagenSalida[1]-W)//2
        else:
            despl_h= despl[0]
            despl_w= despl[1]
    
    #3. los aplico
    salida[despl_h:despl_h+H, despl_w:despl_w+W]= img.copy()
    if(type(mascara)==list):
        for i in range(len(mascara)):
            msalida[i][despl_h:despl_h+H, despl_w:despl_w+W]= mascara[i].copy()
    else:
        msalida[despl_h:despl_h+H, despl_w:despl_w+W]= mascara.copy()
    
    #4. devuelvo como tuple
    return (salida,msalida)


#%%
## Función que calcula la cantidad de end point que tiene una imagen binaria.
# Se utiliza la función 'thinning()' de OpenCV para el calculo del esqueleto, para luego convolucionar con un filtro de 3x3 con todos números 1 y un 10 en el centro.
# Entonces, luego se cuenta los píxeles que resulten con un valor de 11: quiere decir que tienen sólo un vecino.
#@param mask Imagen binaria de dos dimensiones en formato arreglo de NumPy. 
#@return Emtero que indica la cantidad de end points de la máscara.
def cantEP(mask):
    data= mask.copy()
    data= cv.morphologyEx(data, cv.MORPH_CLOSE, np.ones((3,3),dtype=np.uint8)*255)
    esqueleto= cv.ximgproc.thinning(data)
    #filtro
    kernel= np.uint8([[1,1,1], [1,10,1], [1,1,1]])
    filtrada= cv.filter2D(esqueleto/255,-1,kernel)
    #devolver cantidad contando los que son iguales a 11
    return (filtrada==11).sum()


#%% 
## Función que tiene como objetivo adicionar dos clusters de cromosomas teniendo cuidad de que los píxeles de borde queden más suaves.
# Para ello, primero se obtiene el contorno de la zona de solapamiento mediante operaciones morfológicas de OpenCV.
# Luego se suman las dos imágenes de cromosomas 'data1' y 'data2' normalmente.
# A continuación, se le aplica un filtro gaussiano con desvío igual a 'sigmaGauss' en el borde obtenido anteriormente.
#@param data1 Imagen 2D en formato arreglo de NumPy.
#@param mask1 Imagen binaria 2D en formato arreglo de NumPy.    
#@param data2 Imagen 2D en formato arreglo de NumPy, con la zona de solapamiento ya quitada previamente de forma de permitir la suma con 'data1' para generar el solapamiento.
#@param mask2 Imagen binaria 2D en formato arreglo de NumPy, , con la zona de solapamiento ya quitada previamente. 
#@parm sigmagauss Desvío utilizado en el filtro gaussiano de 3x3 para la suavización del borde de la zona de solapamiento.
#@return Tupla con la imagen y una lista con las máscaras 2D en formato de arreglo de NumPy. 
def suavizarPegada(data1,mask1,data2,mask2,sigmaGauss=1.5):
    #1. obtengo borde de zona de solapamiento coincidente con mask1
    # dilatando mask2 y haciendo AND con mask1
    ee= (np.ones((3,3))*255).astype(np.uint8)
    borde= cv.bitwise_and(mask1, cv.dilate(mask2,ee,iterations=1))
    
    #2. suma normal
    salida= cv.bitwise_and(mask1,data1) + cv.bitwise_and(mask2,data2)
    
#    #3. en esa linea queda el minimo
    indices= cv.findNonZero(borde)
#    #3.1 si no hay borde, no hace nada
    if(type(indices)==np.ndarray):
        indices= indices[:,0,:]
#        for w,h in indices:
#            #3.1 si el de atras (data2) era menor que el de adelante (data1/salida)
#            if(data2[h,w]<salida[h,w]):
#                #3.2 edito salida
#                salida[h,w]= data2[h,w]
#                
#                #3.3 edito mascaras
#                mask1[h,w]= 0
#                mask2[h,w]= 255
    
    
    #4. fondo blanco
    salida= salida + (255 - cv.bitwise_or(mask1,mask2))
    
    #5. filtro suavizante en ese borde un poco mas ancho
#    borde= cv.dilate(borde,ee,iterations=1)
    if(type(indices)==np.ndarray):
        suavizada= cv.GaussianBlur(salida,(3,3),sigmaGauss)
        for w,h in indices:
            salida[h,w]= suavizada[h,w]
        
    #6. devuelvo
    return (salida,[mask1,mask2])


#%% img1 queda sobre img2
## Función que agrega a un solapamiento 'img1' otro cromosoma 'img2', quedando 'img2' por detrás de 'img1'.
# Primero se obtiene la máscara del cromosoma a agregar y se rellenan los agujeros con la función 'rellenoAgujeros()' del módulo de 'preprocesamiento'.
# Luego, se itera rotando 'img2' un ángulo al azar y agregándolo al solapamiento hasta que cumpla con las condiciones de porcentaje de solapamiento y cantidad de end points nuevos.
# Si se supera la cantidad máxima de intentos, se devuelve una máscara con todos 255 indicando que se falló y que se debe intentar agregar otro cromosoma.
#@param img1 Imagen 2D en formato arreglo de NumPy correspondiente al solapamiento actual.
#@param mask1 Imagen binaria 2D en formato arreglo de NumPy.    
#@param img2 Imagen 2D en formato arreglo de NumPy correspondiente al cromosoma que se agregará.
#@param porcMinSolap Porcentaje mínimo de solapamiento que debe tener el nuevo cluster medido respecto al cromosoma que se agrega.
#@param porcMaxSolap Porcentaje mínimo de solapamiento que debe tener el nuevo cluster medido respecto al cromosoma que se agrega.
#@param minNewEP Cantidad mínima de end points que debe tener el nuevo cluster.
#@param maxIntentos Cantidad máxima de intentos que se prueba agregar el nuevo cromosoma.
#@return Tupla que contiene la imagen generada y una lista con las máscaras de cada cromosoma. Si se supera la cantidad máxima de intentos, la última máscara de la lista es una imagen con todos los píxeles iguales a 255.
def generarSolap(img1,mascara1,img2,porcMinSolap, porcMaxSolap, minNewEP,maxIntentos=30):
    #1. mascaras
    data1= img1.copy()
    data2= img2.copy()
    mask1= mascara1.copy()
    _,mask2= cv.threshold(data2,254,255,cv.THRESH_BINARY_INV)
    mask2= pre.rellenoAgujeros(mask2)
    
    #2. mientras no se cumplan los porcentajes y los endpoints
    porcSolap= newEP= 0
    oldEP= cantEP(mask1)
    primeraVez= True
    cuenta= 0
    while(porcSolap<porcMinSolap or porcMaxSolap<porcSolap or newEP<minNewEP or primeraVez):
        #print("while generarsolap")
        #2.0 despues del intento maximo, devuelvo mascaras de unos asi falla cuando verifica que no se solapen varios en el mismo lugar
        cuenta+=1
        if(cuenta>maxIntentos):
            return (data1,[mask1, (np.ones_like(data1)*255).astype(np.uint8),
                          (np.ones_like(data1)*255).astype(np.uint8)])
        
        #2.1 roto la segunda un angulo al azar y la desplazo al azar
        angle= randint(0,360)
        data2Auxx,mask2Auxx= rotar(data2,mask2,angle) #otro nombre para que while no la vaya cambiando a cada rato
#        nuevoTam= (int(data2Aux.shape[0]*1.5),int(data2Aux.shape[1]*1.5))
#        data2Aux,mask2Aux= desplazarResize(data2Aux,mask2Aux,nuevoTam)
        
        #2.2 corrijo tamanios para que sean de iguales, desplazandolas hasya que haya algun contacto
        zerosMask1= cv.countNonZero(mask1)
        tamFinal= (int(max(data1.shape[0],data2Auxx.shape[0]*1.6)),int(max(data1.shape[1],data2Auxx.shape[1]*1.6)))
        data1Aux,mask1Aux= desplazarResize(data1,     mask1,     tamFinal, despl=(0,0)) #no hay que mover nunca a la 1
        mask2Aux= np.zeros_like(mask1Aux)
        
        fallo= False
        contador=0
        while(cv.countNonZero(cv.bitwise_and(mask1Aux,mask2Aux))==0):
            #print("while generarsolap2")
            contador+=1
            data2Aux,mask2Aux= desplazarResize(data2Auxx, mask2Auxx, tamFinal)
            #verifico que mask1 no sea solo zeros
            if(zerosMask1==0):
                break
            if(contador>50):
                fallo= True
                break
        
        if(fallo):
            continue
        
        
        #2.3 calculo el porcentaje de solapamiento contando pixeles que no son 0
        zonaSolapamiento= cv.bitwise_and(mask1Aux,mask2Aux)
        zerosMask2Aux= cv.countNonZero(mask2Aux)
        porcSolap= cv.countNonZero(zonaSolapamiento)/(zerosMask2Aux if zerosMask2Aux>0 else 1)
        
        #2.4 calculo de los nuevos end points
        maskCombinadas= cv.bitwise_or(mask1Aux,mask2Aux)
        newEP= cantEP(maskCombinadas) - oldEP
        
        #2.5 saco primeraVez (util para inicializar)
        primeraVez= False
        
        
    
    #-----------------------------EXITO-----------------------------
    
    #3. quedan definitivas
    data1= data1Aux
    mask1= mask1Aux
    data2= data2Aux
    mask2= mask2Aux
    
    #4. a mask2 le saco la zona de solapamiento para poder sumarlas despues aplicando las mascaras
    zonaSolapamiento= cv.bitwise_and(mask1,mask2)
    mask2= mask2 - zonaSolapamiento
    
    #5. suavizar pegada
    salida, masks= suavizarPegada(data1,mask1,data2,mask2)
    
    #6. recortar    
#    salida, masks= recortar(salida,masks,margen=2)
    
    #7. devuelvo como tuple
    masks.append(zonaSolapamiento)
    return (salida,masks)
    
    
#%%
## Función que se encarga de guardar las imágenes generadas como archivo ".npz".
# Primero se definen las matrices de NumPy en las que se almacenan las imágenes generadas y las máscaras. Éstas tienen 4 dimensiones como se utilizan en PyTorch posteriormente: la primera es la cantidad de datos, la segunda la cantidad de canales (siempre 1 en este caso) y las restantes el tamaño de la imagen. Luego se recorren las listas de 'salida' para ir completándolas. 
# Por último se guarda la inversa de las imágenes mediante la función 'savez_compressed' de NumPy puesto que la misma es eficiente comprimiendo matrices sparse. Los datos se guardan bajo la claave ‘data’ y los máscaras bajo la clave ‘maskLineal’.
#@param salida Tupla que contiene tres listas: una con las imágenes de los solapamientos generados, otra que es a su vez una lista con las máscaras correspondientes a cada cromosoma y una tercera que es una lista que contiene el número de clase del mismo.
#@param tamImagen Tupla con el tamaño de la imagen que se guarda.
#@param nombre Nombre del archivo '.npz' que se genera.
#@param folder Carpeta de salida en que se guarda el archivo generado.
def guardar(salida,tamImagen,nombre,folder="./"):
    print("Guardando archivo...")
    #1. inicializo salidas y utilidades
    cantReal= len(salida)
    imagenes= np.zeros((cantReal, 1 ,tamImagen[0],tamImagen[1]),dtype= np.uint8)
    maskLineal= np.zeros((cantReal,tamImagen[0],tamImagen[1]),dtype= np.uint8)
    
    #2. recorro y voy procesando
    for i,(image,masks,indices) in enumerate(salida):
        #2.1 imagenes de salida (inversa asi puedo usar npz despues)
        imagenes[i]= 255 - image
        
        cantCanales= len(masks)
        #2.2 mascaras 
        for c in range(cantCanales):
            
        #2.3 mascaras lineal 
            maskLineal[i ] += (masks[c]/255*(indices[c])).astype(np.uint8)
        
    #3. guardar
    np.savez_compressed(folder+nombre+".npz",
                        data=imagenes,
                        maskLineal=maskLineal)

    print("Archivo guardado.")


#%% 
## Función que dada una imagen y su máscara o lista de máscaras las recorta al mínimo cuadrado que la contiene dejando un margen dado por 'margen'.
# Se comienza buscando el mínimo rectángulo que contiene a los cromosomas de la imagen, sabiendo el color de fondo de la misma, con la función 'boundingRect' de OpenCV.
# Se genera la imagen de salida con color 'fondo' y se copia la información relevande de la imagen original. 
# También se generan máscaras de salida con el mismo rectángulo obtenido anteriormente.
#@param img Imagen 2D en formato arreglo de NumPy.
#@param mascaras Imagen binaria 2D en formato arreglo de NumPy o lista que contenga imágenes de ese tipo, ya que pueden ser las máscaras de los cromosomas que se fueron agregando.
#@param margen Entero que indica el margen que se deja a cada lado del mínimo rectángulo.
#@param fondo Color de fondo que se utiliza en la búsqueda del mínimo rectángulo.
#@return Tupla con imagen y máscara/lista de máscaras 2D en formato de arreglo de NumPy. 
def recortar(img,mascaras,margen,fondo=255):
    #1. inicializo
    
    #2. rectangulo alrededor
    y,x,h,w= cv.boundingRect(cv.findNonZero(fondo-img))
    
    #3. imagen
    salida= (np.ones((w+2*margen,h+2*margen))*fondo).astype(np.uint8)
    salida[margen:margen+w,margen:margen+h]= img[x:x+w,y:y+h].copy()
    
    #4. mascaras
    if(type(mascaras)==list):
        msalida= []
        for m in mascaras:
            # y despues agrego
            aux= np.zeros_like(salida)
            aux[margen:margen+w,margen:margen+h]= m[x:x+w,y:y+h]
            msalida.append(aux.copy())
    else:
        msalida= np.zeros_like(salida)
        msalida[margen:margen+w,margen:margen+h]= mascaras[x:x+w,y:y+h].copy()
    
    #devuelvo
    return (salida,msalida)


#%%
## Función que aplica histogram matching con una gaussiana de media 128 y desvío indicado.
#@param source Imagen o píxeles a los que se aplicará.
#@param sigma Desvío de la gaussiana que se utilizará.
#@return Imagen o píxeles corregidos según la gaussiana.
def histGauss(source, sigma= 60):
    oldshape = source.shape
    source = source.ravel()
    
    from scipy import signal
    t_values= np.arange(0,254).astype(np.uint8)
    t_counts = np.append(signal.gaussian(250, std= sigma), np.zeros((4)))

    # get the set of unique pixel values and their corresponding indices and
    # counts
    s_values, bin_idx, s_counts = np.unique(source, return_inverse=True,
                                            return_counts=True)

    # take the cumsum of the counts and normalize by the number of pixels to
    # get the empirical cumulative distribution functions for the source and
    # template images (maps pixel value --> quantile)
    s_quantiles = np.cumsum(s_counts).astype(np.float64)
    s_quantiles /= s_quantiles[-1]
    t_quantiles = np.cumsum(t_counts).astype(np.float64)
    t_quantiles /= t_quantiles[-1]

    # interpolate linearly to find the pixel values in the template image
    # that correspond most closely to the quantiles in the source image
    interp_t_values = np.interp(s_quantiles, t_quantiles, t_values)

    return interp_t_values[bin_idx].reshape(oldshape)