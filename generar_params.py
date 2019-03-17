## @package generar_params.py
# Script que define los parámetros que se utilizan para la generación de los datos.
# Los parametros que trae por defecto son los correspondientes al dataset Gen5todosKaryoNormVar.

#path de cromosomas separados
directorio= "./Separados/raw" #+str(r+1)+"/"
out_path= "./salida/"

# cuantos intervalos de carpetas de cariogramas hay y de cuanto es ese intervalo
total_cario= 536 #total de cariogramas
cuantos_raw= 12 #cuantos archivos de salida habra
intervalo= 40 #cuantos 

#cantidades
tamImagen= (256,256)
cant_angles= 4 #cantidad de angulos que se gira
cantDespl= 1 #cantidad de desplazamientos por cada angulo
train_cant= 10 #cantidad de solapamientos por cariograma y por cantidad de cromosomas en para train
test_cant= 10 #cantidad de solapamientos por cariograma y por cantidad de cromosomas en para test
cant_crom_min= 2 #cantidad minima de cromosomas solapados
cant_crom_max= 5 #cantidad maxima de cromosomas solapados


#detalles
porcMinSolap= 0.01  # % respecto al nuevo
porcMaxSolap= 0.27  # % respecto al nuevo
minNewEP= [1,2]         # end points nuevos por solapamiento. Se elige al azar entre ellos
maxIntentos= 20     # para agregar nuevo cromosoma, sino cambia de indice
maxIntentosAux= 20 # para agregar nuevo cromosoma en generarAux.generarSolap()


#%%
from random import randint
from random import shuffle
# Las siguientes funciones se agregan a los parámetros ya que utilizan los siguientes parámetros y pueden modificarse
# para que el reemplazo de los numeros al azar sea con o sin reposición.
minStd= 50
maxStd= 70
minBlur= 0
maxBlur= 0.7
cant_clases= 24 #contando el fondo

#%% random number without replacement
numeros= []
def getRandomNum():
    global numeros
    global cant_clases
    if(len(numeros)==0):
        numeros= [i for i in range(1,cant_clases)]
        shuffle(numeros)
        
    alAzar= numeros.pop()
    
    return 2*alAzar + randint(0,1)

#%% random blur and std
blurs= []
stds= []
def getRandomBlurAndStd():
    global blurs
    global cant_angles
    global stds
    global minStd
    global maxStd
    global minBlur
    global maxBlur
    if(len(blurs)==0):
        stepBlur= maxStd//cant_angles
        stepStd = maxBlur*100//cant_angles
        for i in range(cant_angles):
            blurs.append(minBlur+randint(i*stepBlur,(i+1)*stepBlur)/100)
            stds.append( minStd+ randint(i*stepStd, (i+1)*stepStd) )
        shuffle(blurs)
        shuffle(stds)
    return (blurs.pop(), stds.pop())
