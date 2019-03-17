%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
--------     CONTENTS     --------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

0- REQUIREMENTS
1- INSTALLATION AND DIRECTORIES
2- SETTINGS
3- DATASET
4- OUTPUT FILES
5- EXAMPLES


=================================
0- REQUIREMENTS
=================================
- Python (Tested on Python 3.6)

- Python librarys:
  + matplotlib (2.2.3)
  + numpy (1.15.2)
  + opencv-contrib-python (3.4.2.17)
  + opencv-python (3.4.2.17)
  + random
  + scikit-image (0.14.0)

** This tool was tested with these libraries versions. However, older versions
   could also work.


=================================
1- INSTALLATION AND DIRECTORIES
=================================
Installation only requires to unzip the "GenData.zip" file.
This tool can be launched by typing in the command line:

>> python3 generar.py

Additionally, the following command extracts the chromosomes from the karyograms:

>> python3 separarKaryo.py


Directories:
  karyograms_pki-3_612:   karyograms from Pki-3 dataset
  salida:                 output generated files.
  Separados:              individual chromosomes extracted from karyograms.


====================================
2- SETTINGS
====================================
The arguments of training are stored in "generar_params.py".

--- main parameters ---

 directorio                ---> path to the individual chromosomes
 cuantos_raw               ---> number of output files '.npz'
 intervalo                 ---> how many karyograms process each output file '.npz'
 cant_angles               ---> how many angles are the generated images rotated
 tamImagen                 ---> output image size
 train_cant                ---> how many images are generated for each karyogram for training
 test_cant                 ---> how many images are generated for each karyogram for testing
 cant_crom_min             ---> minimal amount of chromosomas per generated image
 cant_crom_max             ---> maximal amount of chromosomas per generated image
 porcMinSolap              ---> minimal percentage of overlap for each chromosome added
 porcMaxSolap              ---> maximal percentage of overlap for each chromosome added
 minNewEP                  ---> minimal amount of new end point for each chromosome added


More details in "documentacion.pdf".
A typical configuration for all these parameters is provided with the software.


====================================
3- DATASET
====================================
Images files in "/karyograms_pki-3_612/" are from "Passau Chromosome Image Data, Pki-3", used in 
Gunter Ritter, Le Gao: Automatic segmentation of metaphase cells based on global 
context and variant analysis. Pattern Recognition 41 (2008) 38-55.

Link to the dataset: 
http://www.fim.uni-passau.de/en/faculty/former-professors/mathematical-stochastics/chromosome-image-data


=================================
4- OUTPUT FILES
=================================
Generated images for training are stored in "salida/train<n>.npz".
<n> is an incremental number to avoid repetition.
Generated images for testing are stored in "salida/test.npz".


=================================
5- EXAMPLES
=================================
'separarKaryo.py' is configured to extract the chromosomes for all the karyograms which
contain 46 chromosomes. 
This ones are already in 'Separados/raw<n>/', <n> is an incremental number to avoid repetition.

'generar_params.py' is configured to generate 12 output files for training using
480 karyograms.
The rest are used to generate an output file for testing.
The correspondent output is already in "salida/".

