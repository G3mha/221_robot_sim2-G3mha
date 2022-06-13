#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

from fotogrametria import encontrar_maior_contorno

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "laserdefense.mp4"

##################################################
def encontra_nave(bgr):
    """ Define o ROI com a imagem da nave e a mascara """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,(20//2, 50, 50),(40//2, 255, 255))
    contorno = encontrar_maior_contorno(mask)

    bbox = cv2.boundingRect(contorno)
    x0,y0,w,h = bbox
    roi = bgr[y0:y0+h, x0:x0+w]
    return roi, mask
##################################################
def encontra_tiro(roi_bgr):
    """ Retorna True quando tem tiro na ROI da nave """
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,(170//2, 50, 50),(200//2, 255, 255))
    contorno = encontrar_maior_contorno(mask)
    area = 0
    if contorno is not None:
        cv2.drawContours(roi_bgr, [contorno], 0, (0,0,255),-1)
        area = cv2.contourArea(contorno)

    return contorno is not None and area > 20

##################################################



if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)


    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")


    # inicializa as variaveis de estado
    tinha_tiro = False
    conta_tiro = 0
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            #cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            break

        # Our operations on the frame come here
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        roi, mascara_nava = encontra_nave(frame)
        
        tem_tiro = encontra_tiro(roi)

        if tem_tiro:
            frame[mascara_nava == 255] = (255,255,255)

        if conta_tiro >=10:
            frame[mascara_nava == 255] = (128,128,128)


        if tem_tiro and not tinha_tiro:
            conta_tiro += 1
            print("ENTROU")
            print("TIROS: ", conta_tiro)
            tinha_tiro = True

        
        if not tem_tiro and tinha_tiro:
            print("SAIU")
            tinha_tiro = False

    
        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('ROI', roi)
        cv2.imshow('imagem', frame)

        # Pressione 'q' para interromper o video
        if cv2.waitKey(1000//30) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

