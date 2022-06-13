#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

from fotogrametria import *

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "laserdefense.mp4"

def encontra_naves(img_bgr):
    """ Seleciona uma regiao de interesse (ROI) entre a nave e a mascara """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV) # Converte para HSV
    mask = cv2.inRange(hsv, (20//2, 50, 50), (40//2, 255, 255)) # Seleciona a regiao de interesse
    contorno = encontrar_maior_contorno(mask) # Encontra o maior contorno
    bounding_box = cv2.boundingRect(contorno) # Encontra o bounding box
    x0, y0, w, h = bounding_box # Separa as coordenadas do bounding box
    roi = img_bgr[y0:y0+h, x0:x0+w] # Seleciona a regiao de interesse
    return roi, mask

def recebeu_tiro(img_bgr):
    """ Retorna True quando a nave recebeu um tiro no seu ROI """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV) # Converte para HSV
    mask = cv2.inRange(hsv, (170//2, 50, 50), (200//2, 255, 255)) # Seleciona a regiao de interesse
    contorno = encontrar_maior_contorno(mask) # Encontra o maior contorno
    area = 0 # Area do contorno
    if contorno is not None:
        cv2.drawContours(img_bgr, [contorno], 0, (0, 0, 255), -1) # Desenha o contorno na imagem
        area = cv2.contourArea(contorno)
        return area > 20
    return False


if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)


    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

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
        



        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('imagem', frame)

        # Pressione 'q' para interromper o video
        if cv2.waitKey(1000//30) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

