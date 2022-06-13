#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from tf import transformations
from math import pi, atan2, degrees

REGISTRA_CREEPERS = 0
PROCURA_CREEPER = 1
SEGUE_CREEPER = 2
VOLTA_CENTRO = 3

TOKEN_GIRANDO = 0
TOKEN_DEU_VOLTA = 1
TOKEN_ACHOU = 2
TOKEN_DERRUBOU = 3


def diferenca_angulos(deg1, deg2):
    """ Calcula a diferença entre dois ângulos, em graus, com resultado entre -180 e 180 """
    deg_diff = deg2 - deg1
    deg_diff = deg_diff - 360 if deg_diff > 180 else deg_diff
    deg_diff = deg_diff + 360 if deg_diff < -180 else deg_diff
    return deg_diff


class Robo:

    def __init__(self):
        """ Inicialização do controle do robô"""

        ############################################################################################
        
        ### Variáveis de estado do robô ###
        
        # Posição inicial dada pela odometria          
        self.x_ini = None
        self.y_ini = None
        self.theta_ini = None 

        # Posição dada pela odometria          
        self.x = 0
        self.y = 0
        self.theta = 0

        # Dicionario de IDs e ângulos de creepers a serem derrubados
        self.creepers = {}
                
        # Indica se deu pelo menos uma volta
        self.deu_volta = False
        # Indica se saiu de perto do início da pista
        self.saiu = False
        # Inidica se voltou à posição do início da pista
        self.voltou_inicio = False
        # Indica se tem creeper à frente
        self.tem_creeper = False
        # Indica se a garra está levantada
        self.garra_arriba = False

        # Erro para o controle porporcional do segue_creeeper()
        self.err = 0
        
        # Comportamento atual
        self.estado = REGISTRA_CREEPERS
        
        #########################################################################################
        
        ###  Publishers e subscribers
        
        self.bridge = CvBridge()
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.image_sub = rospy.Subscriber('/camera/image/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)

        self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                             Twist, 
                                             queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                  LaserScan, 
	 		                                    self.laser_callback)

        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        

        #########################################################################################
        
        ###  Variáveis do ROS

        self.twist = Twist()
        self.laser_msg = LaserScan()
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)

        #########################################################################################

    def laser_callback(self, msg):
        """ 
        Método chamado quando uma nova leitura do laser é obtida.
        Além de guardar a última leitura, determina se há um creeper próximo
        """
        self.laser_msg = msg

        self.tem_creeper = False
        
        if min(np.min(msg.ranges[:10]),np.min(msg.ranges[350:])) < 0.3:
            self.tem_creeper = True
        else:
            self.tem_creeper = False


    def odom_callback(self, msg):
        """
        Método invocado quando uma nova leitura da odometria é obtida
        Além de guardar a última posição do robô, determina se deu a volta na pista
        """
        
        quat = msg.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista))
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = angulos[2]

        # Detecta posição inicial
        if self.x_ini is None:
            self.x_ini = self.x
            self.y_ini = self.y
            self.theta_ini = self.theta
            print("ANGULO INI: ", self.theta)

        # Determina se deu a volta
        theta_diff = diferenca_angulos(self.theta_ini, self.theta)
        if abs(theta_diff) < 10:
            if self.saiu and not self.deu_volta:
                # Só deu a volta se passou perto do angulo inicial após ter saído
                self.deu_volta = True
                print("DEU A VOLTA!!!!!")
            else:
                self.deu_volta = False
        else:
            # Indica que saiu de perto do angulo inicial
            self.saiu = True

        # Determina se está perto do ponto inicial
        if (self.x-self.x_ini)**2 + (self.y-self.y_ini)**2 < 0.3**2:
            self.voltou_inicio = True


    def image_callback(self, msg):
        """ 
        Método invocado quando chega uma nova imagem 
        Se estiver no estado de registrar creepers, anota os IDs de aruco que estão próximos aos creepers
        """
        
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            cv_image = cv_image.copy()

            ## Encontra os Arucos com seus centros e IDs
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            all_corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            aruco.drawDetectedMarkers(cv_image, all_corners, ids);
            
            if ids is not None:
                centro_x_arucos = np.empty(len(ids), dtype=int)

                for i in range(len(ids)):
                    for corners in all_corners[i]:
                        # Encontra a coordenada X do centro das quinas do Aruco 
                        centro_x_arucos[i] = np.array(corners)[:,0].mean()

                ## Encontra os contornos amarelos

                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                lower_yellow = np.array([40//2, 50, 50],dtype=np.uint8)
                upper_yellow = np.array([70//2, 255, 255],dtype=np.uint8)
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                cv2.imshow("mask", mask)

                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                # Encontra os cdntros dos contornos, além de achar o erro para o contorno
                centro_x_contornos = []
                if contours is not None:
                    for c in contours:
                        M = cv2.moments(c)
                        # Só pega contornos bons, cuja área é maio do que 50 pixels
                        if M['m00'] > 50:
                            cx = M['m10']/M['m00']
                            centro_x_contornos.append(int(cx))
                            self.err = cx - cv_image.shape[1]/2
                            cv2.drawContours(cv_image,[c],-1,(0,255,0),3)
                
                ## Registra apena os IDs dos arucos que satisafezem:
                #  -- Centralizados em um contorno amarelo (10 pixels de tolerância)
                #  -- Centralizados no centro da imagem (1/4 de largura de tolerância)
                if self.estado == REGISTRA_CREEPERS: # Evita registrar um creeper que foi deletado
                    for i in range(len(ids)):
                        for x_contorno in centro_x_contornos:
                            if abs(centro_x_arucos[i] - x_contorno) < 10:
                                if abs(centro_x_arucos[i] - cv_image.shape[1]/2) < cv_image.shape[1]/4:
                                    self.creepers[ids[i][0]] = self.theta
                                    print(str(self.creepers))
    

            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)
    
########  COMPORTAMENTOS ###############

    def registra_creepers(self):
        """ Comportamento de registrar os creepers para identificar seus IDs """
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.5

        # Checar se terminou de dar a volta
        if self.deu_volta:
            print(str(self.creepers))
            return TOKEN_DEU_VOLTA
        
        return TOKEN_GIRANDO

    def procura_creeper(self):
        """ Comportamento de girar até encontrar o ângulo do creeper de ID correto """
        
        if len(self.creepers.keys()) == 0:
            print("ACABOU!!!")
            return TOKEN_GIRANDO

        proximo_creeper = np.min(list(self.creepers.keys()))
        angulo_creeper = self.creepers[proximo_creeper]
        delta_angulo = diferenca_angulos(angulo_creeper, self.theta)
        
        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.3 * delta_angulo/abs(delta_angulo)
    
        # Checar se chegou próximo ao ângulo correto (tolerância de 20º)
        if abs(diferenca_angulos(angulo_creeper, self.theta)) < 20:
            # Remove o creeper atual da lista
            del self.creepers[proximo_creeper]
            return TOKEN_ACHOU
        
        return TOKEN_GIRANDO

    def segue_creeper(self):
        """ Comportamento de seguir o creeper até derrubar """
    
        self.twist.linear.x = 0.3
        self.twist.angular.z = -float(self.err) / 1000

        # Verifica se há o creeper está proximo para levantar a garra
        # Após isso, avanço por 50 cm para frente
        if self.tem_creeper:
            if not self.garra_arriba:
                # levanta garra
                self.ombro.publish(0.0) ## para frente
                self.garra_arriba = True
                self.x_ultimo = self.x
                self.y_ultimo = self.y

        if self.garra_arriba:
            distancia_percorrida = ((self.x - self.x_ultimo)**2 + (self.y - self.y_ultimo)**2)**0.5
            if distancia_percorrida > .3:
                self.ombro.publish(-1.0) ## abaixo a garra
                self.garra_arriba = False
                return TOKEN_DERRUBOU
        
        return TOKEN_ACHOU

    def volta_centro(self):
        """ Comportamento de voltar de ré até o ponto inicial """
        
        # calcula o ângulo entre o robô e ponto inicial
        angulo = degrees(atan2(self.y_ini - self.y, self.x_ini - self.x))
        
        self.twist.linear.x = 0.30
        self.twist.angular.z = -(diferenca_angulos(angulo,self.theta))/20

        if self.voltou_inicio:
            return TOKEN_DEU_VOLTA
        
        return TOKEN_GIRANDO

        
    def control(self):
        """ Função que implementa o controle principal e gerencia a mudança de estados"""
        
        # --- BEGIN CONTROL ---
        
        if self.estado == REGISTRA_CREEPERS:
            token = self.registra_creepers()

            if token == TOKEN_DEU_VOLTA:
                self.estado = PROCURA_CREEPER
                print("ESTADO: ", self.estado)

        if self.estado == PROCURA_CREEPER:
            token = self.procura_creeper()

            if token == TOKEN_ACHOU:
                self.tem_creeper = False
                self.estado = SEGUE_CREEPER
                print("ESTADO: ", self.estado)

        if self.estado == SEGUE_CREEPER:
            
            token = self.segue_creeper()

            if token == TOKEN_DERRUBOU:
                self.voltou_inicio = False
                self.estado = VOLTA_CENTRO
                print("ESTADO: ", self.estado)

        if self.estado == VOLTA_CENTRO:
            token = self.volta_centro()

            if token == TOKEN_DEU_VOLTA:
                self.estado = PROCURA_CREEPER
                print("ESTADO: ", self.estado)

        # --- ENDCONTROL ---
        
        #publica velocidade obtida pelo comportamento atual
        self.cmd_vel_pub.publish(self.twist)
        #rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()


# Main loop
if __name__=="__main__":
    rospy.init_node('q3')
    robo = Robo()

    while not rospy.is_shutdown():
        robo.control()

# END ALL
