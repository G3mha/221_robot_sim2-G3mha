#!/usr/bin/env python3
# -*- coding:utf-8 -*-


#   exemplo adaptado do livro:
#   
#  Programming Robots with ROS.
#  A Practical Introduction to the Robot Operating System
#  Example 12-5. follower_p.py pag265
#  
#  Referencia PD:https://github.com/martinohanlon/RobotPID/blob/master/mock/mock_robot_pd.py 

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

from object_detection_webcam import detect

SEGUE_LINHA = 0
PROCURA_CUBO = 1
SEGUIR_CUBO = 2
VOLTA_LINHA = 3
PARADO = 4

TOKEN_LINHA = 0
TOKEN_CUBO = 1
TOKEN_CHEGOU = 2
TOKEN_ACHOU = 3
TOKEN_NAO_ACHOU = 4
TOKEN_PAROU = 5

class Follower:

    def __init__(self):
        """ Inicialização do controle do robô"""

        # Posição inicial obtida através de `rostopic echo /odom``
        self.x_ini = -0.014652035901415825
        self.y_ini = -1.7307811667677115

        ############################################################################################
        
        ### Variáveis de estado do robô ###
        
        # Posição dada pela odometria          
        self.x = self.x_ini
        self.y = self.y_ini
        
        # Posição do robô onde um cubo foi detectado pela última vez
        self.x_ultimo_cubo = None
        self.y_ultimo_cubo = None

        # Distância até a posição inicial
        self.dist_ini = 0
        # Distância até a posição onde um cubo foi detectado pela última vez
        self.dist_ultimo_cubo = None
        
        # Indica se deu pelo menos uma volta
        self.deu_volta = False
        # Indica se saiu de perto do início da pista
        self.saiu = False
        # Indica se foi detectado um cubo do lado
        self.tem_cubo_lateral = False
        # Indica se foi detectado um cubo na frente
        self.tem_cubo_frontal = False

        # Indica se a linha foi detectada 
        self.achou_linha = False
        # Erro para o controle porporcional do segue_linha()
        self.err = 0
        
        # Contador que indica quantas vezes uma categoria foi detectada seguidamente
        self.conta_mobile_net = 0
        # Última categoria mobilenet detectada
        self.mobile_net_ant = None
        # Última categoria detectada de forma robusta, ou seja, 5x em seguida
        self.mobile_net = None

        # Comportamento atual
        self.estado = SEGUE_LINHA
        
        #########################################################################################
        
        ###  Publishers e subscribers
        
        self.bridge = CvBridge()
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
        Além de guardar a última leitura, determina se há um cubo lateral ou frontal
        """
        self.laser_msg = msg

        if np.min(msg.ranges[260:280]) < 0.8:
            self.tem_cubo_lateral = True
            print("TEM CUBO LATERAL")
        else:
            self.tem_cubo_lateral = False

        self.tem_cubo_frontal = False
        if min(np.min(msg.ranges[:10]),np.min(msg.ranges[350:])) < 0.3:
            self.tem_cubo_frontal = True
        else:
            self.tem_cubo_frontal = False


    def odom_callback(self, msg):
        """
        Método invocado quando uma nova leitura da odometria é obtida
        Além de guardar a última posição do robô, determina se deu a volta na pista
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Determina a distância até o ponto inicial
        self.dist_ini = ((self.x-self.x_ini)**2 + (self.y-self.y_ini)**2)**0.5
        
        # Determina a distância até o ponto de detecção do último cubo
        if self.x_ultimo_cubo is not None:
            self.dist_ultimo_cubo = ((self.x-self.x_ultimo_cubo)**2 + (self.y-self.y_ultimo_cubo)**2)**0.5
        else: self.dist_ultimo_cubo

        # Determina se deu a volta
        if self.dist_ini < 0.3:
            if self.saiu:
                # Só deu a volta se chegou perto do ponto inicial após ter saído
                self.deu_volta = True
                print("DEU A VOLTA!!!!!")
            else:
                self.deu_volta = False
        else:
            # Indica que saiu de poerto do ponto inicial
            self.saiu = True

    
    def image_callback(self, msg):
        """ 
        Método invocado quando chega uma nova imagem 
        Invoca a mobile net e, caso enxergue a lihha verde, encontra o erro de posicionamento da mesma na imagem
        """
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            cv_image = cv_image.copy()
            
            _, resultados = detect(cv_image) 

            if len(resultados) > 0:
                for resultado in resultados:
                    if resultado[0] in ('cow', 'dog', 'horse', 'car'):
                        # Só aceita o resultado se foi o msmo por 5 vezes seguidas
                        if self.mobile_net_ant == resultado[0]:
                            self.conta_mobile_net += 1
                        else:
                            self.mobile_net_ant = resultado[0]

                        if self.conta_mobile_net >= 5:
                            self.mobile_net = self.mobile_net_ant
                            self.conta_mobile_net = 0
                            print("FOUND ", resultado[0])
                        
                        break

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([60, 50, 50],dtype=np.uint8)
            upper_yellow = np.array([100, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            h, w, d = cv_image.shape
            search_top = 2*h//4
            search_bot = 3*h//4 + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            M = cv2.moments(mask)

            self.achou_linha = False
            if M['m00'] > 0:
                self.achou_linha = True
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.err = cx - w/2
                cv2.circle(cv_image, (cx,cy), 20, (0,0,255), -1)

            cv2.imshow("window", cv_image)
            cv2.imshow("mask", mask)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)
    
########  COMPORTAMENTOS ###############

    def segue_linha(self):
        """ Comportamento de seguir a linha """
    
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(self.err) / 1000

        # Verifica se há um novo cubo lateralmente
        # Só procuro por um cubo se o robô deu a volta E se afastou pelo menos
        # 1.5 m da última posição em que viu um cubo
        if self.tem_cubo_lateral and self.deu_volta:
            if self.dist_ultimo_cubo is None or self.dist_ultimo_cubo > 1.5:
                self.x_ultimo_cubo = self.x
                self.y_ultimo_cubo = self.y
                return TOKEN_CUBO
        
        return TOKEN_LINHA


    def procura_cubo(self):
        """ Comportamento de checar se o cubo é da classe procurada """
        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.2

        # Checar se imagem do cubo é a que quero
        if self.mobile_net is not None:
            # Checar se 'e o que quero
            if self.mobile_net == 'dog':
                return TOKEN_ACHOU
            else:
                return TOKEN_NAO_ACHOU

        # se ainda não achou a imagem, giramos até ficar de frente para a caixa
        # e começamos a dar ré, pois a image é possivelmente muito grande

        # Girar até ficar de frente com a caixa, parando o giro lentamente para não escorregar
        #print("LASER ARGMIN: ", np.argmin(self.laser_msg))
#        if np.argmin(self.laser_msg.ranges) in (0,1,358,359):
#            self.twist.linear.x = -0.1
#            self.twist.angular.z += 0.5
#            self.twist.angular.z = min(0, self.twist.angular.z)
            
        return TOKEN_CUBO

    def seguir_cubo(self):
        """ Comportamento de avançar até parar próximo ao cubo """
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0

        if self.tem_cubo_frontal:
            return TOKEN_PAROU

        return TOKEN_CUBO


    def voltar_linha(self):
        """ Comportamento de voltar para a pista """
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.2

        # Só muda o comportamento para voltar a seguir linha
        # se o cubo estiver bem à direita para não trombar nele
        if self.achou_linha and 200 < np.argmin(self.laser_msg.ranges) < 290:
            return TOKEN_LINHA
        
        return TOKEN_NAO_ACHOU

        
    def control(self):
        """ Função que implementa o controle principal e gerencia a mudança de estados"""
        print("ESTADO: ", self.estado)
        
        # --- BEGIN CONTROL ---
        
        if self.estado == SEGUE_LINHA:
            token = self.segue_linha()

            if token == TOKEN_LINHA:
                self.estado = SEGUE_LINHA
            elif token == TOKEN_CUBO:
                # Preparação para iniciar o comportamento de checar o cubo 
                self.conta_mobile_net = 0
                self.mobile_net = None
                self.mobile_net_ant = None
                self.estado = PROCURA_CUBO

        if self.estado == PROCURA_CUBO:
            token = self.procura_cubo()

            if token == TOKEN_ACHOU:
                self.estado = SEGUIR_CUBO
            elif  token == TOKEN_NAO_ACHOU:
                self.estado = VOLTA_LINHA
            elif token == TOKEN_CUBO:
                self.estado = PROCURA_CUBO

        if self.estado == VOLTA_LINHA:
            token = self.voltar_linha()

            if token == TOKEN_NAO_ACHOU:
                self.estado = VOLTA_LINHA
            elif token == TOKEN_LINHA:
                self.estado = SEGUE_LINHA

        if self.estado == SEGUIR_CUBO:
            token = self.seguir_cubo()

            if token == TOKEN_PAROU:
                self.estado = PARADO
            else: self.estado = SEGUIR_CUBO

        if self.estado == PARADO:
            # Aqui não chamamos um método para o comportamento pois ele seria muito simples.
            # Então o comportamento é implementado aqui mesmo
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            print("CHEGOU, EEEEEEEEEEE!!!!!")

        # --- ENDCONTROL ---
        
        #publica velocidade obtida pelo comportamento atual
        self.cmd_vel_pub.publish(self.twist)
        #rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()


# Main loop
if __name__=="__main__":
    rospy.init_node('q4')
    follower = Follower()

    while not rospy.is_shutdown():
        follower.control()

# END ALL
