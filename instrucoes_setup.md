## OpenCV

Para usar o Arduco você deve usar o OpenCV instalado originalmente no SSD. Para garantir que voê não intalou o OpenCV do `pip`, caso não esteja reconhecendo o Aruco, fazer:

    pip uninstall opencv-python


## Jupyterlab

Para instalar Jupyterlab no Linux faça: 

    pip install jupyterlab



## Gazebo Turtlebot

Certifique-se de que seu `.bashrc` têm as variáveis `ROS_IP` e `ROS_MASTER_URI` desabilitadas antes e rodar o Gazebo.

Essas variáveis estarão desabilitadas se tiverem um `#` precedendo a linha. 

## Versão do Python

Os códigos ROS são compatíveis somente com Python 3.

Você pode executar as questões usando o comando `rosrun`, ou ainda pode abrir o VSCode a partir da pasta `./p2_212/scripts`.

## Teleop

Sempre que usar o  `teleop` encerre o programa logo em seguida.  Enquanto estiver aberto o `teleop` ficará enviando comandos de velocidade para o robô, conflitando com seus programas que controlam o robô. 

Lembrando que para lançar o teleop faça: 

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Feche o `teleop` depois de usar.


## catkin_make

Executar `catkin_make` após fazer o download do repositório da prova: 

    cd ~/catkin_ws/
    catkin_make

## Onde baixar os arquivos

O código deve sempre ser baixado na pasta `cd ~/catkin_ws/src` :

    cd ~/catkin_ws/src
    git clone <nome do repo>

## Arquivos executáveis

Certifique-se de que seus scripts Python são executáveis. Após o `catkin_make`, faça:

    roscd p2_212
    cd scripts
    chmod a+x *py

## Executar prova

Para executar arquivos do ROS, faça:

    rosrun p2_212 arquivo.py 

Onde `arquivo.py` é algum script Python executável que você deve ter na pasta `p2_212/scripts`.

Note que os programas `q1/q1.py`e `q2/q2.py` podem ser simplesmente executados direto porque não precisa de ROS.


Certifique-se de que seus scripts ROS rodam com Python 3 e têm sempre no começo:

    #! /usr/bin/env python3
    # -*- coding:utf-8 -*-
    
    from __future__ import division, print_function


## Commit no Github

    Lembre-se, de regularmente fazer
        cd ~/catkin_ws/src/PROVA
        git add --all
        git commit -m "Mensagem aqui"
        git push

