Bem vindo, aqui se encontra o tutorial para a instalação do micro-ros (testado no humble e jazzy)

O tutorial se divide em três etapas, 1° Clonar esse repositorio, 2° que é a instalação do agente micro-ros em seu pc e a 3° que é os arquivos pre-compilados para o arduino.


######################################### 1° Etapa, Clonar o repositorio ###############################################################
1° Clonar o repositorio usando "git clone git@github.com:edifier15/Project_Atlas.git".

######################################### 2 ° Etapa, instalação do micro-ros no pc #####################################################
1° Realizar a instalação do ROS2 em sua maquina, verificar se vai utilizar a versão humble ou jazzy para a instalação, basta clicar no link https://www.ros.org/blog/getting-started/ escolher a versão e seguir com o proprio tutorial do site, se atentar apenas para instalar a versão desktop full.

2° Hora de instalar o micro-ros, acesse o site https://github.com/hippo5329/micro_ros_arduino_examples_platformio/wiki , navegue até a parte onde contém "Build micro-ROS agent" (lembre-se, não criar a pasta uros_ws pois você vai fazer o clone do git dentro do "Project_Atlas/src") .

3° Instalar a dependencias (sudo apt-get install ros-jazzy-pointcloud-to-laserscan , sudo apt-get install ros-jazzy-joy-linux, sudo apt-get install ros-jazzy-tf2-ros)

Lembre-se de compilar no final.

######################################## 3° Etapa, Preparar o ambiente do IDE ARDUINO #################################################
Instale a IDE do ARDUINO acessando o site https://www.arduino.cc/en/software/ e siga o tutorial https://github.com/micro-ROS/micro_ros_arduino/tree/humble?tab=readme-ov-file precisa somente da etapa "How to use the precompiled library", lembre-se quando instalar a placa esp32 através da board manager, selecionar a versão v2.0.2 que é a compativel com o micro-ros
Após isso, faça o upload na placa com o firmware do projeto, deve funcionar sem erros, você pode usar tanto via cabo, como via wifi, bastante comentar a linha responsavel pela usb e descomentar a linha responsavel pelo wifi, e no arquivo "robot_completo.launch", fazer o mesmo.

####################################### Teste ####################################################
Execute o comando ros2 launch teleop_motor robot_completo.launch.py , e logo em seguida ligue o esp32, o terminal deve mostrar que o esp se conectou e esta comunicando, em outro terminal abra o rviz2 e configure como na imagem abaixo, se atentando principalmente para fixed_frame e realibity policy.

<img width="1569" height="852" alt="image" src="https://github.com/user-attachments/assets/3ba9946f-c6a0-4f4c-b435-ae76293e7d00" />
