Bem vindo, aqui se encontra o tutorial para a instalação do micro-ros (testado no humble e jazzy)

O tutorial se divide em três etapas, 1° Clonar esse repositorio, 2° que é a instalação do agente micro-ros em seu pc e a 3° que é os arquivos pre-compilados para o arduino.


######################################### 1° Etapa, Clonar o repositorio ###############################################################
1° Clonar o repositorio usando "git clone git@github.com:edifier15/Project_Atlas.git".

######################################### 2 ° Etapa, instalação do micro-ros no pc #####################################################
1° Realizar a instalação do ROS2 em sua maquina, verificar se vai utilizar a versão humble ou jazzy para a instalação, basta clicar no link https://www.ros.org/blog/getting-started/ escolher a versão e seguir com o proprio tutorial do site, se atentar apenas para instalar a versão desktop full.

2° Hora de instalar o micro-ros, acesse o site https://github.com/hippo5329/micro_ros_arduino_examples_platformio/wiki , navegue até a parte onde contém "Build micro-ROS agent" (lembre-se, não criar a pasta uros_ws pois você vai fazer o clone do git dentro do "Project_Atlas/src") .

######################################## 3° Etapa, Preparar o ambiente do IDE ARDUINO #################################################
Instale a IDE do ARDUINO acessando o site https://www.arduino.cc/en/software/ e siga o tutorial https://github.com/micro-ROS/micro_ros_arduino/tree/humble?tab=readme-ov-file precisa somente da etapa "How to use the precompiled library", lembre-se quando instalar a placa esp32 através da board manager, selecionar a versão v2.0.2 que é a compativel com o micro-ros

