Para ejecutar la simulacion, se deben seguir los siguientes pasos:
1. Crear una carpeta catkin_ws para almacenar los archivos de ROS
2. Instalar los paquetes de turtlebot3 en la carpeta src del workspace (guia de instalacion: https://www.youtube.com/watch?v=f_lcbVQ3Oa4)
3. Compilar el paquete usando catkin_make
4. Insertar el folder controller_teleop (brindado aqui) en la carpeta src del workspace
5. Compilar los paquetes usando catkin_make

Luego de la instalacion. Se usan los siguiente comandos para correr la simulacion
1. roslaunch turtlebot3_gazebo turtlebot3_house.launch ----> consola 1
2. rosrun controller_teleop imu_cont_teleop.py

NOTA: Es importante agregar las ruta del workspace y el tipo de turtlebot a utilizar
