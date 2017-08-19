# AutoBoat
O Melhor Barco de Minas


# Estrutura do Repositório

<pre>
╠══autoboat_ws    workspace ROS *
║  ║
║  ╠══ src        Codigo fonte
║  ╚══ config     Arquivos de configuracao para os programas que rodam no barco
║
╠══doc            Documentações
║
╠══firmware       Codigos e imagens para Arduinos, raspberrys, beagleBones, etc...
║
╚══util           Utilitários usados juntos do robô (ex: calibração, processamento de bags)


* As pastas devel e build não devem ser incluidas no repositório
  pois são binários. Essas pastas estão no gitignore.
</pre>

# Instruções

### Compilação e Firmware
#### Workspace ROS
Para compilar os pacotes no workspace, basta seguir as pequenas instruções da telemetria mencionadas abaixo.  
Em seguida basta rodar o `catkin_make` na raiz do workspace.

Para rodar o software do robô, basta dar source nos pacotes (`. devel\setup.bash`) e rodar o seguinte launch:  
```
roslaunch autoboat_msgs autoboat.launch
```

#### Arduinos
Serão utilizados dois Arduinos, o [Mestre](firmware/Mestre) e o [Chassi](firmware/chassi).
Para compilar os códigos, é necessário que as seguintes versões mais recentes destas bibliotecas estejam na pasta **libraries** do seu computador:
- [WireBotz](https://github.com/pedroblanc/modulos/tree/master/WireBotz/copy_to_libraries/WireBotz)
- [MotorDePasso e MotorCC](https://github.com/pedroblanc/modulos/tree/master/Acionamento%20de%20motores)
- [Interrompe](https://github.com/pedroblanc/modulos/tree/master/Interrompe)
- [NewPing](https://bitbucket.org/teckel12/arduino-new-ping/downloads/NewPing_v1.8.zip)
- [I2C, I2Cdev e MPU6050](https://drive.google.com/drive/u/0/folders/0BwwHJR9yDw-LWFl2aUVUamx4WG8)

### Inicialização
É necessário pressionar o botão da placa de interface para que o robô inicialize. Enquanto ele não for inicializado, os LEDs da placa ficarão piscando de maneira cíclica. Pode ser necessário manter o botão pressionado por um determinado período de tempo, até que os LEDs se apaguem.

### Telemetria
- Antes de compilar, é necessário instalar o pacote qt-ros  
```
sudo apt install ros-kinetic-qt-ros
```  
Também é necessária a seguinte gambiarra:  
 1. Procure o arquivo `/usr/include/boost/type_traits/detail/has_binary_operator.hpp` em seu computador
 2. Envolva-o entre `#ifndef Q_MOC_RUN` e `#endif` (você precisará alterar as permissões para isso)

E tá pronto o sorvetinho

- Caso se deseje estabelecer a conexão entre os dois computadores:  
  *(será adicionado em breve, por enquanto talvez [isso](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) e [isso](http://wiki.ros.org/ROS/NetworkSetup) sejam útieis)*
