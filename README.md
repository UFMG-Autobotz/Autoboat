# AutoBoat
O Melhor Barco de Minas


# Estrutura do Repositório

<pre>
╠══autoboat_ws    workspace ROS *
║  ║
║  ╠══ src        Codigo fonte
║  ╚══ config     Arquivos de configuracao para os programas que rodam no barco
║
╠══firmware       Codigos e imagens para Arduinos, raspberrys, beagleBones, etc...
║
╠══telemetria     Códigos de telemetria **
║
╚══util           Utilitários usados juntos do robô (ex: calibração, processamento de bags)


* As pastas devel e build não devem ser incluidas no repositório
  pois são binários. Essas pastas estão no gitignore.

** O nó de telemetria que roda dentro do robô fica em 
   autoboat_ws. O cliente que roda em outro PC fica em telemetria.
</pre>

# Instruções

### Compilação e Firmware
Serão utilizados dois Arduinos, o [Mestre](firmware/Mestre) e o [Chassi](firmware/chassi).
Para compilar os códigos, é necessário que as seguintes versões mais recentes destas bibliotecas estejam na pasta **libraries** do seu computador:
- [WireBotz](https://github.com/pedroblanc/modulos/tree/master/WireBotz/copy_to_libraries/WireBotz)
- [MotorDePasso e MotorCC](https://github.com/pedroblanc/modulos/tree/master/Acionamento%20de%20motores)
- [Interrompe](https://github.com/pedroblanc/modulos/tree/master/Interrompe)
- [NewPing](https://bitbucket.org/teckel12/arduino-new-ping/downloads/NewPing_v1.8.zip)
- [I2C, I2Cdev e MPU6050](https://drive.google.com/drive/u/0/folders/0BwwHJR9yDw-LWFl2aUVUamx4WG8)

### Inicialização
TODO: Instruções de inicialização do robô

### Calibração
TODO: Instruções de como gerar calibrações e colocá-las em autoboat_ws/config (se houver alguma calibração)

### Telemetria
Instruções de como inicializar a telemetria
- Para instalar e compilar: leia [este documento](telemetria/README.md).
- Para estabelecer a conexão entre os dois computadores:  
  *(será adicionado em breve, por enquanto contente-se com [isso](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) e [isso](http://wiki.ros.org/ROS/NetworkSetup))*
