# AutoBoat
O Melhor Barco de Minas


# Estrutura do Repositório

<pre>
|--autoboat_ws    workspace ROS
|  |
|  |-- src        Codigo fonte
|  |-- config     Arquivos de configuracao para os programas que rodam no barco
|
|--firmware       Codigos e imagens para Arduinos, raspberrys, beagleBones, etc...
|
|--telemetria     Códigos de telemetria*
|
|--util           Utilitários usados juntos do robô (ex: calibração, processamento de bags)
|
|--doc            Documentação

* A nó de telemetria que roda dentro do robô fica em 
  autoboat_ws. O cliente que roda em outro PC fica em telemetria.
</pre>

# Instruções

### Compilação e Firmware
TODO: Instruções de como compila o workspace e os programas do firmware. Instruções de como passa o firmware pro robô.

### Inicialização
TODO: Instruções de inicialização do robô

### Calibração
TODO: Instruções de como gerar calibrações e colocá-las em autoboat_ws/config (se houver alguma calibração)

### Telemetria
TODO: Instruções de como inicializar a telemetria
