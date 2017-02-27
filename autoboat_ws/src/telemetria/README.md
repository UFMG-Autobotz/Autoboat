# Telemetria

Antes de compilar, é necessário instalar o pacote qt-ros
```
sudo apt install ros-kinetic-qt-ros
```

Também é necessária a seguinte gambiarra:

1. Procure o arquivo `/usr/include/boost/type_traits/detail/has_binary_operator.hpp` em seu computador
2. Envolva-o entre `#ifndef Q_MOC_RUN` e `#endif` (você precisará alterar as permissões para isso)

E tá pronto o sorvetinho
