# DRN-Navigation

Fontes dos repositorios utilizados:

- [DRC-flexnav](https://github.com/isisim/drc-flexnav)

> **Observação**: Seguir os passos de instalação da DRC-flexnav e DRC-simulation.

## ⚙️ Requisitos

```bash
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros2-control
sudo apt-get install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-rqt_joint_trajectory_controller ros-$ROS_DISTRO-moveit-*
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
sudo apt install xterm
sudo apt install ros-$ROS_DISTRO-slam-toolbox
sudo apt install ros-$ROS_DISTRO-geographic-msgs
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
```
Qualquer dependencia faltanta pode ser encontrada na documentação dos repositorios.


### Build the clones
```bash
cd to_your_workspace
colcon build --symlink-install
```
### <span style="color:#87CEEB;"> Mandar comandos de velocidade e rotação

**Para Dalca:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/base_vel_cmd
```
## Mapeamento
### <span style="color:#87CEEB;"> Mapeamento Manual
Execução manual do mapeamento, para salvar o mesmo pode utilizar-se o serviço "/slam_toolbox/save_map" ou pelo plugin de slam_toolbox no Rviz2.

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=to_your_workspace/src/drn-navigation/drn_navigation_mapping/config/slam.yaml use_sim_time:=true
```

### <span style="color:#87CEEB;"> Salvar mapeado feito na pasta de execução 
```bash
ros2 run nav2_map_server map_saver_cli
```

## Execução da Simulação

### <span style="color:#87CEEB;"> 1- Abrir o ambiente simulado da fabrica da DALCA + robô da DALCA 
Para iniciar a simulação da fabrica da dalca no gazebo com o robô da dalca, execute os seguintes comandos no terminais:

**Terminal 1:**
```bash
ros2 launch drc_flexnav_bringup simulation_controller.launch.xml \
    world:=dalca_light \
    x:=17.0 y:=10.83 z:=0.1 Y:=3.14 \
    lightweight:=true \
    config_rviz:=pathplanning \
    rear_lidar_sim:=true
```

### Argumentos do Comando

#### **world**
- **dalca_full**: Cenário da dalca com objetos dinâmicos;
- **dalca_light**: Cenário da dalca sem objetos dinâmicos;
- **dalca_light_2**: Novo cenário da dalca eliminando os bunkers para testes de navegação;
- **empty**: Cenário vazio.

#### **lightweight**
- **true**: Versão mais leve do robô.
  - **Exclusões**: Magnetômetro.
  - Optimização dos parâmetros do LiDARs para a simulação
  - Troca por uma versão mais leve do chassis. (Precisa ter o arquivo chassis_1.dae na pasta meshes do flexnav)
- **false**: Versão original da Dalca.

#### **config_rviz**
- **Default:** Configuração do rviz centrado nos sensores.
- **pathplanning:** Configuração do rviz centrado na navegação (costmap, global planner , local planner, etc)

#### **rear_lidar_sim**
- **false:** Desativar o lidar trasseiro.
- **true:** Ativar o lidar trasseiro.


Para executar os nós de navegação, execute os seguinte comando no terminal :

### <span style="color:#87CEEB;"> 2- Utilizar os dois LiDARs para o topico \scan (Só utilizar caso não seja chamado desde o bringup)
**Terminal 2:**

Caso 2 LiDARs:
```bash
ros2 launch drn_integration_pointcloud_to_laserscan pointcloud2_to_laserscan_launch.py
```
ou 
caso 1 LiDAR:
```bash
ros2 launch drn_integration_pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
```

### <span style="color:#87CEEB;">3- Executar nós de navegação

**Terminal 3:**
```bash
ros2 launch drn_navigation_mapping drn_navigation_costmap_launch.py \
    map_name:=map_2_lidar
```
### Configuração de parâmetro dentro do launcher
#### **map_name**
- **map**:  Nome padrão utilizado para os mapas. (Depende do recebimento dos arquivos do supervisório)
- **map_1_lidar**:  Mapeamento foi feito com dalca robot(1 LiDARs). Pode ser usado nos ambientes dalca_full e dalca_light.(Menor precisão no distanciamentos)
- **map_2_lidar**: Mapeamento foi feito com dalca robot(2 LiDARs). Pode ser usado nos ambientes dalca_full e dalca_light.
- **map_gt**: Mapeamento foi feito com dalca robot(2 LiDARs). Pode ser usado no ambiente dalca_light.



