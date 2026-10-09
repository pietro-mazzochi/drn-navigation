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

No filters.yaml do pacote path_planner_server contem um endereço local à imagem map3.yaml do pacote map_server 
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
### <span style="color:#87CEEB;"> Utilizar os dois LiDARs para o topico \scan


```bash
ros2 launch drn_integration_pointcloud_to_laserscan pointcloud2_to_laserscan_launch.py 
```

## Execução da Simulação

###  🚐 <span style="color:#87CEEB;"> 1- Abrir o ambiente simulado da fabrica da DALCA + robô da DALCA 
Para iniciar a simulação da fabrica da dalca no gazebo com o robô da dalca, execute os seguintes comandos no terminais:

**Terminal 1:**
```bash
ros2 launch drc_flexnav_bringup simulation_controller.launch.xml \
    world:=dalca_light_2 \
    x:=17.0 y:=10.83 z:=0.1 Y:=3.14 \
    lightweight:=true \
    config_rviz:=pathplanning \
    lidar_rear:=false
```

### Argumentos do Comando

#### **world**
- **dalca_full**: Cenário completo.
- **dalca_light**: Versão mais leve do cenário.
- **dalca_light_2**: Novo cenário para testes de navegação 
- **empty**: Cenário vazio.

#### **lightweight**
- **true**: Versão mais leve do robô.
  - **Exclusões**: Magnetômetro.
  - Optimização dos parâmetros do LiDARs para a simulação
- **false**: Versão original da Dalca.

#### **config_rviz**
- **Default:** Configuração do rviz centrado nos sensores.
- **pathplanning:** Configuração do rviz centrado na navegação (costmap, global planner , local planner, etc)

#### **lidar_rear**
- **false:** Desativar o lidar trasseiro e os laser scan.
- **true:** Ativar o lidar trasseiro e os laser scan.



