# Dock Navigation Client

Este pacote contém um nó ROS 2 responsável por detectar docks e navegar até eles utilizando o `drn_perception_msgs::srv::DocksDetected` e a ação `nav2_msgs::action::NavigateToPose`.

## Execução

Para executar o nó de docagem, utilize:

```bash
ros2 run drn_navigation_decision_maker test_docking_node --ros-args -p direction:=front -p base_frame:=map -p dock_index:=0
```
Parâmetros:
- `direction` (string, padrão: "front") - Direção para detecção do dock.
- `dock_index` (int, padrão: 0) - Índice do tipo de detecção da doca.

Certifique-se de que o `ros2 launch` está rodando os nós necessários antes de executar o cliente de navegação.

Nó dock_detector_service_node:
Lembrando que tem que estar configurado com "map"
```bash
ros2 launch drn_perception_reference_detection drn_perception_reference_detection_launch.py
```
Para testar o serviço de detecção de Dock:

```bash
ros2 service call /drn_perception/dock_detector/dock_detection drn_perception_msgs/srv/DocksDetected "{direction: 'front', base_frame: 'map'}"
```

## Teste no mapa dalca_light

Ativar simulação, não ativar os dois Lidars pois acaba diminuindo a performance do serviço de detecção de doca.

Primeiro levar o robo proximo da doca:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 27.550685422189854, y: 10.070328718115048, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.8602089810472161, w: 0.5099416720819255}}}}"
```

Logo levar o robô na posição para detectar a doca:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 26.599769662294122, y: 7.933717613913461, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.8192330035710549, w: 0.5734607971430549}}}}"
```

Ativar o nó de docagem:

```bash
ros2 run drn_navigation_decision_maker test_docking_node --ros-args -p direction:=front -p base_frame:=map -p dock_index:=0
```

