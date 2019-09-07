 Lidar Node
=============
## 1. Description
1) sub_cloud.cpp: Lidar의 데이터를 수집하여 감지된 Totem의 Baselink 기준, 2D 평면 기준, Lidar 기준의 xyz 좌표를 전달. 
Rviz를 통해 포착한 Totem의 위치를 Visualization 가능. 2개의 Lidar를 통해 전방의 장애물만 탐지하도록 수정. 

2) docker_cloud.cpp: Docker 평면의 위치와 평면 방정식을 출력. Rviz를 통해 포착한 평면 위치 Visualization 가능. 추후 하나의 노드로 통합 예정
______
## 2. How to Use
1) wamv_gazebo/urdf/wamv_gazebo.urdf.xacro 일부를 다음과 같이 수정: 
'''
  <!-- Add 3D LIDAR -->
    <xacro:if value="$(arg lidar_enabled)">
      <xacro:lidar name="lidar_wamv" type="32_beam"/>
       <xacro:lidar name="lidar_wamv2" x="0.7" type="16_beam"/>
    </xacro:if> 

 <!-- Add 3D LIDAR -->
      <xacro:lidar name="lidar_wamv" z="2" R="${radians(180)}" type="32_beam"/>
       <xacro:lidar name="lidar_wamv2" x="0.7" type="16_beam"/> 
'''

2) 실행 명령어
+ Totem 감지: rosrun lidar_node lidar_node_node
+ Docker 감지: rosrun lidar_node docker_node
______
## 3. Characteristic
1) 프로그램 성능
유효 거리: 50m 내외
오차 범위: 10cm 

2) Lidar Characteristic
전방에 16ch, 32ch의 Lidar를 사용. 32ch Lidar는 baselink 기준 x=0.75, y=0.03, z=2에 위치하였으며, 16ch Lidar는 x=0.75 y=-0.03 z=2에 위치.
_______
## 4. Message

+ rel_pos1.msg
	+ baselink_x, baselink_y, baselink_z: wamv 전방에 위치한 Cluster의 좌표를 baselink 기준으로 저장.
	+ twodim_x, twodim_y, twodim_z : wamv 전방에 위치한 Cluster의  좌표를 2D 맵 기준으로 저장.

+ task5.msg
	+ baselink_plane_x, baselink_plane_y, baselink_plane_z: wamv 전방에 위치한 평면 중앙의 좌표를 baselink 기준으로 저장.
	+ twodim_plane_x, twodim_plane_y, twodim_plane_z : wamv 전방에 위치한 평면 중앙의 x, y, z 좌표를 2D 맵 기준으로 저장.
_________
## 5. coordinate_axis.odp: Message에 사용된 좌표축 도식화


________

변경사항
1) msg 내 lidar_pos.msg 삭제. 이에 따라 Cmakelist 파일 변경
2) 전 후방 장애물 인식에서 전방 인식으로 변경
3) rel_pos1.msg 내용 변경: baselink2_x, baselink2_y, baselink2_z 및 twodim2_x, twodim2_y, twodim2_z 삭제
4) docker node 추가. 추후 하나의 노드로 통합 예정. 이에 따라 Cmakelist 수정




