cmake_minimum_required(VERSION 3.5)
project(automap)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Python 스크립트 설치 설정
install(
  PROGRAMS
    automap/mapping.py
  DESTINATION lib/${PROJECT_NAME}  # 설치 경로 설정
)

# Launch 파일 설치
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()