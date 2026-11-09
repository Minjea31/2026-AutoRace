cmake_minimum_required(VERSION 3.8)
project(yolo_detect)

find_package(ament_cmake REQUIRED)

install(PROGRAMS yolo_detect.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detect
)
install(FILES sign_ocr.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ocr DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../model/
  DESTINATION share/${PROJECT_NAME}/model
  FILES_MATCHING PATTERN "*.pt" PATTERN "*.onnx" PATTERN "*.engine"
)

ament_package()
