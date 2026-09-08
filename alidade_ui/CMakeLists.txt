cmake_minimum_required(VERSION 3.8)
project(alidade_ui)

find_package(ament_cmake REQUIRED)

# templates/ : widgets Vizanti, lies dans le dossier public de vizanti_server
# par system/sources.sh (le serveur ne decouvre que son propre dossier).
# config/ : profils de widgets (default_widget_config).
install(DIRECTORY config templates DESTINATION share/${PROJECT_NAME})

ament_package()
