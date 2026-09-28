cmake_minimum_required(VERSION 3.8)
project(alidade_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Un URDF nettoye par revision materielle. Chaque config/alidade_<rev>.yaml
# declare l'export Onshape qu'il corrige ; ajouter une revision, c'est ajouter
# un YAML, sans toucher ici.
file(GLOB REVISION_FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/alidade_*.yaml)
# Les exports Onshape en dependance, au glob : c'est la revision qui declare
# lequel elle corrige (cle onshape_export), et coder un nom ici reintroduit le
# couplage que la cle existe justement pour eviter -- un export renomme faisait
# echouer le build sur un fichier absent, sans rapport visible avec la cause.
file(GLOB ONSHAPE_EXPORTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/onshape/*.urdf)
set(GENERATED_URDFS)
foreach(revision_file ${REVISION_FILES})
  get_filename_component(stem ${revision_file} NAME_WE)
  set(generated ${CMAKE_CURRENT_BINARY_DIR}/generated/${stem}.urdf)
  add_custom_command(
    OUTPUT ${generated}
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fix_urdf.py
            --revision ${revision_file}
            --urdf-dir ${CMAKE_CURRENT_SOURCE_DIR}/urdf
            --package ${PROJECT_NAME}
            --output ${generated}
    DEPENDS ${revision_file}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fix_urdf.py
            ${ONSHAPE_EXPORTS}
    COMMENT "Correction de l'export Onshape pour la revision ${stem}"
    VERBATIM
  )
  list(APPEND GENERATED_URDFS ${generated})
endforeach()
add_custom_target(generated_urdfs ALL DEPENDS ${GENERATED_URDFS})

install(DIRECTORY urdf meshes config launch scripts DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_URDFS} DESTINATION share/${PROJECT_NAME}/urdf/generated)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
