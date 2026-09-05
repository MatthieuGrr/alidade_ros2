cmake_minimum_required(VERSION 3.8)
project(alidade_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Un URDF nettoye par revision materielle. Chaque config/alidade_<rev>.yaml
# declare l'export Onshape qu'il corrige ; ajouter une revision, c'est ajouter
# un YAML, sans toucher ici.
file(GLOB REVISION_FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/alidade_*.yaml)
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
            ${CMAKE_CURRENT_SOURCE_DIR}/urdf/onshape/alidade.urdf
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
