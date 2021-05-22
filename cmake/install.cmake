#include(GNUInstallDirs)
#install(
#        TARGETS
#            ${PROJECT_NAME}
#        EXPORT
#            ${PROJECT_NAME}Targets
#        LIBRARY DESTINATION
#            ${CMAKE_INSTALL_LIBDIR}
#        RUNTIME DESTINATION
#            ${CMAKE_INSTALL_BINDIR}
#        ARCHIVE DESTINATION
#            ${CMAKE_INSTALL_LIBDIR}
#        INCLUDES DESTINATION
#            include
#        PUBLIC_HEADER DESTINATION
#            include
#)
#
#install(
#        DIRECTORY
#            include/${PROJECT_NAME}
#        DESTINATION
#            include
#)
