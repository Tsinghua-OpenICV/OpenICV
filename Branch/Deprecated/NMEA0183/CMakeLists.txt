project(NMEA0183LIB)

#find_package(Qt4 REQUIRED)
#include(${QT_USE_FILE})

#add_definitions(${QT_DEFINITIONS}) 
#link_directories (${QT_LIBRARIES})

include_directories(
                    #${QT_INCLUDE_DIR}
                    include/
                   )

#protocole de la centrale IXSEA
set(PHINS_STANDARD_SRC
src/HEHDT.cpp
src/ATTITUD.cpp
src/POSITI.cpp
src/SPEED.cpp
src/UTMWGS.cpp
src/HEAVE.cpp
src/STDHRP.cpp
src/STDPOS.cpp
src/STDSPD.cpp
src/TIME__.cpp
#src/LOGIN.cpp
#src/LOGDVL.cpp
#src/LOGWAT.cpp
include/HEHDT.hpp
include/ATTITUD.hpp
include/POSITI.hpp
include/SPEED.hpp
include/UTMWGS.hpp
include/HEAVE.hpp
include/STDHRP.hpp
include/STDPOS.hpp
include/STDSPD.hpp
include/TIME__.hpp
#include/LOGIN.hpp
#include/LOGDVL.hpp
#include/LOGWAT.hpp
)
SOURCE_GROUP(PHINS_std FILES ${PHINS_STANDARD_SRC} )

set(NMEA_SRC
src/AAM.cpp
src/ALM.cpp
src/APB.cpp
src/ASD.cpp
src/BEC.cpp
src/BOD.cpp
src/BWC.cpp
src/BWR.cpp
src/BWW.cpp
src/CHECKSUM.cpp
src/DBT.cpp
src/DCN.cpp
src/DECCALOP.cpp
src/DPT.cpp
src/EXPID.cpp
src/FIELD.cpp
src/FREQMODE.cpp
src/FSI.cpp
src/GDA.cpp
src/GDF.cpp
src/GDP.cpp
src/GGA.cpp
src/GLA.cpp
src/GLC.cpp
src/GLF.cpp
src/GLL.cpp
src/GLP.cpp
src/GOA.cpp
src/GOF.cpp
src/GOP.cpp
src/GSA.cpp
src/GST.cpp
src/GSV.cpp
src/GTD.cpp
src/GXA.cpp
src/GXF.cpp
src/GXP.cpp
src/HCC.cpp
src/HDG.cpp
src/HDM.cpp
src/HDT.cpp
src/HEX.cpp
src/HEXVALUE.cpp
src/HSATIT.cpp
src/HSC.cpp
src/HSPOS.cpp
src/IMA.cpp
src/LAT.cpp
src/LATLONG.cpp
src/LCD.cpp
src/LONG.cpp
src/LORANTD.cpp
src/MANUFACT.cpp
src/MHU.cpp
src/MLIST.cpp
src/MTA.cpp
src/MTW.cpp
src/MWV.cpp
src/NMEA0183.cpp
src/OLN.cpp
src/OMEGAPAR.cpp
src/OSD.cpp
src/P.cpp
src/RADARDAT.cpp
src/RATIOPLS.cpp
src/RESPONSE.cpp
src/RMA.cpp
src/RMB.cpp
src/RMC.cpp
src/ROT.cpp
src/RPM.cpp
src/RSA.cpp
src/RSD.cpp
src/RTE.cpp
src/SATDAT.cpp
src/SENTENCE.cpp
src/SFI.cpp
src/STN.cpp
src/TALKERID.cpp
src/TEP.cpp
src/TRF.cpp
src/TTM.cpp
src/VBW.cpp
src/VDR.cpp
src/VHW.cpp
src/VLW.cpp
src/VPW.cpp
src/VTG.cpp
src/VWE.cpp
src/WAYPTLOC.cpp
src/WCV.cpp
src/WDC.cpp
src/WDR.cpp
src/WNC.cpp
src/WPL.cpp
src/XDR.cpp
src/XTE.cpp
src/XTR.cpp
src/ZDA.cpp
src/ZFI.cpp
src/ZFO.cpp
src/ZLZ.cpp
src/ZPI.cpp
src/ZTA.cpp
src/ZTE.cpp
src/ZTG.cpp
src/ZTI.cpp
src/ZWP.cpp
src/ZZU.cpp
)
SOURCE_GROUP(NMEA_std FILES ${NMEA_SRC} )

add_library(${PROJECT_NAME} ${NMEA_SRC} ${PHINS_STANDARD_SRC} )

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )

# install(TARGETS ${PROJECT_NAME}
    # RUNTIME DESTINATION ${PACPUSBIN_DIRECTORY}
    # LIBRARY DESTINATION ${PACPUSBIN_DIRECTORY}
    # ARCHIVE DESTINATION ${PACPUSLIB_DIRECTORY}
# )

# if(MSVC)
    # set(TARGET_PATH "${CMAKE_CURRENT_BINARY_DIR}/Debug/${PROJECT_NAME}_d.pdb")
    # if(EXISTS ${TARGET_PATH})
        # install(FILES ${TARGET_PATH}
            # DESTINATION ${PACPUSLIB_DIRECTORY}
        # )
    # endif()

    # set(TARGET_PATH "${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo/${PROJECT_NAME}.pdb")
    # if(EXISTS ${TARGET_PATH})
        # install(FILES ${TARGET_PATH}
            # DESTINATION ${PACPUSLIB_DIRECTORY}
        # )
    # endif()
# endif()

# install(DIRECTORY include/
    # DESTINATION ${PACPUSDEV_INCLUDE_DIR}/extlib/NMEA0183
    # PATTERN ".svn" EXCLUDE
    # PATTERN "*~" EXCLUDE
# )
