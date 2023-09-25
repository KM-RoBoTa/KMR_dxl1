# Check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    message("Doxygen found")

    # Create output directory
    set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated_docs)
	file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

	# Set Doxygen input and output directories
	set(DOXYGEN_INPUT_DIR ${CMAKE_SOURCE_DIR})  #not used atm
    set(DOXYGEN_IMG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/img)
	set(DOXYGEN_AWESOME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doxygen-awesome)

    # Set input and output files to generate Doxyfile
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_SOURCE_DIR}/../docs/generated_docs/Doxyfile)

    # Request to configure the file: create Doxyfile from Doxyfile.in
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # Note the option ALL which allows to build the docs together with the application
    add_custom_target(docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../docs/generated_docs
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


# use:
# mkdir build && cd build
# cmake -DBUILD_DOCS=ON ../
# make docs
# cmake --build .