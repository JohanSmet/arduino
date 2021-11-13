
# arduino configuration
FQBN ?= arduino:avr:mega
SERIAL ?= /dev/ttyACM0

BUILD_DIR 	:= $(subst :,_,$(PWD)/_build/${FQBN})
SRC 		:= $(wildcard *.ino)
ELF 		:= ${BUILD_DIR}/${SRC}.elf

# output information about the build
$(info FQBN: ${FQBN})
$(info serial port: ${SERIAL})
$(info build dir: ${BUILD_DIR})
$(info sketch: ${SRC})

.PHONY: all upload clean

all: ${ELF} upload

${ELF}: ${SRC}
	arduino-cli compile --fqbn ${FQBN} ${SRC} --build-path ${BUILD_DIR}

upload:
	arduino-cli upload --fqbn ${FQBN} -p ${SERIAL} --input-dir ${BUILD_DIR}

clean:
	rm -rf ${BUILD_DIR}
