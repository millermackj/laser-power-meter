#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/PIC_serial.o ${OBJECTDIR}/PID.o ${OBJECTDIR}/functions.o ${OBJECTDIR}/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/PIC_serial.o.d ${OBJECTDIR}/PID.o.d ${OBJECTDIR}/functions.o.d ${OBJECTDIR}/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/PIC_serial.o ${OBJECTDIR}/PID.o ${OBJECTDIR}/functions.o ${OBJECTDIR}/main.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

# The following macros may be used in the pre and post step lines
Device=dsPIC33FJ64MC202
ProjectDir=/Volumes/Overflow\ HDD/Dropbox/school/school\ fall\ 12/MIE\ 415/Prima-Electro_Project/Prima-Electro\ Project\ Shared/Google-Drive/firmware/laser-power-meter/LaserPowerMeter_v1
ConfName=default
ImagePath=dist/default/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
ImageDir=dist/default/${IMAGE_TYPE}
ImageName=LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

.build-conf:  .pre ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
	@echo "--------------------------------------"
	@echo "User defined post-build step: []"
	@
	@echo "--------------------------------------"

MP_PROCESSOR_OPTION=33FJ64MC202
MP_LINKER_FILE_OPTION=,-Tp33FJ64MC202.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/PIC_serial.o: PIC_serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PIC_serial.o.d 
	@${RM} ${OBJECTDIR}/PIC_serial.o.ok ${OBJECTDIR}/PIC_serial.o.err 
	@${FIXDEPS} "${OBJECTDIR}/PIC_serial.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/PIC_serial.o.d" -o ${OBJECTDIR}/PIC_serial.o PIC_serial.c    
	
${OBJECTDIR}/PID.o: PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PID.o.d 
	@${RM} ${OBJECTDIR}/PID.o.ok ${OBJECTDIR}/PID.o.err 
	@${FIXDEPS} "${OBJECTDIR}/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/PID.o.d" -o ${OBJECTDIR}/PID.o PID.c    
	
${OBJECTDIR}/functions.o: functions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/functions.o.d 
	@${RM} ${OBJECTDIR}/functions.o.ok ${OBJECTDIR}/functions.o.err 
	@${FIXDEPS} "${OBJECTDIR}/functions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/functions.o.d" -o ${OBJECTDIR}/functions.o functions.c    
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o.ok ${OBJECTDIR}/main.o.err 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c    
	
else
${OBJECTDIR}/PIC_serial.o: PIC_serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PIC_serial.o.d 
	@${RM} ${OBJECTDIR}/PIC_serial.o.ok ${OBJECTDIR}/PIC_serial.o.err 
	@${FIXDEPS} "${OBJECTDIR}/PIC_serial.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/PIC_serial.o.d" -o ${OBJECTDIR}/PIC_serial.o PIC_serial.c    
	
${OBJECTDIR}/PID.o: PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PID.o.d 
	@${RM} ${OBJECTDIR}/PID.o.ok ${OBJECTDIR}/PID.o.err 
	@${FIXDEPS} "${OBJECTDIR}/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/PID.o.d" -o ${OBJECTDIR}/PID.o PID.c    
	
${OBJECTDIR}/functions.o: functions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/functions.o.d 
	@${RM} ${OBJECTDIR}/functions.o.ok ${OBJECTDIR}/functions.o.err 
	@${FIXDEPS} "${OBJECTDIR}/functions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/functions.o.d" -o ${OBJECTDIR}/functions.o functions.c    
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o.ok ${OBJECTDIR}/main.o.err 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -I".." -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=1024,-L"..",-Map="${DISTDIR}/MIE597G_Group6_Lab6.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PICKIT2=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=1024,-L"..",-Map="${DISTDIR}/MIE597G_Group6_Lab6.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}/pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/LaserPowerMeter_v1.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif

.pre:
	@echo "--------------------------------------"
	@echo "User defined pre-build step: []"
	@
	@echo "--------------------------------------"

# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
