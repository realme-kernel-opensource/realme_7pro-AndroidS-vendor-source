LOCAL_PATH := $(call my-dir)
# OIFACE_BUILD_DATE := $(shell date  "+%Y%m%d%H%M")
GIT_VERSION := $(shell git --git-dir=$(LOCAL_PATH)/.git rev-parse --short HEAD)
ifneq ($(strip $(shell cd $(LOCAL_PATH) && git status -uno -s .)),)
    $(warning GIT STATUS IS DIRTY, PLEASE DONT UPLOAD COMPILED OUTPUT TO GERRIT.SCM.ADC.COM UNTIL UPLOAD TO SH GERRIT)
    GIT_VERSION := $(GIT_VERSION)-dirty
endif
LAST_COMMIT := "$(shell git --git-dir=$(LOCAL_PATH)/.git log --pretty=format:%s -n 1 | sed 's/'\''//g')"

PREM := 0
#Setup oiface configuration
ifeq ($(shell test $(PLATFORM_SDK_VERSION) -lt 26 && echo PreOreo),PreOreo)
    ifeq ($(shell test $(PLATFORM_SDK_VERSION) -lt 23 && echo PreM),PreM)
        USE_NEO := 0
        USE_OIFACE_SERVICE := 1
        PREM := 1
    else
        USE_NEO := 1
        USE_OIFACE_SERVICE := 0
    endif
else
    USE_NEO := 0
    USE_OIFACE_SERVICE := 0
endif

OIFACE_YACC_FLAGS := -v

#CPP flags
OIFACE_CPPFLAGS :=  -Wno-unused-parameter \
                    -Wno-deprecated-register \
                    -Wno-write-strings \
                    -Wno-unused-function \
                    -Wno-unneeded-internal-declaration \
                    -Wno-implicit-fallthrough \

# cflags configuration
OIFACE_CFLAGS := -DOIFACE_VERSION=\"$(shell git --git-dir=$(LOCAL_PATH)/.git for-each-ref refs/tags --sort=-committerdate --format=%\(refname\) --count=0  | grep 'oiface\/[0-9]\+.[0-9]\+' | cut -f 4 -d '/' | sort -r | head -n1)\"
OIFACE_CFLAGS += -DLOG_TAG=\"oiface\"
OIFACE_CFLAGS += -DJSON_USE_EXCEPTION=0 -DFEATURE_ANDROID
OIFACE_CFLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"
OIFACE_CFLAGS += -DANDROID_VERSION=$(PLATFORM_SDK_VERSION)
# OIFACE_CFLAGS += -DBUILD_DATE=$(OIFACE_BUILD_DATE)

ifeq ($(USE_NEO),1)
OIFACE_CFLAGS += -DUSE_NEO
OIFACE_CFLAGS += -DOIFACE_LOG_PROPERTY=\"persist.sys.neo.loglevel\"
else
OIFACE_CFLAGS += -DOIFACE_LOG_PROPERTY=\"persist.sys.oiface.loglevel\"
endif

ifeq ($(USE_OIFACE_SERVICE),1)
OIFACE_CFLAGS += -DUSE_OIFACE_SERVICE
endif

ifeq ($(PREM),1)
OIFACE_CFLAGS += -DPREM
endif

ifeq ($(shell test $(PLATFORM_SDK_VERSION) -lt 28; echo $$?),0)
OIFACE_CFLAGS += -DPREP
endif

ifeq ($(shell test $(PLATFORM_VERSION) -lt 10; echo $$?),0)
OIFACE_CFLAGS += -DPREQ
endif

# Libraries
OIFACE_SHARED_LIBRARIES := liblog libcutils libcrypto libbinder libutils libgui
ifeq ($(shell test $(PLATFORM_SDK_VERSION) -ge 28 && echo PLater),PLater)

#FIXME
OIFACE_HILD := 0
OIFACE_SHARED_LIBRARIES += libhardware \
                           libhidlbase \
                           libhidltransport \
                           libhwbinder \
                           libbsproxy \
                           libbase \
                           android.hardware.health@2.0 \
                           android.hardware.health@1.0 \
                           android.hardware.thermal@1.0 \
                           vendor.oplus.hardware.charger@1.0 \
                           vendor.oplus.hardware.stability.oplus_project@1.0 \
                           vendor.oplus.hardware.touch@1.0 \
                           vendor.oplus.hardware.performance@1.0\
                           vendor.oplus.hardware.gameopt@1.0 \

# OIFACE_CFLAGS += -DOIFACE_HIDL
else
OIFACE_HILD := 0
endif

OIFACE_STATIC_LIBRARIES := libjsoncpp libhealthhalutils

# Includes
OIFACE_C_INCLUDES := $(LOCAL_PATH)/../framework/common/inc \
                      vendor/oplus/frameworks/native_common/libs/bsproxy \

# Android M and later includes is generated automatically
ifeq ($(PREM),1)
OIFACE_SHARED_LIBRARIES += liblog libdl libc++

OIFACE_C_INCLUDES += external/libcxx/include \
                     external/openssl/include \
                     external/jsoncpp/include \
                     $(TARGET_OUT_HEADERS)

OIFACE_CPPFLAGS += -std=c++11
endif

OIFACE_SRCS :=  \
           HoraeServiceProxy.cpp \
           OIface.cpp \
           DecisionDriver.cpp \
           PlatformAdaptor.cpp \
           OIfaceModuleLoader.cpp \
           Utils.cpp \
           ThreadState.cpp \
           OIfaceServer.cpp \
           EngineParser.cpp \
           JsonParser.cpp \
           OimParser.cpp \
           IOService.cpp \
           TimedJob.cpp \
           AffinityService.cpp \
           LogSink.cpp \
           ActivityServiceProxy.cpp \
           OIfaceClient.cpp \
           GlobalConfig.cpp \
           BinderManager.cpp \
           BinderClient.cpp \
           GeneralClient.cpp \
           GeneralCheck.cpp \
           DataCollector.cpp \
           MemoryInfo.cpp \
           NetworkLatencyManager.cpp \
           GameStatusObserverManager.cpp \
           FrameRescuer.cpp \
           HealthListener.cpp \
           ThermalService.cpp \
           SurfaceFlingerProxyManager.cpp \
           util/KeyThreadTracer.cpp \
           util/OrmsProxy.cpp \
           util/FrameStatsTracer.cpp\
           util/DataReader.cpp\
           ScreenModeProxy.cpp \
           OifaceCallbackManager.cpp \
           feature/gpa/ConfigMap.cpp \
           feature/gpa/FsWrapper.cpp \
           feature/gpa/PropReader.cpp \
           feature/gpa/SysConfig.cpp \
           feature/gpa/SystemBase.cpp \
           feature/gpa/SystemPlatform.cpp \
           feature/gpa/Utils.cpp \
           feature/gpa/GpaReader.cpp \
           CooExxManager.cpp \
           TaskMonitor.cpp \
           TaskManager.cpp \
           ChargerService.cpp \
           GameListManager.cpp \
           OplusProjectService.cpp \
           OplusTouchService.cpp \
           OplusOIfaceService.cpp \
           LightningStartManager.cpp \
           PerformanceHalService.cpp \
           FrameStateManager.cpp \
           GameOptService.cpp


# FIXME
ifeq ($(OIFACE_HILD),1)
#OIFACE_SRCS += OIfaceHalService.cpp
endif

ifeq ($(USE_OIFACE_SERVICE),1)
OIFACE_CFLAGS += -DUSE_OIFACE_SERVICE
endif

ifeq ($(USE_NEO),1)
OIFACE_SRCS += ../framework/common/src/log_base.cpp
endif

############## liboiface.so #############
include $(CLEAR_VARS)

LOCAL_SRC_FILES := $(OIFACE_SRCS)
LOCAL_CPPFLAGS := $(OIFACE_CPPFLAGS)
LOCAL_CFLAGS := $(OIFACE_CFLAGS)
LOCAL_C_INCLUDES := $(OIFACE_C_INCLUDES)
LOCAL_SHARED_LIBRARIES := $(OIFACE_SHARED_LIBRARIES)
LOCAL_STATIC_LIBRARIES := $(OIFACE_STATIC_LIBRARIES)
LOCAL_MODULE := liboiface
LOCAL_MULTILIB := 64
LOCAL_SYSTEM_EXT_MODULE := true
LOCAL_EUCLID_CLASS := system_ext

include $(BUILD_SHARED_LIBRARY)

############## oiface for one plus #############
include $(CLEAR_VARS)


LOCAL_SRC_FILES := $(OIFACE_SRCS) \
                   OIfaceDaemon.cpp
LOCAL_CPPFLAGS := $(OIFACE_CPPFLAGS)
LOCAL_CFLAGS := $(OIFACE_CFLAGS)
LOCAL_C_INCLUDES := $(OIFACE_C_INCLUDES)
LOCAL_SHARED_LIBRARIES := $(OIFACE_SHARED_LIBRARIES)
LOCAL_STATIC_LIBRARIES := $(OIFACE_STATIC_LIBRARIES)
LOCAL_MODULE := oifaceoplusonly
LOCAL_MULTILIB := 64
LOCAL_SYSTEM_EXT_MODULE := true
LOCAL_EUCLID_CLASS := system_ext
LOCAL_INIT_RC := oifaceoplusonly.rc
include $(BUILD_EXECUTABLE)


############ oiface ###########
include $(CLEAR_VARS)

OIFACE_CFLAGS += -DOPLUS_FEATURE_OPPO_OIFACE

LOCAL_SRC_FILES := $(OIFACE_SRCS) \
                   OIfaceService.cpp \
                   OIfaceDaemon.cpp
LOCAL_CPPFLAGS := $(OIFACE_CPPFLAGS)
LOCAL_CFLAGS := $(OIFACE_CFLAGS)
LOCAL_C_INCLUDES := $(OIFACE_C_INCLUDES)
LOCAL_SHARED_LIBRARIES := $(OIFACE_SHARED_LIBRARIES)
LOCAL_STATIC_LIBRARIES := $(OIFACE_STATIC_LIBRARIES)
LOCAL_MODULE := oiface
LOCAL_MULTILIB := 64
LOCAL_SYSTEM_EXT_MODULE := true
LOCAL_EUCLID_CLASS := system_ext
LOCAL_INIT_RC := oiface.rc
include $(BUILD_EXECUTABLE)

$(warning mkdir $(TARGET_OUT_GEN))
$(shell mkdir -p $(TARGET_OUT_GEN)/oiface_update/files)
$(shell mkdir -p $(TARGET_OUT_GEN)/oiface_update/upload)

include $(call all-makefiles-under,$(LOCAL_PATH))
