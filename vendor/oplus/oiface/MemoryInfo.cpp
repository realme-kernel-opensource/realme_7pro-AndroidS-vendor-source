#include <binder/IServiceManager.h>
#include <utils/RefBase.h>
#include <binder/Parcel.h>

#include "MemoryInfo.h"
#include "OIface.h"
using namespace android;
using namespace std;

#define RETURN_ON_ERROR(X) \
    do { \
        status_t res = (X); \
        if (res != NO_ERROR) \
            return res; \
    } while(false)

status_t SystemMemInfo::writeToParcel(Parcel* parcel) const{
    RETURN_ON_ERROR(parcel->writeInt64(availMem));
    RETURN_ON_ERROR(parcel->writeInt64(totalMem));
    return 0;
}

status_t MemoryInfo::writeToParcel(Parcel* parcel) const{
    DEBUG("calling writeToParcel");
    parcel->writeInt32(dalvikPss);
    parcel->writeInt32(dalvikSwappablePss);
    parcel->writeInt32(dalvikPrivateDirty);
    parcel->writeInt32(dalvikSharedDirty);
    parcel->writeInt32(dalvikPrivateClean);
    parcel->writeInt32(dalvikSharedClean);
    parcel->writeInt32(dalvikSwappedOut);
    parcel->writeInt32(dalvikSwappedOutPss);
    parcel->writeInt32(nativePss);
    parcel->writeInt32(nativeSwappablePss);
    parcel->writeInt32(nativePrivateDirty);
    parcel->writeInt32(nativeSharedDirty);
    parcel->writeInt32(nativePrivateClean);
    parcel->writeInt32(nativeSharedClean);
    parcel->writeInt32(nativeSwappedOut);
    parcel->writeInt32(nativeSwappedOutPss);
    parcel->writeInt32(otherPss);
    parcel->writeInt32(otherSwappablePss);
    parcel->writeInt32(otherPrivateDirty);
    parcel->writeInt32(otherSharedDirty);
    parcel->writeInt32(otherPrivateClean);
    parcel->writeInt32(otherSharedClean);
    parcel->writeInt32(otherSwappedOut);
    parcel->writeInt32(hasSwappedOutPss ? 1 : 0);
    parcel->writeInt32(otherSwappedOutPss);
    parcel->writeInt32Array(ARRAY_SIZE(otherStats), otherStats);

    return 0;
}

status_t SystemMemInfo::readFromParcel(const Parcel* parcel) {
    availMem = parcel->readInt64();
    totalMem = parcel->readInt64();

    return 0;
}

status_t MemoryInfo::readFromParcel(const Parcel* parcel) {
    DEBUG("calling readFromParcel");
    dalvikPss = parcel->readInt32();
    dalvikSwappablePss = parcel->readInt32();
    dalvikPrivateDirty = parcel->readInt32();
    dalvikSharedDirty = parcel->readInt32();
    dalvikPrivateClean = parcel->readInt32();
    dalvikSharedClean = parcel->readInt32();
    dalvikSwappedOut = parcel->readInt32();
    dalvikSwappedOutPss = parcel->readInt32();
    nativePss = parcel->readInt32();
    nativeSwappablePss = parcel->readInt32();
    nativePrivateDirty = parcel->readInt32();
    nativeSharedDirty = parcel->readInt32();
    nativePrivateClean = parcel->readInt32();
    nativeSharedClean = parcel->readInt32();
    nativeSwappedOut = parcel->readInt32();
    nativeSwappedOutPss = parcel->readInt32();
    otherPss = parcel->readInt32();
    otherSwappablePss = parcel->readInt32();
    otherPrivateDirty = parcel->readInt32();
    otherSharedDirty = parcel->readInt32();
    otherPrivateClean = parcel->readInt32();
    otherSharedClean = parcel->readInt32();
    otherSwappedOut = parcel->readInt32();
    hasSwappedOutPss = parcel->readInt32() != 0;
    otherSwappedOutPss = parcel->readInt32();
    int32_t sz = parcel->readInt32();
    for (int i = 0; (i < (int)ARRAY_SIZE(otherStats)) && (i < sz); i++) { 
        otherStats[i] = parcel->readInt32();
    }

    return 0;
}

string MemoryInfo::getMemoryStat(string statName) {
        if (statName == "summary.java-heap")
            return to_string(getSummaryJavaHeap());
        if (statName == "summary.native-heap")
            return to_string(getSummaryNativeHeap());
        if (statName ==  "summary.code")
            return to_string(getSummaryCode());
        if (statName ==  "summary.stack")
            return to_string(getSummaryStack());
        if (statName ==  "summary.graphics")
            return to_string(getSummaryGraphics());
        if (statName == "summary.private-other")
            return to_string(getSummaryPrivateOther());
        if (statName == "summary.system")
            return to_string(getSummarySystem());
        if (statName == "summary.total-pss")
            return to_string(getSummaryTotalPss());
        if (statName == "summary.total-swap")
            return to_string(getSummaryTotalSwap());

        return "";
}

/**
 * Returns a map of the names/values of the memory statistics
 * that {@link #getMemoryStat(string)} supports.
 *
 * @return a map of the names/values of the supported memory statistics.
 */
map<string, string> MemoryInfo::getMemoryStats() {
    map<string, string> stats;
    stats["summary.java-heap"] = to_string(getSummaryJavaHeap());
    stats["summary.native-heap"] = to_string(getSummaryNativeHeap());
    stats["summary.code"] = to_string(getSummaryCode());
    stats["summary.stack"] = to_string(getSummaryStack());
    stats["summary.graphics"] = to_string(getSummaryGraphics());
    stats["summary.private-other"] = to_string(getSummaryPrivateOther());
    stats["summary.system"] = to_string(getSummarySystem());
    stats["summary.total-pss"] = to_string(getSummaryTotalPss());
    stats["summary.total-swap"] = to_string(getSummaryTotalSwap());

    return stats;
}

/**
 * Pss of Java Heap bytes in KB due to the application.
 * Notes:
 *  * OTHER_ART is the boot image. Anything private here is blamed on
 *    the application, not the system.
 *  * dalvikPrivateDirty includes private zygote, which means the
 *    application dirtied something allocated by the zygote. We blame
 *    the application for that memory, not the system.
 *  * Does not include OTHER_DALVIK_OTHER, which is considered VM
 *    Overhead and lumped into Private Other.
 *  * We don't include dalvikPrivateClean, because there should be no
 *    such thing as private clean for the Java Heap.
 * @hide
 */
int MemoryInfo::getSummaryJavaHeap() {
    return dalvikPrivateDirty + getOtherPrivate(OTHER_ART);
}

/**
 * Pss of Native Heap bytes in KB due to the application.
 * Notes:
 *  * Includes private dirty malloc space.
 *  * We don't include nativePrivateClean, because there should be no
 *    such thing as private clean for the Native Heap.
 * @hide
 */
int MemoryInfo::getSummaryNativeHeap() {
    return nativePrivateDirty;
}

/**
 * Pss of code and other static resource bytes in KB due to
 * the application.
 * @hide
 */
int MemoryInfo::getSummaryCode() {
    return getOtherPrivate(OTHER_SO)
        + getOtherPrivate(OTHER_JAR)
        + getOtherPrivate(OTHER_APK)
        + getOtherPrivate(OTHER_TTF)
        + getOtherPrivate(OTHER_DEX)
        + getOtherPrivate(OTHER_OAT);
}

/**
 * Pss in KB of the stack due to the application.
 * Notes:
 *  * Includes private dirty stack, which includes both Java and Native
 *    stack.
 *  * Does not include private clean stack, because there should be no
 *    such thing as private clean for the stack.
 * @hide
 */
int MemoryInfo::getSummaryStack() {
    return getOtherPrivateDirty(OTHER_STACK);
}

/**
 * Pss in KB of graphics due to the application.
 * Notes:
 *  * Includes private Gfx, EGL, and GL.
 *  * Warning: These numbers can be misreported by the graphics drivers.
 *  * We don't include shared graphics. It may make sense to, because
 *    shared graphics are likely buffers due to the application
 *    anyway, but it's simpler to implement to just group all shared
 *    memory into the System category.
 * @hide
 */
int MemoryInfo::getSummaryGraphics() {
    return getOtherPrivate(OTHER_GL_DEV)
        + getOtherPrivate(OTHER_GRAPHICS)
        + getOtherPrivate(OTHER_GL);
}

/**
 * Pss in KB due to the application that haven't otherwise been
 * accounted for.
 * @hide
 */
int MemoryInfo::getSummaryPrivateOther() {
    return getTotalPrivateClean()
        + getTotalPrivateDirty()
        - getSummaryJavaHeap()
        - getSummaryNativeHeap()
        - getSummaryCode()
        - getSummaryStack()
        - getSummaryGraphics();
}

/**
 * Pss in KB due to the system.
 * Notes:
 *  * Includes all shared memory.
 * @hide
 */
int MemoryInfo::getSummarySystem() {
    return getTotalPss()
        - getTotalPrivateClean()
        - getTotalPrivateDirty();
}

/**
 * Total Pss in KB.
 * @hide
 */
int MemoryInfo::getSummaryTotalPss() {
    return getTotalPss();
}

/**
 * Total Swap in KB.
 * Notes:
 *  * Some of this memory belongs in other categories, but we don't
 *    know if the Swap memory is shared or private, so we don't know
 *    what to blame on the application and what on the system.
 *    For now, just lump all the Swap in one place.
 *    For kernels reporting SwapPss {@link #getSummaryTotalSwapPss()}
 *    will report the application proportional Swap.
 * @hide
 */
int MemoryInfo::getSummaryTotalSwap() {
    return getTotalSwappedOut();
}

/**
 * Total proportional Swap in KB.
 * Notes:
 *  * Always 0 if {@link #hasSwappedOutPss} is false.
 * @hide
 */
int MemoryInfo::getSummaryTotalSwapPss() {
    return getTotalSwappedOutPss();
}

int MemoryInfo::describeContents() {
    return 0;
}

int SystemMemInfo::getSystemMem(long &total, long &avail) {
    SystemMemInfo info;

    sp<IServiceManager> sm = defaultServiceManager();
    if (sm == NULL) {
        ERROR("Unable to get default service manager!");
        return -1;
    }

    sp<IBinder> service = sm->checkService(String16("activity"));
    if (service == NULL) {
        ERROR("unable to get activity service");
        return -1;
    }

    Parcel data, reply;
    data.writeInterfaceToken(String16("android.app.IActivityManager"));

#ifdef PREQ
    static const int GET_MEMINFO_TRANSACTION_CODE = 74;
#else
    static const int GET_MEMINFO_TRANSACTION_CODE = 65;
#endif
    status_t result = service->transact(GET_MEMINFO_TRANSACTION_CODE, data, &reply);
    if (result != NO_ERROR) {
        ERROR("%s, failed to transact: %d", __func__, result);
    }

    if (reply.readExceptionCode()) {
        ERROR("get memory info exception code error");
        return -1;
    }

    if (reply.readParcelable(&info) < 0) {
        ERROR("read from parcel failed");
        return -1;
    }

    avail = info.getAvailMem();
    total = info.getTotalMem();
    return 0;
}

int MemoryInfo::getProcessMemoryInfo(vector<MemoryInfo> *info) {
    return getProcessMemoryInfo(info, getpid());
}

int MemoryInfo::getProcessMemoryInfo(vector<MemoryInfo> *info, int pid) {
    if (info == NULL) {
        ERROR("invalid info argument");
        return -1;
    }
    sp<IServiceManager> sm = defaultServiceManager();
    if (sm == NULL) {
        ERROR("Unable to get default service manager!");
        return -1;
    }

    sp<IBinder> service = sm->checkService(String16("activity"));
    if (service == NULL) {
        ERROR("unable to get activity service");
        return -1;
    }

    Parcel data, reply;
    data.writeInterfaceToken(String16("android.app.IActivityManager"));
    RETURN_ON_ERROR(data.writeInt32Vector({pid}));

#if defined(PREP)
    static const int GET_PROCESS_MEMORY_INFO_TRANSACT_CODE = 92;
#else
    static const int GET_PROCESS_MEMORY_INFO_TRANSACT_CODE = 82;
#endif
    status_t result = service->transact(GET_PROCESS_MEMORY_INFO_TRANSACT_CODE, data, &reply);
    if (result != NO_ERROR) {
        ERROR("%s, failed to transact: %d", __func__, result);
    }

    if (reply.readExceptionCode()) {
        ERROR("get memory info exception code error");
        return -1;
    }
    if (reply.readParcelableVector(info) < 0) {
        ERROR("read from parcel failed");
        return -1;
    }

    return 0;

}

/**
 * Return total PSS memory usage in kB.
 */
int MemoryInfo::getTotalPss() {
    return dalvikPss + nativePss + otherPss + getTotalSwappedOutPss();
}

/**
 * @hide Return total PSS memory usage in kB.
 */
int MemoryInfo::getTotalUss() {
    return dalvikPrivateClean + dalvikPrivateDirty
        + nativePrivateClean + nativePrivateDirty
        + otherPrivateClean + otherPrivateDirty;
}

/**
 * Return total PSS memory usage in kB mapping a file of one of the following extension:
 * .so, .jar, .apk, .ttf, .dex, .odex, .oat, .art .
 */
int MemoryInfo::getTotalSwappablePss() {
    return dalvikSwappablePss + nativeSwappablePss + otherSwappablePss;
}

/**
 * Return total private dirty memory usage in kB.
 */
int MemoryInfo::getTotalPrivateDirty() {
    return dalvikPrivateDirty + nativePrivateDirty + otherPrivateDirty;
}

/**
 * Return total shared dirty memory usage in kB.
 */
int MemoryInfo::getTotalSharedDirty() {
    return dalvikSharedDirty + nativeSharedDirty + otherSharedDirty;
}

/**
 * Return total shared clean memory usage in kB.
 */
int MemoryInfo::getTotalPrivateClean() {
    return dalvikPrivateClean + nativePrivateClean + otherPrivateClean;
}

/**
 * Return total shared clean memory usage in kB.
 */
int MemoryInfo::getTotalSharedClean() {
    return dalvikSharedClean + nativeSharedClean + otherSharedClean;
}

/**
 * Return total swapped out memory in kB.
 * @hide
 */
int MemoryInfo::getTotalSwappedOut() {
    return dalvikSwappedOut + nativeSwappedOut + otherSwappedOut;
}

/**
 * Return total swapped out memory in kB, proportional.
 * @hide
 */
int MemoryInfo::getTotalSwappedOutPss() {
    return dalvikSwappedOutPss + nativeSwappedOutPss + otherSwappedOutPss;
}

/** @hide */
int MemoryInfo::getOtherPss(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetPss];
}


/** @hide */
int MemoryInfo::getOtherSwappablePss(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetSwappablePss];
}


/** @hide */
int MemoryInfo::getOtherPrivateDirty(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetPrivateDirty];
}

/** @hide */
int MemoryInfo::getOtherSharedDirty(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetSharedDirty];
}

/** @hide */
int MemoryInfo::getOtherPrivateClean(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetPrivateClean];
}

/** @hide */
int MemoryInfo::getOtherPrivate(int which) {
    return getOtherPrivateClean(which) + getOtherPrivateDirty(which);
}

/** @hide */
int MemoryInfo::getOtherSharedClean(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetSharedClean];
}

/** @hide */
int MemoryInfo::getOtherSwappedOut(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetSwappedOut];
}

/** @hide */
int MemoryInfo::getOtherSwappedOutPss(int which) {
    return otherStats[which*NUM_CATEGORIES + offsetSwappedOutPss];
}

/** @hide */
string MemoryInfo::getOtherLabel(int which) {
    switch (which) {
        case OTHER_DALVIK_OTHER: return "Dalvik Other";
        case OTHER_STACK: return "Stack";
        case OTHER_CURSOR: return "Cursor";
        case OTHER_ASHMEM: return "Ashmem";
        case OTHER_GL_DEV: return "Gfx dev";
        case OTHER_UNKNOWN_DEV: return "Other dev";
        case OTHER_SO: return ".so mmap";
        case OTHER_JAR: return ".jar mmap";
        case OTHER_APK: return ".apk mmap";
        case OTHER_TTF: return ".ttf mmap";
        case OTHER_DEX: return ".dex mmap";
        case OTHER_OAT: return ".oat mmap";
        case OTHER_ART: return ".art mmap";
        case OTHER_UNKNOWN_MAP: return "Other mmap";
        case OTHER_GRAPHICS: return "EGL mtrack";
        case OTHER_GL: return "GL mtrack";
        case OTHER_OTHER_MEMTRACK: return "Other mtrack";
        case OTHER_DALVIK_NORMAL: return ".Heap";
        case OTHER_DALVIK_LARGE: return ".LOS";
        case OTHER_DALVIK_ZYGOTE: return ".Zygote";
        case OTHER_DALVIK_NON_MOVING: return ".NonMoving";
        case OTHER_DALVIK_OTHER_LINEARALLOC: return ".LinearAlloc";
        case OTHER_DALVIK_OTHER_ACCOUNTING: return ".GC";
        case OTHER_DALVIK_OTHER_CODE_CACHE: return ".JITCache";
        case OTHER_DALVIK_OTHER_COMPILER_METADATA: return ".CompilerMetadata";
        case OTHER_DALVIK_OTHER_INDIRECT_REFERENCE_TABLE: return ".IndirectRef";
        case OTHER_DEX_BOOT_VDEX: return ".Boot vdex";
        case OTHER_DEX_APP_DEX: return ".App dex";
        case OTHER_DEX_APP_VDEX: return ".App vdex";
        case OTHER_ART_APP: return ".App art";
        case OTHER_ART_BOOT: return ".Boot art";
        default: return "????";
    }
}


