#ifndef __MEMORY_INFO_H__
#define __MEMORY_INFO_H__
#include <binder/Parcelable.h>
#include <utils/String16.h>

class SystemMemInfo: public  android::Parcelable {
    public:
        virtual android::status_t writeToParcel(android::Parcel* parcel) const;
        virtual android::status_t readFromParcel(const android::Parcel* parcel);
        /* get total and avail ram in KB */
        static int getSystemMem(long &total, long &avail);
        int64_t getAvailMem() {return availMem;};
        int64_t getTotalMem() {return totalMem;};
    private:
        int64_t availMem;
        int64_t totalMem;
};

class MemoryInfo: public android::Parcelable {
public:

    virtual android::status_t writeToParcel(android::Parcel* parcel) const;
    virtual android::status_t readFromParcel(const android::Parcel* parcel);
    static MemoryInfo getProcessMemoryInfo();
    static int getProcessMemoryInfo(std::vector<MemoryInfo> *info, int pid);
    static int getProcessMemoryInfo(std::vector<MemoryInfo> *info);
    std::string getMemoryStat(std::string statName);
    std::map<std::string, std::string> getMemoryStats();
    int getSummaryJavaHeap();
    int getSummaryNativeHeap();
    int getSummaryCode();
    int getSummaryStack();
    int getSummaryGraphics();
    int getSummaryPrivateOther();
    int getSummarySystem();
    int getSummaryTotalPss();
    int getSummaryTotalSwap();
    int getSummaryTotalSwapPss();
    int describeContents();
    int getTotalPss();
    int getTotalUss();
    int getTotalSwappablePss();
    int getTotalPrivateDirty();
    int getTotalSharedDirty();
    int getTotalPrivateClean();
    int getTotalSharedClean();
    int getTotalSwappedOut();
    int getTotalSwappedOutPss();
    int getOtherPss(int which);
    int getOtherSwappablePss(int which);
    int getOtherPrivateDirty(int which);
    int getOtherSharedDirty(int which);
    int getOtherPrivateClean(int which);
    int getOtherPrivate(int which);
    int getOtherSharedClean(int which);
    int getOtherSwappedOut(int which);
    int getOtherSwappedOutPss(int which);
    static std::string getOtherLabel(int which);

private:
    static const int HEAP_UNKNOWN = 0;
    static const int HEAP_DALVIK = 1;
    static const int HEAP_NATIVE = 2;

    static const int OTHER_DALVIK_OTHER = 0;
    static const int OTHER_STACK = 1;
    static const int OTHER_CURSOR = 2;
    static const int OTHER_ASHMEM = 3;
    static const int OTHER_GL_DEV = 4;
    static const int OTHER_UNKNOWN_DEV = 5;
    static const int OTHER_SO = 6;
    static const int OTHER_JAR = 7;
    static const int OTHER_APK = 8;
    static const int OTHER_TTF = 9;
    static const int OTHER_DEX = 10;
    static const int OTHER_OAT = 11;
    static const int OTHER_ART = 12;
    static const int OTHER_UNKNOWN_MAP = 13;
    static const int OTHER_GRAPHICS = 14;
    static const int OTHER_GL = 15;
    static const int OTHER_OTHER_MEMTRACK = 16;

    // Needs to be declared here for the DVK_STAT ranges below.
    static const int NUM_OTHER_STATS = 17;

    // Dalvik subsections.
    static const int OTHER_DALVIK_NORMAL = 17;
    static const int OTHER_DALVIK_LARGE = 18;
    static const int OTHER_DALVIK_ZYGOTE = 19;
    static const int OTHER_DALVIK_NON_MOVING = 20;
    // Section begins and ends for dumpsys, relative to the DALVIK categories.
    static const int OTHER_DVK_STAT_DALVIK_START =
        OTHER_DALVIK_NORMAL - NUM_OTHER_STATS;
    static const int OTHER_DVK_STAT_DALVIK_END =
        OTHER_DALVIK_NON_MOVING - NUM_OTHER_STATS;

    // Dalvik Other subsections.
    static const int OTHER_DALVIK_OTHER_LINEARALLOC = 21;
    static const int OTHER_DALVIK_OTHER_ACCOUNTING = 22;
    static const int OTHER_DALVIK_OTHER_CODE_CACHE = 23;
    static const int OTHER_DALVIK_OTHER_COMPILER_METADATA = 24;
    static const int OTHER_DALVIK_OTHER_INDIRECT_REFERENCE_TABLE = 25;
    static const int OTHER_DVK_STAT_DALVIK_OTHER_START =
        OTHER_DALVIK_OTHER_LINEARALLOC - NUM_OTHER_STATS;
    static const int OTHER_DVK_STAT_DALVIK_OTHER_END =
        OTHER_DALVIK_OTHER_INDIRECT_REFERENCE_TABLE - NUM_OTHER_STATS;

    // Dex subsections (Boot vdex, App dex, and App vdex).
    static const int OTHER_DEX_BOOT_VDEX = 26;
    static const int OTHER_DEX_APP_DEX = 27;
    static const int OTHER_DEX_APP_VDEX = 28;
    static const int OTHER_DVK_STAT_DEX_START = OTHER_DEX_BOOT_VDEX - NUM_OTHER_STATS;
    static const int OTHER_DVK_STAT_DEX_END = OTHER_DEX_APP_VDEX - NUM_OTHER_STATS;

    // Art subsections (App image, boot image).
    static const int OTHER_ART_APP = 29;
    static const int OTHER_ART_BOOT = 30;
    static const int OTHER_DVK_STAT_ART_START = OTHER_ART_APP - NUM_OTHER_STATS;
    static const int OTHER_DVK_STAT_ART_END = OTHER_ART_BOOT - NUM_OTHER_STATS;

    static const int NUM_DVK_STATS = 14;

    static const int NUM_CATEGORIES = 8;

    static const int offsetPss = 0;
    static const int offsetSwappablePss = 1;
    static const int offsetPrivateDirty = 2;
    static const int offsetSharedDirty = 3;
    static const int offsetPrivateClean = 4;
    static const int offsetSharedClean = 5;
    static const int offsetSwappedOut = 6;
    static const int offsetSwappedOutPss = 7;

    /** The proportional set size for dalvik heap.  (Doesn't include other Dalvik overhead.) */
    int dalvikPss;
    /** The proportional set size that is swappable for dalvik heap. */
    /**  We may want to expose this, eventually. */
    int dalvikSwappablePss;
    /** The private dirty pages used by dalvik heap. */
    int dalvikPrivateDirty;
    /** The shared dirty pages used by dalvik heap. */
    int dalvikSharedDirty;
    /** The private clean pages used by dalvik heap. */
    /**  We may want to expose this, eventually. */
    int dalvikPrivateClean;
    /** The shared clean pages used by dalvik heap. */
    /**  We may want to expose this, eventually. */
    int dalvikSharedClean;
    /** The dirty dalvik pages that have been swapped out. */
    /**  We may want to expose this, eventually. */
    int dalvikSwappedOut;
    /** The dirty dalvik pages that have been swapped out, proportional. */
    /**  We may want to expose this, eventually. */
    int dalvikSwappedOutPss;

    /** The proportional set size for the native heap. */
    int nativePss;
    /** The proportional set size that is swappable for the native heap. */
    /**  We may want to expose this, eventually. */
    int nativeSwappablePss;
    /** The private dirty pages used by the native heap. */
    int nativePrivateDirty;
    /** The shared dirty pages used by the native heap. */
    int nativeSharedDirty;
    /** The private clean pages used by the native heap. */
    /**  We may want to expose this, eventually. */
    int nativePrivateClean;
    /** The shared clean pages used by the native heap. */
    /**  We may want to expose this, eventually. */
    int nativeSharedClean;
    /** The dirty native pages that have been swapped out. */
    /**  We may want to expose this, eventually. */
    int nativeSwappedOut;
    /** The dirty native pages that have been swapped out, proportional. */
    /**  We may want to expose this, eventually. */
    int nativeSwappedOutPss;

    /** The proportional set size for everything else. */
    int otherPss;
    /** The proportional set size that is swappable for everything else. */
    /**  We may want to expose this, eventually. */
    int otherSwappablePss;
    /** The private dirty pages used by everything else. */
    int otherPrivateDirty;
    /** The shared dirty pages used by everything else. */
    int otherSharedDirty;
    /** The private clean pages used by everything else. */
    /**  We may want to expose this, eventually. */
    int otherPrivateClean;
    /** The shared clean pages used by everything else. */
    /**  We may want to expose this, eventually. */
    int otherSharedClean;
    /** The dirty pages used by anyting else that have been swapped out. */
    /**  We may want to expose this, eventually. */
    int otherSwappedOut;
    /** The dirty pages used by anyting else that have been swapped out, proportional. */
    /**  We may want to expose this, eventually. */
    int otherSwappedOutPss;

    /** Whether the kernel reports proportional swap usage */
    /**  */
    bool hasSwappedOutPss;

    int otherStats[(NUM_OTHER_STATS+NUM_DVK_STATS)*NUM_CATEGORIES];

};

#endif
