#include <utils/RefBase.h>
#include <stdio.h>
#include <dlfcn.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <cutils/log.h>
#include <cutils/properties.h>
#include "OIface.h"

#include <openssl/aes.h>

static const char* gOifacePath[] = {
        OIFACE_DATA_PATH,
        OIFACE_DATA_OPLUS_LIB_PATH,
};

extern "C" int oiface_main(void* resp);

/* return a prefered oiface entry point */
static int (*getPreferedCallback(void))(void*) {
    int index = -1;
    int (*entry)(void*) = oiface_main;
    void *sHandle = NULL;

    /* FIXME: may leave package dlopened without dlclose */
    for (int i = 0; i < (int)ARRAY_SIZE(gOifacePath); i++) {
        struct OifaceVersion *gVersion;
        void *handle = NULL;
        int (*callback)(void*) = NULL;
        struct stat st;

        if (stat(gOifacePath[i], &st) < 0)  {
            ERROR("stat %s failed(%s), continue...", gOifacePath[i],
                    strerror(errno));
            continue;
        }

        handle = dlopen(gOifacePath[i], RTLD_NOW);
        if (handle == NULL) {
            ERROR("dlopen %s failed(%s)", gOifacePath[i], dlerror());
            continue;
        }

        gVersion = (struct OifaceVersion*)dlsym(handle, "gVersion");
        if (gVersion == NULL) {
            ERROR("get version from %s returned NULL, continue...",
                    gOifacePath[i]);
            dlclose(handle);
            continue;
        }

        if (gVersion->sdk != ANDROID_VERSION) {
            ERROR("library version(platform:%d) not compatible, continue...",
                    gVersion->sdk);
            dlclose(handle);
            continue;
        }

        callback = (int (*)(void*))dlsym(handle, "oiface_main");
        if (callback == NULL) {
            ERROR("unable to locate oiface_main, continue...");
            dlclose(handle);
            continue;
        }

        index = i;
        entry = callback;
        if (sHandle != NULL) { /* last selected handle, close it if selected new one */
            DEBUG("close last selected handle ");
            dlclose(sHandle);
            sHandle = handle;
        }
        dlclose(handle);
    }

    if (index >= 0) {
        if (setenv(OIFACE_LOCATION_KEY, gOifacePath[index], 1)) {
            ERROR("setenv failed.(%s)", strerror(errno));
        }
    }

    return entry;
}

int main()
{
    int (*entry)(void*);
    char buf[PROPERTY_VALUE_MAX];

    DEBUG("oiface version:(platform:%d)", ANDROID_VERSION);

    if (setenv(OIFACE_LOCATION_KEY, "system", 1)) {
        ERROR("setenv failed.(%s)", strerror(errno));
    }

    property_get(OIFACE_PROPERTY, buf, OIFACE_PROPERTY_DEFAULT_VALUE);
    if ((strcmp(buf, "1") != 0) && (strcmp(buf, "2") != 0)) {
        ERROR("oiface is not enabled\n");
        return -1;
    }

    oiface_main(NULL);

    return -1;
}
