#ifndef __BINDER_CLIENT_H__
#define __BINDER_CLIENT_H__

#include <utils/Looper.h>
#include <map>
#include <string>

#include <utils/String8.h>
#include "OIfaceService.h"
#include "BinderMessage.h"
#include "OIfaceClient.h"
#include "comp.h"

class BinderClient: public OIfaceClient, public virtual android::RefBase {
    public:
        BinderClient(int uid, int pid);
        virtual ~BinderClient();

        virtual int handleEvent(const BinderMessage &msg);
        int registerNotifier(const android::sp<android::IBinder>& binder);
        void enableNotifier(bool enable) {mNotifierEnabled = enable;};
        void notifyCallback(const std::string &msg);
        bool isNotifierEnabled() {return mNotifierEnabled;};
    private:
#ifdef OPLUS_FEATURE_OPPO_OIFACE
        android::sp<IOIfaceNotifier> mNotifier;
#endif
        bool mNotifierEnabled;

        /* apply decision if match a rule in configuration */
        void matchAndApply(const Json::Value& value);
        bool isInterested(const Json::Value& value);
        Json::Value mConfig;

        /* OSL syntax tree */
        std::vector<nodeType*> mSyntaxTree;

        /* global variables defined in oiface configration language */
        std::map<std::string, int> mClientVariable;
        int64_t mStartTime;
    protected:
        virtual void onTriggerStop();
};

#endif
