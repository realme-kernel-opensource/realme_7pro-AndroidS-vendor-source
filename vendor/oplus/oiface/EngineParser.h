#ifndef __ENGINE_PARSER_H__
#define __ENGINE_PARSER_H__

#include <string>
#include "OIfaceServer.h"
#include "OIface.h"
#include "DecisionDriver.h"
#include "OIfaceModuleLoader.h"
#include "PlatformAdaptor.h"

using namespace std;

class EngineParser: public SocketParser {
    public:
        EngineParser(const char *socketName, int fd, int type, int pid, int uid);
        virtual ~EngineParser();
        virtual int handlePollIn();
    protected:
        int formatJson();
        virtual void onTriggerStop();

    private:
        std::vector<nodeType*> mSyntaxTree;
        void matchAndApply(const Json::Value& value);
        bool isInterested(const Json::Value& value);
        Json::Value mConfig;
        std::map<std::string, int> mClientVariable;
        int64_t mStartTime;
        string mRawBuffer;
        string mJsonBuffer;
};

#endif
