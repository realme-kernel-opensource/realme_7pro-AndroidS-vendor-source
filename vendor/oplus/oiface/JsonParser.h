#ifndef __JSON_PARSER_H__
#define __JSON_PARSER_H__

#include "OIfaceServer.h"
#include "OIface.h"
#include "DecisionDriver.h"
#include "OIfaceModuleLoader.h"
#include "PlatformAdaptor.h"

#define NETWORK_LOG_ACTION_SHIFT        100
#define SCENE_LOG_ACTION_SHIFT          110

class JsonParser: public SocketParser {
    public:
        JsonParser(const char *socketName, int fd, int type, int pid, int uid);
        virtual ~JsonParser();

        void dumpKeys();

        int parse();
        int handlePollIn();

        virtual bool isInitialized() {
            return ((getPlatform() != PLAT_UNKNOWN) && (mLoader.isInitialized()));
        }

        void clearDirty() {
            dirty = 0;
        };
        bool isDirty() {
            return dirty;
        };

        int getPlatform() {
            PlatformAdaptor& adaptor(PlatformAdaptor::getInstance());
            return adaptor.getPlatform();
        }

    protected:
        void resetOneshotValue();
        int formatJson();
        const map<string, JsonValue>& getJsonValue();
        int getJsonValue(const string& str, JsonValue* val);
        void setJsonValue(const string& str, JsonValue& val);
        /* Copy JsonKey list from strategy */
        int setJsonKey(const struct json_key* keyList);
        const vector<JsonKey>& getJsonKey();
        virtual int performAction();

        virtual void onTriggerStop();

    private:
        DISALLOW_COPY_AND_ASSIGN(JsonParser);

        /* internal buffer */
        string mRawBuffer;
        string mJsonBuffer;

        vector<JsonKey> mJsonKey;
        map<string, JsonValue> mJsonValue;
        bool dirty;

        DecisionDriver& driver;

        OIfaceModuleLoader mLoader;

        Decision mDecision[MAX_DECISIONS];
        int64_t mStartTime;

        bool compareJsonValue(int type, JsonValue& v1, JsonValue& v2);
        /* convert str to desired type and assign it to value */
        int assignValue(int type, JsonValue& value, const char* str);
};

#endif
