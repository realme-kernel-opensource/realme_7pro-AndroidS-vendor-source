#ifndef __SOCKET_PARSER_H__
#define __SOCKET_PARSER_H__

#include <stdio.h>
#include <binder/Parcel.h>
#include <utils/Singleton.h>
#include <utils/Looper.h>

#ifndef USE_NEO
//#include <android-base/macros.h>
#endif

#include <map>

#include "OIfaceClient.h"
#include "OIfaceServer.h"
#include "BinderMessage.h"
#include "BinderManager.h"

#define MAX_CONN    32

/* server's socket info is constructed by SocketHandler */
class SocketParser: public OIfaceClient, public android::LooperCallback {
    public:
        using OIfaceClient::getPackageName;
        SocketParser(const char *socketName, int fd, int type, int pid = -1, int uid = -1);
        virtual ~SocketParser();

        virtual bool isInitialized() {return true;}; /* child override this value. */

        virtual int handleEvent(int fd, int events, void* data);
        virtual int handlePollIn();

    protected:

        /* str is just a container and may not null-terminated */
        virtual int readRemoteMessage(std::string &str);
        virtual int sendRemoteMessage(const std::string &str);

        static const int kMaxSocketClients = 10;

    private:
        DISALLOW_COPY_AND_ASSIGN(SocketParser);

        /* Server related functions */
        int acceptNewClient();
        int getPackageName(std::vector<std::string> &packages);

        std::string mOutBuffer;
};

class OIfaceServer: public android::Singleton<OIfaceServer> {
    public:
        static int isServerSocket(const char *name);
        static int isServerSocket(int type);

        /* Return corresponding socketParser */
        android::sp<SocketParser> asSocketParser(int fd);
        android::sp<SocketParser> asSocketParser(const char *name);
        android::sp<SocketParser> asSocketParserWithLayerName(const std::string& layerName);

        android::sp<OIfaceClient> asOIfaceClient(const char *name);
        android::sp<OIfaceClient> asOIfaceClientWithLayerName(const std::string& layerName);
        bool hasAnyClientStartedFrameBoost();

        /* get socket name by package name
         * FIXME: clone app have same package name as original one
         * this function may be called from different thread. Should
         * protect with lock.
         */
        std::string getClientName(const std::string &packageName);

        bool isClientRunning(const pid_t pid);
        void setHeavytask(const pid_t pid, const int tid, const int normalizedTime);
        void getHeavyTask(pid_t pid, map<int, int>& outTaskLoad);
        void clearHeavyTask(pid_t pid);
        void setAuthOff(const bool off) {
            authOff = off;
        }
        int getAuthOff() {
            return authOff;
        }

        int addClient(const std::string &name, int fd, int type);
        int removeClient(int fd);

        int joinSocketLoop();

        int sendRemoteLog(const std::string& str);

        /* called from binder service */
        int sendMessage(const BinderMessage& msg);
        int sendMessageDelayed(const BinderMessage& msg, nsecs_t delay);
        int removeMessage(const BinderMessage& msg);

        /* called in main looper */
        int handleMessage(const BinderMessage& msg);

        /* get client list */
        int getClientList(std::vector<android::sp<OIfaceClient>> *list);

        int getCurrentTemp();

    private:
        struct ServerSocket {
            const char *name;
            int type;
        };
        static const ServerSocket sServerName[];
        /* cached fd->SocketParser map */
        std::map<int, android::sp<SocketParser>> mSock;

        mutable android::Mutex mLock;
        int mLogFile;
        android::sp<android::Looper> mLooper;
        BinderManager mBinderManager;
        bool mInitialized;
        int currentBatteryLevel;
        friend class Singleton<OIfaceServer>;
        int authOff = 0;

        OIfaceServer();
        ~OIfaceServer();

        int initLocalServerSocket(const std::string &str);
        void sendClientRemoteLog();
};

#endif
