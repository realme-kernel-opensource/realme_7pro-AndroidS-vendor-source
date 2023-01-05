#define ATRACE_TAG  ATRACE_TAG_GRAPHICS
#include <utils/Trace.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <utils/RefBase.h>
#include <sys/epoll.h>
#include <sys/types.h>
#include <private/android_filesystem_config.h>
#include <json/value.h>

#include "OIface.h"
#include "OIfaceServer.h"
#include "JsonParser.h"
#include "EngineParser.h"
#include "Utils.h"
#include "OimParser.h"
#include "LogSink.h"
#include "ThreadState.h"
#include "OIfaceClient.h"
#include "GlobalConfig.h"
#include "BinderManager.h"
#include "TaskMonitor.h"
#include "SurfaceFlingerProxyManager.h"
#include "OifaceCallbackManager.h"
#include "TaskManager.h"
#include "LightningStartManager.h"
#include "FrameStateManager.h"

#define LOG_FILE    "/sdcard/reslog.txt"

using namespace android;

SocketParser::SocketParser(const char *socketName, int fd, int type,
        int pid /* = -1*/, int uid /* = -1*/) :
        OIfaceClient(socketName, type, uid, pid, fd)
{
    DEBUG("SocketParser %s(%d %d) constructor", socketName, fd, type);
}

SocketParser::~SocketParser() {
    DEBUG("SocketParser %s(%d %d) destructor", getClientName().c_str(), getFd(), getType());
}

int SocketParser::acceptNewClient() {
    int newFd;
    int flags;
    int ret;
    int fd = getFd();

    newFd = accept(fd, NULL, NULL);
    if (newFd < 0) {
        if ((errno == EWOULDBLOCK) || (errno == EAGAIN))
            return -EAGAIN;
        ERROR("accept failed.(%s)\n", strerror(errno));
        return -1;
    }

    /* set to non-blocking */
    flags = fcntl(newFd, F_GETFL, 0);
    if (flags < 0) {
        ERROR("F_GETFL failed(%s)\n", strerror(errno));
        close(newFd);
        return -1;
    }

    ret = fcntl(newFd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0) {
        ERROR("F_SETFL failed(%s)\n", strerror(errno));
        close(newFd);
        return -1;
    }

    return newFd;
}

/* should avoid to use this */
int SocketParser::getPackageName(vector<string> &packages)
{
    int uid = getUid();

    if (uid < 0) {
        ERROR("get uid failed\n");
        return -1;
    }

    getPackageForUid(uid, packages);

    if (packages.size() <= 0) {
        ERROR("incoming connection does not have package name\n");
        return -1;
    }

    return 0;
}

/* return 1 if continue to receive */
int SocketParser::handlePollIn()
{
    int newFd = -1;
    int type = getType();
    int uid;
    uint32_t config;

    ATRACE_CALL();

    /* continue to receive event */
    DEBUG("socket %s(%d %d) received event", getClientName().c_str(), getFd(), getType());
    if (!OIfaceServer::isServerSocket(type)) {
        ERROR("unable to handle client event %s(%d %d)",
                getClientName().c_str(), getFd(), getType());
        return -1;
    }

    /* handle server socket event */
    newFd = acceptNewClient();
    if (newFd < 0) {
        ERROR("accept new client for %s failded", getClientName().c_str());
        return -1;
    }

    uid = getUid(newFd);
    if ((uid < AID_APP) && (type != OIfaceClient::OIFACE_TYPE_LOG_SERVER)) {
        ERROR("invalid uid %d connected", uid);
        close(newFd);
        return -1;
    }

    if (ThreadState::getInstance().getConfig(uid, &config) == 0) {
        /* module is disabled according to cached data */
        if ((config & MODULE_ENABLE_MASK) == 0) {
            ERROR("module for uid %d is disabled according to cached config", uid);
            close(newFd);
            return -1;
        }
    }

    OIfaceServer &sm(OIfaceServer::getInstance());

    string socketName = "uid_" + to_string(uid) + "_pid_" + to_string(getPid(newFd)) +
        "_fd_" + to_string(newFd);

    vector<sp<OIfaceClient>> clients;
    int count = 0;

    sm.getClientList(&clients);
    for (auto &iter: clients) {
        if (iter->getUid() == uid) {
            count++;
        }
    }

    if (count >= kMaxSocketClients) {
        ERROR("client %s exceed maximum socket clients:%d", socketName.c_str(), kMaxSocketClients);
        close(newFd);
        return -1;
    }

    /* tricky: oiface socket type server & client is stored in interleave way */
    if (sm.addClient(socketName, newFd, type + 1) < 0) {
        ERROR("unable to add client for %d", uid);
        close(newFd);
        return -1;
    }

    return 0;
}

/* @override, return 1 to continue receive event */
int SocketParser::handleEvent(int fd, int events, void*)
{
    DEBUG("SocketParser socket %s(%d %d) received event 0x%x from %d",
            getClientName().c_str(), getFd(), getType(), events, fd);
    int ret = 0;
    ATRACE_INT("SocketParser_handleEvent fd", fd);

    if (fd != getFd()) {
        ERROR("%s(%d %d) received unintentional event 0x%x from %d",
                getClientName().c_str(), getFd(), getType(), events, fd);
        return 1;
    }

    if (events & Looper::EVENT_INPUT) {
        ret = handlePollIn();
    }

    if ((events & Looper::EVENT_HANGUP) || (events & Looper::EVENT_ERROR) ||
            (events & Looper::EVENT_INVALID) || (ret == -2)) {
        ERROR("socket %s(%d %d) received event:0x%x", getClientName().c_str(),
                getFd(), getType(), events);

        OIfaceServer::getInstance().removeClient(fd);
        close(fd);

        return 0;
    }

    return 1;
}

int SocketParser::sendRemoteMessage(const string &str)
{
    int rc;

    mOutBuffer += str;

    DEBUG("buffer size:%d\n", (int)mOutBuffer.size());

    do {
        rc = send(getFd(), mOutBuffer.data(), mOutBuffer.size(), 0);
        if (rc < 0) {
            /* This should not happen. Just output error message for further debug.. */
            if ((errno != EWOULDBLOCK) && (errno != EAGAIN)) {
                ERROR("recv() failed(%s)\n",
                        strerror(errno));
                return -1;
            }

            ERROR("buffer is full(%s)..", strerror(errno));
            mOutBuffer = "";
            break;
        }

        DEBUG("buffer sent:%d", rc);
        mOutBuffer.erase(0, rc);
    } while (mOutBuffer.size() > 0);

    return 0;
}

int SocketParser::readRemoteMessage(string &str)
{
#define BUFFER_LEN      512
    int rc;
    char buffer[BUFFER_LEN];

    /* expect to receive full control message */
    while (1) {
        rc = recv(getFd(), buffer, (size_t)(sizeof(buffer) - 1), 0);
        if (rc < 0) {
            if ((errno != EWOULDBLOCK) && (errno != EAGAIN)) {
                ERROR("recv() failed(%s)\n",
                        strerror(errno));
                return -1;
            }

            DEBUG("recv done\n");
            break;
        }

        if (rc == 0) {
            INFO("Connection closed(%s)\n",
                    strerror(errno));
            return 0;
        }

        str.append(buffer, rc);
    }

    return 0;
}

/*********************************************************************************/
/*                             OIfaceServer                                     */
/*********************************************************************************/
const OIfaceServer::ServerSocket OIfaceServer::sServerName[] =
{
    {"@resmon", OIfaceClient::OIFACE_TYPE_JSON_SERVER},
    {"@oiface", OIfaceClient::OIFACE_TYPE_OIM_SERVER},
    {"@reslog", OIfaceClient::OIFACE_TYPE_LOG_SERVER},
    {"@engine_boost", OIfaceClient::OIFACE_TYPE_ENGINE_SERVER},
};

ANDROID_SINGLETON_STATIC_INSTANCE(OIfaceServer);

/* Create epoll control file descriptor */
OIfaceServer::OIfaceServer():  mLogFile(-1), mLooper(new Looper(false)), mInitialized(false)
{
    int fd;
    uint32_t mask;

    DEBUG("OIfaceServer constructor");

    for (int i = 0; i < static_cast<int>(ARRAY_SIZE(sServerName)); i++) {
        fd = initLocalServerSocket(sServerName[i].name);
        if (fd < 0) {
            ERROR("init local server socket %s failed", sServerName[i].name);
            continue;
        }

        if (addClient(sServerName[i].name, fd, sServerName[i].type)) {
            ERROR("add client %s faield", sServerName[i].name);
            break;
        }
    }

    if (GlobalConfig::getInstance().getConfig(&mask, {OIFACE_FEATURE_PROPERTY_PREFIX, "enable"}) < 0) {
        ERROR("get config for oiface failed");
        return;
    }

    if (mask & (1 << OIFACE_ENABLE_LOG_TO_FILE)) {
        mLogFile = open(LOG_FILE, O_WRONLY | O_CREAT | O_APPEND,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
        if (mLogFile < 0) {
            ERROR("open %s failed(%s)", LOG_FILE, strerror(errno));
        }
    }

    mInitialized = true;
}

OIfaceServer::~OIfaceServer() {
    DEBUG("OIfaceServer destructor");
    if (mLogFile > 0) {
        close(mLogFile);
    }
}

int OIfaceServer::isServerSocket(const char *name)
{
    return ((strcmp(OIFACE_JSON_SERVER, name) == 0) ||
        (strcmp(OIFACE_OIM_SERVER, name) == 0) ||
        (strcmp(OIFACE_LOG_SERVER, name) == 0) ||
        (strcmp(OIFACE_ENGINE_SERVER, name) == 0));
}

int OIfaceServer::isServerSocket(int type)
{
    return ((OIfaceClient::OIFACE_TYPE_JSON_SERVER == type) ||
            (OIfaceClient::OIFACE_TYPE_OIM_SERVER == type) ||
            (OIfaceClient::OIFACE_TYPE_LOG_SERVER == type) ||
            (OIfaceClient::OIFACE_TYPE_ENGINE_SERVER == type));
}

sp<SocketParser> OIfaceServer::asSocketParser(const char *name)
{
    /* return if exist */
    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getClientName() == name)
            return iter->second;
    }

    return NULL;
}

sp<SocketParser> OIfaceServer::asSocketParserWithLayerName(const string& layerName) {
    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getLayerName().find(layerName) != string::npos)
            return iter->second;
    }

    return NULL;
}

sp<SocketParser> OIfaceServer::asSocketParser(int fd)
{
    map<int, sp<SocketParser>>::iterator iter = mSock.find(fd);
    if (iter != mSock.end())
        return iter->second;

    return NULL;
}

sp<OIfaceClient> OIfaceServer::asOIfaceClient(const char *clientName) {
    /* FIXME: protect with lock? */
    if(strcmp(clientName, OIFACE_CONNECTION_LESS) == 0){
        return mBinderManager.getConnectionLessClient();
    }
    if (strncmp(clientName, "binder", strlen("binder")) == 0) {
        return mBinderManager.asOIfaceClient(clientName);
    }
    return asSocketParser(clientName);
}

sp<OIfaceClient> OIfaceServer::asOIfaceClientWithLayerName(const string& layerName) {
    sp<OIfaceClient> client = mBinderManager.asOIfaceClientWithLayerName(layerName);
    if (client != NULL)
        return client;

    client = asSocketParserWithLayerName(layerName);
    if (client != NULL)
        return client;

    return NULL;
}

bool OIfaceServer::hasAnyClientStartedFrameBoost() {
    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->isFrameBooted())
            return true;
    }

    return mBinderManager.hasAnyClientStartedFrameBoost();
}

string OIfaceServer::getClientName(const string &packageName) {
    android::Mutex::Autolock _l(mLock);

    if (packageName.size() == 0)
        return "";

    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getPackageName() == packageName)
            return iter->second->getClientName();
    }

    return mBinderManager.getClientName(packageName);
}

bool OIfaceServer::isClientRunning(const pid_t pid) {
    if (pid <= 0) {
        DEBUG("the client pid %d is invalid.", pid);
        return false;
    }

    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getPid() == pid)
            return true;
    }

    return mBinderManager.isClientRunning(pid);
}

void OIfaceServer::setHeavytask(pid_t pid, int tid, int normalizedTime) {
    if (pid <= 0 || tid <= 0 || normalizedTime <= 0) {
        DEBUG("the client pid %d, tid %d, normalizedTime %d is invalid.", pid, tid, normalizedTime);
        return;
    }

    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getPid() == pid) {
            DEBUG("socket client pid %d, set task tid %d, load %d.", pid, tid, normalizedTime);
            iter->second->setHeavyTaskLoad(tid, normalizedTime);
            return;
        }
    }

    return mBinderManager.setHeavytask(pid, tid, normalizedTime);
}

void OIfaceServer::getHeavyTask(pid_t pid, map<int, int>& outTaskLoad) {
    if (pid <= 0) {
        DEBUG("the client pid %d is invalid.", pid);
        return;
    }

    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getPid() == pid) {
            DEBUG("get socket client pid %d task load info.", pid);
            iter->second->getHeavyTaskLoad(outTaskLoad);
            return;
        }
    }

    return mBinderManager.getHeavyTask(pid, outTaskLoad);
}

void OIfaceServer::clearHeavyTask(pid_t pid) {
    if (pid <= 0) {
        DEBUG("the client pid %d is invalid.", pid);
        return;
    }

    map<int, int>& keyTaskInfo = TaskManager::getInstance().getKeyTaskInfo();
    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getPid() == pid) {
            DEBUG("clear socket client pid %d vip thread.", pid);
            if (keyTaskInfo.size() > 0) {
                for (auto itr = keyTaskInfo.begin(); itr != keyTaskInfo.end(); itr++) {
                    DecisionDriver::getInstance().setKeyTask(pid, itr->first, 0);
                }
                TaskManager::getInstance().clearAll();
                iter->second->clearHeavyTaskLoad();
            }
            return;
        }
    }

    return mBinderManager.clearHeavyTask(pid);
}

int OIfaceServer::initLocalServerSocket(const string &str) {
    int flags;
    int ret;
    struct sockaddr_un addr;
    int listenFd;

    listenFd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listenFd < 0) {
        ERROR("create socket failed(%s)\n", strerror(errno));
        return -1;
    }

    flags = fcntl(listenFd, F_GETFL, 0);
    ret = fcntl(listenFd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0) {
        ERROR("set socket to non-blocking failed(%s)\n",
                strerror(errno));
        close(listenFd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", str.c_str());
    addr.sun_path[0] = '\0';

    /*
     * Set addrlen to sizeof(sa_family_t) will set socket to abstract UNIX socket
     * Note sun_path is *not* supposed to be '\0' terminated. See man 7 unix.
     */
    ret = ::bind(listenFd, (struct sockaddr*)&addr, str.size() +
                    offsetof(struct sockaddr_un, sun_path));
    if (ret < 0) {
        ERROR("bind to %s failed(%s)\n", str.c_str(),
                strerror(errno));
        close(listenFd);
        return -1;
    }

    ret = listen(listenFd, MAX_CONN);
    if (ret < 0) {
        ERROR("listen failed.(%s)\n", strerror(errno));
        close(listenFd);
        return -1;
    }

    return listenFd;
}

int OIfaceServer::addClient(const string &name, int fd, int type)
{
    bool isAdded = 0;
    android::Mutex::Autolock _l(mLock);

    sp<SocketParser> parser;
    int uid = OIfaceClient::getUid(fd);
    int pid = OIfaceClient::getPid(fd);

    switch (type) {
        case OIfaceClient::OIFACE_TYPE_JSON_SERVER:
        case OIfaceClient::OIFACE_TYPE_OIM_SERVER:
        case OIfaceClient::OIFACE_TYPE_LOG_SERVER:
        case OIfaceClient::OIFACE_TYPE_ENGINE_SERVER:
            parser = new SocketParser(name.c_str(), fd, type, getpid(), getuid());
            break;

        case OIfaceClient::OIFACE_TYPE_JSON_CLIENT:
            parser = new JsonParser(name.c_str(), fd, type, pid, uid);
            break;
        case OIfaceClient::OIFACE_TYPE_ENGINE_CLIENT:
            parser = new EngineParser(name.c_str(), fd, type, pid, uid);
            break;
        case OIfaceClient::OIFACE_TYPE_OIM_CLIENT:
            parser = new OimParser(name.c_str(), fd, type, pid, uid);
            break;
        case OIfaceClient::OIFACE_TYPE_LOG_CLIENT:
            parser = new LogSink(name.c_str(), fd, type, pid, uid);
            break;
        default:
            ERROR("unknown client type:%d name:%s fd:%d",
                    type, name.c_str(), fd);
            return -1;
    }

    if (type != OIfaceClient::OIFACE_TYPE_OIM_CLIENT) { /* OIM client uses check permission func id*/
        if (!parser->isInitialized()) {
            ERROR("parser %s is not initialized", parser->getClientName().c_str());
            return -1;
        }
    }

    isAdded = mLooper->addFd(fd, Looper::POLL_CALLBACK,
            Looper::EVENT_INPUT, parser, NULL);


    if (!isAdded) {
        ERROR("add client %s(%d %d) failed", name.c_str(), fd, type);
        return -1;
    }

    mSock[fd] = parser;

    sendClientRemoteLog();

    return 0;
}

void OIfaceServer::sendClientRemoteLog() {
    uint32_t config = 0;
#define MAX_LOG_BUF 256
    char buf[MAX_LOG_BUF];
    string out;

    if (GlobalConfig::getInstance().getConfig(&config, {OIFACE_FEATURE_PROPERTY_PREFIX, "enable"}) < 0) {
        ERROR("get config for oiface failed");
        return;
    }

    if ((config & ( 1 << OIFACE_ENABLE_REMOTE_LOG)) == 0)
        return;

    if (mSock.size() <= 0)
        return;

    /* Send log as following json format:
      {
      "client": [
      {
      "name": "@resmon",
      "type": 0,
      "uid": 1000,
      "fd": 9,
      "package": "fake.package"
      },
      {
      "name": "@oiface",
      "type": 2,
      "uid": 1000,
      "fd": 10,
      "package": "fake.package"
      },
      {
      "name": "@reslog",
      "type": 4,
      "uid": 1000,
      "fd": 11,
      "package": "fake.package"
      },
      {
      "name": "uid_0_fd_25",
      "type": 5,
      "uid": 0,
      "fd": 25,
      "package": "fake.package"
      }
      ]
      }
     */

    out = "{\"client\":[";

    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin();
            iter != mSock.end(); iter++) {
        snprintf(buf, sizeof(buf), "{\"name\":\"%s\", \"type\":%d, \"uid\":%d, \"fd\":%d, \"package\":\"%s\"},",
                iter->second->getClientName().c_str(), iter->second->getType(),
                iter->second->getUid(), iter->second->getFd(),
                iter->second->getPackageName().c_str());
        out += buf;
    }

    if (out.back() == ',')
        out.erase(out.size() - 1, 1);

    out += "]}\n";

    sendRemoteLog(out.c_str());
}

int OIfaceServer::removeClient(int fd)
{
    android::Mutex::Autolock _l(mLock);
    /* handle state restore before client disapear */
    auto iter = mSock.find(fd);
    if (iter == mSock.end()) {
        ERROR("unable to find client for fd:%d", fd);
        return -1;
    }
    DecisionDriver::getInstance().updateDecisionState(iter->second->getClientName(), false);
    mSock.erase(fd);

    sendClientRemoteLog();

    return 0;
}

int OIfaceServer::sendRemoteLog(const string &str) {
    uint32_t mask;

    if (GlobalConfig::getInstance().getConfig(&mask, {OIFACE_FEATURE_PROPERTY_PREFIX, "enable"}) < 0) {
        ERROR("get config for oiface failed");
        return -1;
    }

    for (map<int, sp<SocketParser>>::iterator iter = mSock.begin(); iter != mSock.end(); iter++) {
        if (iter->second->getType() == OIfaceClient::OIFACE_TYPE_LOG_CLIENT) {
            sp<LogSink> log = static_cast<LogSink*>(iter->second.get());
            if ((log->getUid() == AID_SYSTEM) || (mask & 1 << OIFACE_ENABLE_REMOTE_LOG)) {
                if (log->LogMessage(str) < 0) {
                    ERROR("sendRemoteLog failed for (%s,%d,%d)",
                            log->getClientName().c_str(),
                            log->getType(),
                            log->getFd());
                }
            }
        }
    }

    if (mLogFile > 0) {
        struct timeval tv;
        char time[128];
        string out;

        gettimeofday(&tv, NULL);
        snprintf(time, sizeof(time), "%ld.%ld:", tv.tv_sec, tv.tv_usec);

        out = time + str;
        if (::write(mLogFile, out.c_str(), out.size()) < 0) {
            ERROR("write log failed(%s)", strerror(errno));
        }
    }

    return 0;
}

int OIfaceServer::joinSocketLoop()
{
    if (mSock.size() < ARRAY_SIZE(sServerName)) {
        ERROR("unexpected socket server size:%d", (int)mSock.size());
        return -1;
    }

    if (!mInitialized) {
        ERROR("OifaceServer is not initialized");
        return 0;
    }

    ERROR("OifaceServer polling!!");
    while (true) {
        mLooper->pollAll(-1);
        DEBUG("OifaceServer loop once");
    }

    ERROR("OifaceServer exited!!");
    return 0;
}

/* called by oiface service thread */
int OIfaceServer::sendMessage(const BinderMessage& msg) {
    sp<BinderHandler> handler = new BinderHandler(msg);
    mLooper->sendMessage(handler, Message(msg.what));
    if(msg.what == BINDER_MESSAGE_BATTERY_LEVEL) {
        currentBatteryLevel = msg.value;
    }
    return 0;
}

int OIfaceServer::sendMessageDelayed(const BinderMessage& msg, nsecs_t delay) {
    sp<BinderHandler> handler = new BinderHandler(msg);
    mLooper->sendMessageDelayed(delay, handler, Message(msg.what));
    return 0;
}

int OIfaceServer::removeMessage(const BinderMessage& msg) {
    sp<BinderHandler> handler = new BinderHandler(msg);
    mLooper->removeMessages(handler, msg.what);
    return 0;
}

int OIfaceServer::handleMessage(const BinderMessage& msg) {
    DEBUG("oiface server handling message:%d json:%s", msg.what, msg.json.c_str());
    switch (msg.what) {
        case BINDER_MESSAGE_RESTORE_STATE: {
            DecisionDriver::getInstance().updateDecisionState(msg.json, true);
            sp<OIfaceClient> client = asOIfaceClient(msg.json.c_str());
            if (client != NULL) {
                client->restoreFrameStats();
            }
        } break;
        case BINDER_MESSAGE_RESTORE_CONNECTIONLESS_STATE: {
            mBinderManager.updateConnectionLessClient(msg.json, true);
        } break;
        case BINDER_MESSAGE_CANCEL_STATE: {
            DecisionDriver::getInstance().updateDecisionState(msg.json, false);
            sp<OIfaceClient> client = asOIfaceClient(msg.json.c_str());
            if (client != NULL) {
                client->getFrameStats();
                client->freshBackgroundStats();
            }
        } break;
        case BINDER_MESSAGE_UPDATE_FRAME_STATS: {
            string layerName = msg.json;
            sp<OIfaceClient> client
                    = OIfaceServer::getInstance().asOIfaceClientWithLayerName(layerName);
            if (client != NULL) {
                client->updateFrameStats(msg.data.frameStats);
            }
        } break;
        case BINDER_MESSAGE_FRAME_BOOST: {
            DEBUG("OIfaceServer receive frameboost message");
            FrameRescuer::getInstance().triggerFrameBoost(msg.json, msg.data.frameboost);
        } break;
        case BINDER_MESSAGE_SCHEDTUNE_UPDATE_BOOST_TASKS: {
            sp<OIfaceClient> client = asOIfaceClient(msg.json.c_str());
            if (client != NULL) {
                client->updateFrameBoostTaskList();
            }
        } break;
        case BINDER_MESSAGE_NOTIFY_NORMALIZED_EXEC_TIME:{
            int normalizedTime = msg.value;
            vector<string> taskInfo;
            splitString(msg.json, taskInfo, ":");
            int tid = stoi(taskInfo[1]);
            TaskManager::getInstance().setHeavyTask(tid, normalizedTime);
        } break;
        case BINDER_MESSAGE_TASK_MONITOR_TIMEOUT: {
            TaskMonitor::getInstance().stop();
            vector<string> pickNum;
            splitString(msg.json, pickNum, ":");
            TaskManager::getInstance().pickHeavyTask(msg.pid, stoi(pickNum[1]));
            map<int, int>& keyTaskInfo = TaskManager::getInstance().getKeyTaskInfo();
            if (keyTaskInfo.size() > 0) {
                for (auto iter = keyTaskInfo.begin(); iter != keyTaskInfo.end(); iter++) {
                    TaskManager::getInstance().setMainTask(iter->first);
                    TaskManager::getInstance().setKeyThreadReportInfo(iter->first, TaskManager::getInstance().getKeyThreadBindValue());
                    DecisionDriver::getInstance().setKeyTask(msg.pid, iter->first, TaskManager::getInstance().getKeyThreadBindValue());
                }
                TaskManager::getInstance().setMonitorTask(TaskManager::getInstance().getMainTask());
            }
        } break;
        case BINDER_MESSAGE_PICK_TASK_TIMEOUT: {
            TaskManager::getInstance().findKeyWorker();
        } break;
        case BINDER_MESSAGE_PICK_KEY_TASK_FINISHED: {
            map<int, int>& keyTaskInfo = TaskManager::getInstance().getKeyTaskInfo();
            if (keyTaskInfo.size() > 0) {
                for (auto iter = keyTaskInfo.begin(); iter != keyTaskInfo.end(); iter++) {
                    DecisionDriver::getInstance().setKeyTask(msg.pid, iter->first, iter->second);
                }
            }
        } break;
        case BINDER_MESSAGE_SET_LIGHTNING_START_PACKAGE: {
            LightningStartManager::getInstance().setLightningStartPackage(msg.json.c_str());
        } break;

        case BINDER_MESSAGE_STOP_LIGHTNING_START_PACKAGE: {
            FrameStateManager::getInstance().stopSurfaceFlingerFrameNotification(msg.json);
        } break;
        case BINDER_MESSAGE_COLLECT_EVENT_DATA: {
            OifaceCallbackManager::getInstance().reportEvent2DataCollecter(msg.value, msg.json);
        } break;
        default:
            mBinderManager.handleMessage(msg);
            break;
    }
    return 0;
}

int OIfaceServer::getClientList(std::vector<sp<OIfaceClient>> *list) {
    android::Mutex::Autolock _l(mLock);
    if (list == NULL) {
        DEBUG("invalid argument passed to get client list");
        return -1;
    }
    for (auto &iter: mSock) {
        list->push_back(iter.second);
    }

    mBinderManager.getClientList(list);

    return 0;
}

int OIfaceServer::getCurrentTemp() {
    return currentBatteryLevel;
}