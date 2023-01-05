#ifndef __GAME_LIST_MANAGER_H__
#define __GAME_LIST_MANAGER_H__


#include <vector>
#include <utils/Singleton.h>
#include <utils/Mutex.h>
using namespace std;


class GameListManager : public android::Singleton<GameListManager> {
public:
    GameListManager();
    ~GameListManager();
    void setGameMode(int mode);
    int getGameMode();
    void setCurrentGamePackage(std::string name);
    std::string getCurrentGamePackage();
    void setInstalledGameList(vector<std::string> list);
    void getInstalledGameList(vector<std::string>& list);
private:
    int gameMode;
    std::string currentGamePackage;
    vector<std::string> installedGameList;
    android::Mutex mMutex;
};

#endif

