#include "GameListManager.h"

ANDROID_SINGLETON_STATIC_INSTANCE(GameListManager);

GameListManager::GameListManager() {
    gameMode = 0;
    currentGamePackage = "";
}

GameListManager::~GameListManager() {
}

void GameListManager::setGameMode(int mode){
    gameMode = mode;
}

 int GameListManager::getGameMode(){
     return gameMode;
 }

void GameListManager::setCurrentGamePackage(std::string name){
    currentGamePackage = name;
}

std::string GameListManager::getCurrentGamePackage(){
    return currentGamePackage;
}

void GameListManager::setInstalledGameList(vector<std::string> list){
    android::Mutex::Autolock lock(mMutex);
    installedGameList.assign(list.begin(), list.end());
}

void GameListManager::getInstalledGameList(vector<std::string>& list){
    android::Mutex::Autolock lock(mMutex);
    list.assign(installedGameList.begin(), installedGameList.end());
}