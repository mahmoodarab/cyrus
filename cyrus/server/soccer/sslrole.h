#ifndef SSLROLE_H
#define SSLROLE_H

#include "../ai/SSLWorldModel.h"
#include "../ai/SSLGame.h"
#include "../ai/SSLAnalyzer.h"

#include "../definition/SSLBall.h"
#include "sslskill.h"
#include "sslgamepositions.h"

#define Ball_Position SSLWorldModel::getInstance()->mainBall()->Position()
#define Ball_Speed    SSLWorldModel::getInstance()->mainBall()->Speed()

class SSLAgent;

class SSLRole
{
public:
    enum Type{ e_Active, e_Blocker, e_Defender, e_GoalKeeper, e_OpponentMarker,
               e_PlayStarter, e_Position, e_WaitPass, e_WaitRebound, e_WallStander, e_SideCleaner };

    SSLRole();
    void setAgent(SSLAgent* agent);

    virtual void run() = 0;
    virtual Vector3D expectedPosition() = 0;
    bool Halt();

    Vector3D myPosition();

    SSLAgent* m_agent;
    Type m_type;
    short m_hardness;

protected:
    enum ActiveRoleState { e_FarFromBall, e_NearBall, e_CanKick} m_state;


private:

};

#endif // SSLROLE_H
