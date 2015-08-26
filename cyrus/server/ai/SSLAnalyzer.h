#ifndef _ANALYZER_H
#define _ANALYZER_H

#include "../../shared/general.h"
#include "../../shared/sslnamespace.h"
#include "../../shared/tools/ssllistener.h"
#include <vector>
#include <map>
#include <algorithm>

class SSLGame;
class SSLWorldModel;
class SSLTeam;
class SSLRobot;
class SSLBall;

using namespace SSL;
using namespace std;

#define analyzer SSLAnalyzer::getInstance()

// Singleton Class
class SSLAnalyzer : SSLListener
{
public:
    struct RobotIntersectTime {
        float m_time;
        Vector2D m_position;
        SSLRobot* m_robot;

        bool isValid() {
            return !(m_robot == NULL);
        }

        bool isInfinite()  {
            return ( fabs(m_position.X()) > FIELD_LENGTH_2 * 1.5
                  || fabs(m_position.Y()) > FIELD_WIDTH_2  * 1.5);
        }

        RobotIntersectTime(float time = 0.0, Vector2D position = Vector2D(0.0, 0.0), SSLRobot* robot = NULL) {
            m_time = time;
            m_position = position;
            m_robot = robot;
        }

        bool operator < (const RobotIntersectTime & other) const {
            return this->m_time < other.m_time;
        }
    };
private:
    SSLAnalyzer();  // default constructor
    static SSLAnalyzer *analyzer_instance;

    void updateDistances();
    RobotIntersectTime  whenWhereCanRobotCatchTheBall_imp1(SSLRobot* robot);
    pair<float, float> blockedAngleByARobot(const Vector2D & vec, const SSLRobot* robot);
    float positiveSideAngleInNegativeSide(float angle) const;
    float negativeSideAngleInPositiveSide(float angle) const;
    float wastedTimeForInertia(SSLRobot *robot, Vector2D target) const;

public:

    static SSLAnalyzer *getInstance();
    void check();

//    //LEVEL 0 : BASICS -------------------------------------------------------------------------

    double distanceFromBall(const Vector2D & point);
    double distanceFromBall(const SSLRobot * robot);
    double distanceFromRobotToPoint(const Vector2D & point,const SSLRobot * robot);
    double distanceFromRobotToRobot(const SSLRobot * robot1,const SSLRobot * robot2);

//    //LEVEL 1  --------------------------------------------------------------------------------

    SSLRobot* whichRobotCanKick(); // Null : none
    bool canKick(SSLRobot* robot);

    RobotIntersectTime nearestRobotIntersectBall(uint index = 0);
    RobotIntersectTime nearestRobotIntersectBall(SSL::Color teamColor, uint index = 0);
    RobotIntersectTime nearestRobotIntersectBall(const vector<SSLRobot *> & robots, uint index = 0);

    SSLRobot* nearestToBall(const vector<SSLRobot*> robots, uint index = 0);

    RobotIntersectTime nearestRobotToPoint(const Vector2D & point, uint index = 0);
    RobotIntersectTime nearestRobotToPoint(SSL::Color teamColor, const Vector2D & point, uint index = 0);
    RobotIntersectTime nearestRobotToPoint(const vector<SSLRobot*> & robots, const Vector2D & point, uint index = 0);

    SSLRobot* nearestToPoint(const vector<SSLRobot*> & robots, const Vector2D & point, uint index = 0);

    RobotIntersectTime nearestRobotToRobot(const SSLRobot* robot, uint index = 0);
    RobotIntersectTime nearestRobotToRobot(SSL::Color teamColor, const SSLRobot * robot, uint index = 0);
    RobotIntersectTime nearestRobotToRobot(const vector<SSLRobot*> & robots,const SSLRobot * robot, uint index = 0);

    Vector2D ballIntersectionWithOurGoalLine();

//   //LEVEL 2 : ROBOT & ROBOT ------------------------------------------------------------------
//    RobotIntersectTime nearestRobotsToBlockPointOutOfOurPenaltyArea(const Vector2D targetPoint, uint index =0);
//    RobotIntersectTime nearestRobotsToBlockPoint(const Vector2D targetPoint, uint index = 0);
//    RobotIntersectTime nearestRobotsToMarkRobot(const SSLRobot * robot, uint index = 0); // nazdiktarin robot be robot

// LEVEL 3 : PREDICTION ------------------------------------------------------------
    SSLAnalyzer::RobotIntersectTime  whenWhereCanRobotCatchTheBall(SSLRobot* robot);
    std::vector<SSLRobot *> blockersFromPoint(const Vector2D targetPoint);
    std::vector<pair<float, float> > openAngleToGoal(const Vector2D targetPoint);

// ball Possession methods
    float ballPossession();
    SSLTeam * ballPossessorTeam();

// Referee State Analyze:
    bool isOurKickOffPosition();
    bool isOpponentKickOffPosition();
    bool isOurKickOffKick();
    bool isOpponentKickOffKick();

    bool isOurPenaltyPosition();
    bool isOpponentPenaltyPosition();
    bool isOurPenaltyKick();
    bool isOpponentPenaltyKick();

    bool isOurDirectKick();
    bool isOpponentDirectKick();
    bool isOurIndirectKick();
    bool isOpponentIndirectKick();

    bool isGameRunning();
    bool isPointInOurSide(const Vector2D& point);
    bool isRobotWithinOurPenaltyArea(Vector3D robot_pos);
    bool isPointWithinOurPenaltyArea(Vector2D point);
    bool isPointWithinOurCorner(const Vector2D &point);
    bool isPointWithinOurUpCorner(const Vector2D &point);
    bool isPointWithinOurDownCorner(const Vector2D &point);

private:
    bool m_game_running;

    // f stands for flag
    bool f_distance_robot_ball       [MAX_ID_NUM << 1]                 ;
    bool f_distance_robot_robot      [MAX_ID_NUM << 1][MAX_ID_NUM << 1];
    bool f_canKick                   [MAX_ID_NUM << 1]                 ;
    bool f_whenWhereCanCatchTheBall  [MAX_ID_NUM << 1]                 ;
    bool f_whichRobotCanKick                                           ;


    // c stands for cache
    double             c_distance_robot_ball       [MAX_ID_NUM << 1]                 ;
    double             c_distance_robot_robot      [MAX_ID_NUM << 1][MAX_ID_NUM << 1];
    bool               c_canKick                   [MAX_ID_NUM << 1]                 ;
    RobotIntersectTime c_whenWhereCanCatchTheBall  [MAX_ID_NUM << 1]                 ;
    SSLRobot*          c_whichRobotCanKick                                           ;

    void initialize_caches();
    uint _getHashID(const SSLRobot * robot);

    int ball_move_counter;
    Vector2D ball_stop_position;


};

#endif // _ANALYZER_H
