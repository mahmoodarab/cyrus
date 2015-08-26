#ifndef _BALLFILTER_H
#define _BALLFILTER_H

#include <vector>
#include <stdlib.h>
#include "sslframe.h"
#include "alphabetafilter.h"
#include "../../shared/utility/vector2d.h"
#include "../definition/SSLBall.h"


#define MAX_BALL_MEMORY 30
#define MAX_BALL_MEDIAN_MEMORY 12
#define BALL_SPEED_LIMIT_FILTER 7

using namespace std;

class BallFilter
{
    friend class MainWindow;
    friend class GUIHandler;
    friend class VisionFilter;
public:
    BallFilter();

    void initialize(const OneObjectFrame &initial_frame);

    void putNewFrame(const OneObjectFrame &detected_ball);
    void putNewFrameWithManyBalls(vector<OneObjectFrame> detected_balls);

    // main method for updating state vectors
    void run();
    void executeAlphaBetaFilter();
    void executeClusterFilter();

    bool isEmpty() const;

protected:
    SSLBallState& getRawData(uint i) {return rawData[i];}
    vector<SSLBallState> rawData;

    bool getBallStoppedState();

    vector<OneObjectFrame> rawPositions;

    Vector2D m_rawPosition;
    Vector2D m_displacement;
    Vector2D m_filteredPosition;

    Vector2D m_clusteredVelocity;

    Vector2D m_rawVelocity;
    Vector2D m_filteredVelocity;

    Vector2D m_acceleration;

    AlphaBetaFilter alphaBetaFilter;

    bool hasUnprocessedData;


//    int __medianFilterIndex;
};

#endif // _BALLFILTER_H
