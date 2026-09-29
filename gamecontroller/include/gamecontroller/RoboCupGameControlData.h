#ifndef ROBOCUPGAMECONTROLDATA_H
#define ROBOCUPGAMECONTROLDATA_H

#include <stdint.h>

// -------------------------------------------------------
// GameController Protocol Version 19 (198 bytes)
// -------------------------------------------------------

#define GAMECONTROLLER_DATA_PORT            3838
#define GAMECONTROLLER_RETURN_PORT          3939

#define GAMECONTROLLER_STRUCT_HEADER        "RGme"
#define GAMECONTROLLER_STRUCT_VERSION       20
#define GAMECONTROLLER_STRUCT_SIZE          158

#define MAX_NUM_PLAYERS                     20

// Competition type
#define COMPETITION_TYPE_SMALL              0
#define COMPETITION_TYPE_MIDDLE             1
#define COMPETITION_TYPE_LARGE              2

// Game phase
#define GAME_PHASE_NORMAL                   0
#define GAME_PHASE_PENALTY_SHOOT_OUT        1
#define GAME_PHASE_EXTRA_TIME               2
#define GAME_PHASE_TIMEOUT                  3

// State
#define STATE_INITIAL                       0
#define STATE_READY                         1
#define STATE_SET                           2
#define STATE_PLAYING                       3
#define STATE_FINISHED                      4

// Set play
#define SET_PLAY_NONE                       0
#define SET_PLAY_DIRECT_FREE_KICK           1
#define SET_PLAY_INDIRECT_FREE_KICK         2
#define SET_PLAY_PENALTY_KICK               3
#define SET_PLAY_THROW_IN                   4
#define SET_PLAY_GOAL_KICK                  5
#define SET_PLAY_CORNER_KICK                6

#define KICKING_TEAM_NONE                   255

// Penalty
#define NONE                                0
#define PENALTY_NONE                        0
#define PENALTY_ILLEGAL_POSITIONING         1
#define PENALTY_MOTION_IN_SET               2
#define PENALTY_LOCAL_GAME_STUCK            3
#define PENALTY_INCAPABLE_ROBOT             4
#define PENALTY_PICK_UP                     5
#define PENALTY_BALL_HOLDING                6
#define PENALTY_LEAVING_THE_FIELD           7
#define PENALTY_PLAYING_WITH_ARMS_HANDS     8
#define PENALTY_PUSHING                     9
#define PENALTY_SENT_OFF                    10
#define SUBSTITUTE                          11
#define MANUAL                              15

// secondstate 호환용 (uiUpdate에서 사용)
#define STATE2_NORMAL                       0
#define STATE2_PENALTYSHOOT                 1
#define STATE2_OVERTIME                     2
#define STATE2_TIMEOUT                      3
#define STATE2_DIRECT_FREEKICK              4
#define STATE2_INDIRECT_FREEKICK            5
#define STATE2_PENALTYKICK                  6
#define STATE2_CORNER_KICK                  7
#define STATE2_GOAL_KICK                    8
#define STATE2_THROW_IN                     9

struct RobotInfo
{
    uint8_t penalty;               // penalty state of the player
    uint8_t secsTillUnpenalised;   // estimate of time till unpenalised
    uint8_t cautions;              // number of cautions (yellow cards)
};

struct TeamInfo
{
    uint8_t  teamNumber;
    uint8_t  fieldPlayerColour;
    uint8_t  goalkeeperColour;
    uint8_t  goalkeeper;
    uint8_t  score;
    uint8_t  penaltyShot;
    uint16_t singleShots;
    uint16_t messageBudget;
    RobotInfo players[MAX_NUM_PLAYERS];
};

struct RoboCupGameControlData
{
    char     header[4];           // "RGme"
    uint8_t  version;             // 19
    uint8_t  packetNumber;
    uint8_t  playersPerTeam;
    uint8_t  competitionType;     // COMPETITION_TYPE_*
    uint8_t  stopped;             // 1 = play is currently stopped, 0 otherwise
    uint8_t  gamePhase;           // GAME_PHASE_*
    uint8_t  state;               // STATE_*
    uint8_t  setPlay;             // SET_PLAY_*
    uint8_t  firstHalf;           // 1 = first half
    uint8_t  kickingTeam;         // team number of kicking team (255 = none)
    int16_t  secsRemaining;       // seconds remaining in half
    int16_t  secondaryTime;       // secondary timer
    TeamInfo teams[2];
};

// -------------------------------------------------------
// Return struct (robot → GameController3)  version 4, 32 bytes
// -------------------------------------------------------
#define GAMECONTROLLER_RETURN_STRUCT_HEADER     "RGrt"
#define GAMECONTROLLER_RETURN_STRUCT_VERSION    4
#define GAMECONTROLLER_RETURN_STRUCT_SIZE       32

struct RoboCupGameControlReturnData
{
    char    header[4];      // "RGrt"
    uint8_t version;        // 4
    uint8_t playerNum;      // 1-based player number
    uint8_t teamNum;        // team number
    uint8_t fallen;         // 1 = fallen, 0 = upright
    float   pose[3];        // x, y, theta  (f32 LE)
    float   ballAge;        // seconds since last ball seen  (-1 = unknown)
    float   ball[2];        // ball x, y relative to robot  (f32 LE)

#ifdef __cplusplus
    RoboCupGameControlReturnData()
        : version(GAMECONTROLLER_RETURN_STRUCT_VERSION),
          playerNum(0), teamNum(0), fallen(0),
          pose{0.0f, 0.0f, 0.0f},
          ballAge(-1.0f),
          ball{0.0f, 0.0f}
    {
        const char* init = GAMECONTROLLER_RETURN_STRUCT_HEADER;
        for (unsigned int i = 0; i < sizeof(header); ++i)
            header[i] = init[i];
    }
#endif
};

#endif // ROBOCUPGAMECONTROLDATA_H
