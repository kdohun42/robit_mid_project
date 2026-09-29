int64 robotnum
int64 position
int64 myteam

int64 state
string state_name
int64 stopped
int64 myside
int64 iskickoff
int64 readytime
int64 penalty
int64 secondstate
int64[] secondinfo

# 우리 팀에 남은 전송 가능 메시지 수 (TeamInfo::messageBudget).
# 팀 전체가 공유하는 잔여량이며 GameController가 차감한다.
int64 message_budget
