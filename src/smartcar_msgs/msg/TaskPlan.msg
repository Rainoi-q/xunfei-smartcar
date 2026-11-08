# 任务计划（对应功能②多指令连续执行、③多级语义识别）
std_msgs/Header header

# 状态常量
uint8 PENDING=0
uint8 RUNNING=1
uint8 DONE=2
uint8 FAILED=3

string raw_command      # 用户原始语音转写文本
TaskCommand[] commands  # 拆解后的原子任务序列，按执行顺序排列
uint8 status            # 整体状态，取值见上方常量
int32 current_index     # 当前执行到第几条命令
string message          # 状态说明或错误信息
