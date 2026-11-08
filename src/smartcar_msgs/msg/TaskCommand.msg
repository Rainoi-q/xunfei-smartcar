# 单条原子任务指令（由 nlp_planner 从自然语言拆解得到）
string action        # go / grab / drop / wait / speak / stop
string target        # 目标，如 货架A / 3号门 / 红绿灯
float32 distance     # 目标距离（米），未知填 -1.0
string raw_text      # 该指令对应的原始文本片段，便于溯源
