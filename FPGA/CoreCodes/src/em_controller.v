//******************************************************************************
// em_controller.v — 推拉式电磁铁 FPGA 控制器
//
// 功能:
//   - 4路推拉式电磁铁, 继电器高电平触发 (active HIGH = 推杆伸出)
//   - 硬件互斥: 同一时刻最多只有1路输出为高 (防止电源过载爆炸)
//   - 定时自动缩回: 推出后经 em_duration×10ms 自动关断
//   - 强制冷却期: 缩回后强制等待 cooldown×10ms 才允许下次触发
//   - 急停: estop 立即关断所有输出
//
// 接口:
//   clk_50m      — 50MHz 系统时钟
//   rst_n        — 复位 (低有效)
//   em_target    — 目标推杆号: 4'b0001=EM1, 4'b0010=EM2, 4'b0100=EM3, 4'b1000=EM4
//   em_trig      — 触发脉冲 (上升沿有效, 电平触发)
//   em_duration  — 推出时长 (×10ms), 默认 20 = 200ms
//   cooldown     — 冷却间隔 (×10ms), 默认 50 = 500ms, 设为0跳过冷却
//   estop        — 急停: 立即关断所有输出, 复位状态机
//
//   em1..em4     — 电磁铁 GPIO 输出 (1=推杆伸出/继电器吸合)
//   busy         — 忙标志 (ACTIVE或COOLDOWN期间为1)
//   active_idx   — 当前激活的推杆号 (0=无, 1-4=EM1-EM4)
//******************************************************************************

module em_controller (
    input  wire         clk_50m,
    input  wire         rst_n,

    // 控制输入
    input  wire [3:0]   em_target,      // one-hot 目标推杆
    input  wire         em_trig,         // 触发脉冲
    input  wire [7:0]   em_duration,     // 推出时长 ×10ms
    input  wire [7:0]   cooldown,        // 冷却时长 ×10ms
    input  wire         estop,           // 急停

    // EM 输出
    output reg  [3:0]   em_out,          // {em4,em3,em2,em1} 高电平有效

    // 状态
    output wire         busy,            // 1=忙 (不可触发)
    output reg  [2:0]   active_idx       // 0=空闲, 1-4=当前激活的EM号
);

    //==========================================================================
    // 状态机
    //==========================================================================
    localparam S_IDLE     = 2'd0;   // 空闲, 等待触发
    localparam S_ACTIVE   = 2'd1;   // 推杆伸出中
    localparam S_COOLDOWN = 2'd2;   // 强制冷却中

    reg [1:0]  state;
    reg [24:0] timer;               // 倒计时 (50MHz ticks)
    reg [3:0]  active_target;       // 锁存的激活目标
    reg        trig_acked;          // 已响应标志: 防止电平持续为高时重复触发

    // 10ms = 500,000 ticks @ 50MHz
    localparam TICKS_10MS = 25'd500_000;

    //==========================================================================
    // 看门狗: 如果 em_out 出现多bit (硬件错误), 强制清零
    //==========================================================================
    wire em_fault = (em_out[0] + em_out[1] + em_out[2] + em_out[3]) > 1'b1;

    assign busy = (state != S_IDLE);

    //==========================================================================
    // 状态机 + 定时器
    //==========================================================================
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            timer         <= 25'd0;
            active_target <= 4'd0;
            em_out        <= 4'd0;
            active_idx    <= 3'd0;
            trig_acked    <= 1'b0;
        end else begin
            // em_trig=0 时清除 ack, 允许下次触发
            if (!em_trig) trig_acked <= 1'b0;
            // 急停: 立即关断一切, 回 IDLE
            if (estop) begin
                state         <= S_IDLE;
                timer         <= 25'd0;
                active_target <= 4'd0;
                em_out        <= 4'd0;
                active_idx    <= 3'd0;
            end
            // 看门狗: 多bit故障立即关断
            else if (em_fault) begin
                em_out     <= 4'd0;
                active_idx <= 3'd0;
                state      <= S_IDLE;
                timer      <= 25'd0;
            end
            else begin
                case (state)

                    // ---- IDLE: 等待触发 ----
                    S_IDLE: begin
                        em_out <= 4'd0;
                        // 触发条件: em_trig=1且未响应过, 目标有效(one-hot)
                        if (em_trig && !trig_acked && em_target != 4'd0 &&
                            (em_target == 4'b0001 || em_target == 4'b0010 ||
                             em_target == 4'b0100 || em_target == 4'b1000)) begin
                            active_target <= em_target;
                            em_out       <= em_target;   // 硬件互斥: 只设1bit
                            trig_acked   <= 1'b1;        // 标记已响应, 防止重复触发
                            active_idx   <= (em_target[0] ? 3'd1 :
                                             em_target[1] ? 3'd2 :
                                             em_target[2] ? 3'd3 : 3'd4);
                            // 定时器 = duration × 10ms ticks
                            timer        <= {17'd0, em_duration} * TICKS_10MS;
                            state        <= S_ACTIVE;
                        end else begin
                            active_idx <= 3'd0;
                        end
                    end

                    // ---- ACTIVE: 推杆伸出, 倒计时 ----
                    S_ACTIVE: begin
                        if (timer > 25'd1) begin
                            timer <= timer - 25'd1;
                        end else begin
                            // 时间到, 关断输出
                            em_out     <= 4'd0;
                            active_idx <= 3'd0;
                            // 进入冷却 (如果 cooldown=0 则跳过)
                            if (cooldown != 8'd0) begin
                                timer      <= {17'd0, cooldown} * TICKS_10MS;
                                state      <= S_COOLDOWN;
                            end else begin
                                state <= S_IDLE;
                            end
                        end
                    end

                    // ---- COOLDOWN: 强制冷却, 禁止触发 ----
                    S_COOLDOWN: begin
                        em_out <= 4'd0;
                        if (timer > 25'd1) begin
                            timer <= timer - 25'd1;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
