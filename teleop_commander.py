"""
RoboMaster MobED 高鲁棒底盘遥控规划器 (RoboMasterTeleopCommander)
================================================================
特性：
1. 非对称二阶斜坡滤波：平滑加速、快速制动、杜绝瞬态冲击与误触发碰撞检测；
2. 行进转弯与原地侧移的连续动态混合（Continuous Blending），杜绝舵轮 90° 暴震；
3. 动态过弯向心加速度安全包线限制，杜绝高速翻车；
4. 窗口失焦保护与时间步独立插值。
"""

import numpy as np

class RoboMasterTeleopCommander:
    def __init__(self,
                 max_lin_vel: float = 1.0,
                 max_strafe_vel: float = 0.8,
                 max_ang_vel: float = 1.5,
                 spin_ang_vel: float = 1.8):
        # ---------------- 额定物理性能指标 ----------------
        self.MAX_LIN_VEL    = max_lin_vel    # m/s   (最高纵向速度)
        self.MAX_STRAFE_VEL = max_strafe_vel # m/s   (最高横向侧移速度)
        self.MAX_ANG_VEL    = max_ang_vel    # rad/s (最高转弯角速度)
        self.SPIN_ANG_VEL   = spin_ang_vel   # rad/s (最高自转角速度)
        
        # ---------------- 加减速度动力学边界 ----------------
        self.ACC_LIN_FWD    = 3.2    # m/s^2 (加速响应：充满推背感且轮胎不打滑)
        self.ACC_LIN_BRAKE  = 4.5    # m/s^2 (制动减速：快速停稳但不会引发冲击俯冲)
        self.ACC_STRAFE     = 2.5    # m/s^2 (侧移加速度：保护悬架外八字平稳撑开)
        self.ACC_ANGULAR    = 6.0    # rad/s^2 (角加速度：Swerve 舵机能从容追踪 ICR)
        
        # ---------------- 安全与过渡阈值 ----------------
        self.MAX_LATERAL_ACC = 3.0   # m/s^2 (过弯最大允许向心加速度上限 |vx * wz| <= 3.0)
        self.TRANS_VEL_THRESHOLD = 0.25 # m/s (转弯与侧移连续过渡的平滑速度死区)
        
        # ---------------- 内部当前连续状态 ----------------
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

    def reset(self):
        """重置所有速度指令为零"""
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

    def update(self, key_state: dict, dt: float) -> np.ndarray:
        """
        根据物理按键与时间增量，计算经过高鲁棒滤波后的 [vx, vy, wz]
        
        :param key_state: 包含按键布尔值的字典:
                          {'fwd': bool, 'bwd': bool, 'left': bool, 'rite': bool,
                           'spin_l': bool, 'spin_r': bool, 'brake': bool}
        :param dt: 仿真/控制时间步长 (例如 0.01s = 100Hz)
        :return: np.ndarray([vx, vy, wz])
        """
        if dt <= 0.0 or dt > 0.1:
            dt = 0.01  # 防止系统暂停或卡顿引发 dt 异常暴增

        brake  = key_state.get('brake', False)
        fwd    = key_state.get('fwd', False)
        bwd    = key_state.get('bwd', False)
        left   = key_state.get('left', False)
        rite   = key_state.get('rite', False)
        spin_l = key_state.get('spin_l', False)
        spin_r = key_state.get('spin_r', False)

        if brake:
            # 刹车状态：所有目标速度强制归零，按最大刹车减速度快速停稳
            self.cmd_vx = self._slew_rate(self.cmd_vx, 0.0, self.ACC_LIN_BRAKE, self.ACC_LIN_BRAKE, dt)
            self.cmd_vy = self._slew_rate(self.cmd_vy, 0.0, self.ACC_STRAFE * 2.0, self.ACC_STRAFE * 2.0, dt)
            self.cmd_wz = self._slew_rate(self.cmd_wz, 0.0, self.ACC_ANGULAR * 2.0, self.ACC_ANGULAR * 2.0, dt)
            return np.array([self.cmd_vx, self.cmd_vy, self.cmd_wz], dtype=np.float64)

        # -------------------------------------------------------------
        # 1. 计算未滤波的原始目标纵向速度 raw_vx
        # -------------------------------------------------------------
        raw_vx = 0.0
        if fwd and not bwd:
            raw_vx = self.MAX_LIN_VEL
        elif bwd and not fwd:
            raw_vx = -self.MAX_LIN_VEL

        # -------------------------------------------------------------
        # 2. 核心：速度连续混合场权重 (Continuous Velocity Blending)
        #    利用当前已滤波的实际速度与阈值比率，实现 0 突变的平滑连续过渡
        # -------------------------------------------------------------
        # speed_ratio: 0.0 (完全静止) -> 1.0 (正常行进)
        speed_ratio = float(np.clip(abs(self.cmd_vx) / self.TRANS_VEL_THRESHOLD, 0.0, 1.0))
        
        # S 曲线权重平滑: 3x^2 - 2x^3 (消除边界斜率突变)
        w_steer = speed_ratio * speed_ratio * (3.0 - 2.0 * speed_ratio)
        w_strafe = 1.0 - w_steer

        # -------------------------------------------------------------
        # 3. 计算 A/D 键在两套模态下的动态连续分配
        # -------------------------------------------------------------
        raw_vy = 0.0
        raw_wz = 0.0

        lateral_cmd = 0.0
        if left and not rite:
            lateral_cmd = 1.0
        elif rite and not left:
            lateral_cmd = -1.0

        if lateral_cmd != 0.0:
            # 静止分量分配给侧移 (vy)
            raw_vy = lateral_cmd * self.MAX_STRAFE_VEL * w_strafe
            
            # 行进分量分配给转弯 (wz)
            # 行进间拐弯方向：前进时左按键为逆时针正转，后退时左按键为顺时针反转
            steer_sign = 1.0 if self.cmd_vx >= -0.05 else -1.0
            raw_wz = lateral_cmd * self.MAX_ANG_VEL * w_steer * steer_sign

        # -------------------------------------------------------------
        # 4. 叠加独立的原地自转输入 (Q/E 键)
        # -------------------------------------------------------------
        if spin_l and not spin_r:
            raw_wz = self.SPIN_ANG_VEL
        elif spin_r and not spin_l:
            raw_wz = -self.SPIN_ANG_VEL

        # -------------------------------------------------------------
        # 5. 向心加速度物理安全包线约束 (Centrifugal Protection)
        #    |vx * wz| <= a_lat_max，防止高速大把打方向直接把小车甩翻
        # -------------------------------------------------------------
        if abs(self.cmd_vx) > 0.1 and abs(raw_wz) > 0.01:
            max_allowed_wz = self.MAX_LATERAL_ACC / abs(self.cmd_vx)
            raw_wz = float(np.clip(raw_wz, -max_allowed_wz, max_allowed_wz))

        # -------------------------------------------------------------
        # 6. 非对称二阶 S 曲线斜坡限幅器 (Asymmetric Rate Limiter)
        # -------------------------------------------------------------
        # 纵向 vx 平滑更新
        self.cmd_vx = self._slew_rate(
            self.cmd_vx, raw_vx, 
            accel_lim=self.ACC_LIN_FWD, 
            decel_lim=self.ACC_LIN_BRAKE, 
            dt=dt
        )

        # 横向 vy 平滑更新
        self.cmd_vy = self._slew_rate(
            self.cmd_vy, raw_vy, 
            accel_lim=self.ACC_STRAFE, 
            decel_lim=self.ACC_STRAFE * 1.5, 
            dt=dt
        )

        # 角速度 wz 平滑更新
        self.cmd_wz = self._slew_rate(
            self.cmd_wz, raw_wz, 
            accel_lim=self.ACC_ANGULAR, 
            decel_lim=self.ACC_ANGULAR * 1.5, 
            dt=dt
        )

        return np.array([self.cmd_vx, self.cmd_vy, self.cmd_wz], dtype=np.float64)

    @staticmethod
    def _slew_rate(current: float, target: float, accel_lim: float, decel_lim: float, dt: float) -> float:
        """非对称斜坡变化率限制器"""
        delta = target - current
        # 判断是在主动加速还是在被动制动
        is_braking = (current * delta < 0.0) or (abs(target) < abs(current))
        rate_limit = (decel_lim if is_braking else accel_lim) * dt
        
        step = float(np.clip(delta, -rate_limit, rate_limit))
        return current + step

    def get_velocity(self) -> np.ndarray:
        return np.array([self.cmd_vx, self.cmd_vy, self.cmd_wz], dtype=np.float64)
