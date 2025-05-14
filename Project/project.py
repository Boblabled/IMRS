# import mujoco
# import mujoco.viewer
# import numpy as np
# import time
#
# # Параметры PID-регуляторов
# class PID:
#     def __init__(self, kp, ki, kd, i_limit=np.inf):
#         self.kp, self.ki, self.kd = kp, ki, kd
#         self.i_limit = i_limit
#         self.prev_error = 0.0
#         self.integral = 0.0
#
#     def reset(self):
#         self.prev_error = 0.0
#         self.integral = 0.0
#
#     def compute(self, error, dt):
#         self.integral += error * dt
#         self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
#         derivative = (error - self.prev_error) / dt if dt > 0 else 0
#         self.prev_error = error
#         return self.kp * error + self.ki * self.integral + self.kd * derivative
#
# # PID контроллеры
# z_pid = PID(kp=15.0, ki=5.0, kd=10.0)
# roll_pid = PID(kp=4.0, ki=0.1, kd=2.5)
# pitch_pid = PID(kp=4.0, ki=0.1, kd=2.5)
# yaw_pid = PID(kp=1.0, ki=0.05, kd=0.5)
#
# # Целевая позиция
# target_z = 1.0
# target_rpy = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
#
# def quaternion_to_euler(q):
#     w, x, y, z = q
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll = np.arctan2(t0, t1)
#
#     t2 = +2.0 * (w * y - z * x)
#     t2 = np.clip(t2, -1.0, 1.0)
#     pitch = np.arcsin(t2)
#
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw = np.arctan2(t3, t4)
#
#     return np.array([roll, pitch, yaw])
#
#
# # Добавим PID по позиции
# x_pid = PID(kp=1.0, ki=0.0, kd=0.5)
# y_pid = PID(kp=1.0, ki=0.0, kd=0.5)
#
# # Целевая позиция [x, y, z]
# # target_pos = np.array([2.0, 0.0, 1.0])  # Например, полёт в точку x=2.0, y=0.0, z=1.0
# # target_yaw = 0.0  # Можно стабилизировать направление
#
#
# # Цель
# target_z = 1.5      # высота
# target_vx = 1.0     # скорость вперёд в м/с
#
# # Контроллеры
# Kpz = 15.0          # для высоты
# Kvx = 1.5           # для скорости
#
# hover_thrust = 3.25
#
#
# def motion_control(model, data):
#     global dt
#
#     # --- Контроль высоты ---
#     current_z = data.qpos[2]
#     error_z = target_z - current_z
#     dz = Kpz * error_z
#
#     # --- Контроль скорости вперёд (по x) ---
#     current_vx = data.qvel[0]
#     error_vx = target_vx - current_vx
#     pitch_offset = Kvx * error_vx
#
#     # --- Итоговая тяга ---
#     thrust = hover_thrust + dz
#     thrust = np.clip(thrust, 2.0, 5.0)
#     pitch_offset = np.clip(pitch_offset, -0.5, 0.5)  # ограничим наклон
#
#     # --- Управление моторами ---
#     thrust_values = [thrust + pitch_offset,
#                      thrust - pitch_offset,
#                      thrust - pitch_offset,
#                      thrust + pitch_offset]
#
#     # data.ctrl[0] = thrust + pitch_offset  # задний левый
#     # data.ctrl[1] = thrust - pitch_offset  # передний левый
#     # data.ctrl[2] = thrust - pitch_offset  # передний правый
#     # data.ctrl[3] = thrust + pitch_offset  # задний правый
#
#     #
#     #
#     # pos = data.qpos[:3]  # x, y, z
#     # quat = data.sensor('body_quat').data
#     # rpy = quaternion_to_euler(quat)
#     #
#     # # Ошибки по положению
#     # x_error = target_pos[0] - pos[0]
#     # y_error = target_pos[1] - pos[1]
#     # z_error = target_pos[2] - pos[2]
#     #
#     # # Позиционные PIDs → нужные углы наклона
#     # pitch_target = -x_pid.compute(x_error, dt)  # наклон вперёд/назад
#     # roll_target = y_pid.compute(y_error, dt)    # наклон влево/вправо
#     #
#     # # Ограничим углы чтобы не переворачивался
#     # pitch_target = np.clip(pitch_target, -0.4, 0.4)
#     # roll_target = np.clip(roll_target, -0.4, 0.4)
#     #
#     # # Ошибки по ориентации
#     # roll_error = roll_target - rpy[0]
#     # pitch_error = pitch_target - rpy[1]
#     # yaw_error = target_yaw - rpy[2]
#     #
#     # # PID управление
#     # z_control = z_pid.compute(z_error, dt)
#     # roll_control = roll_pid.compute(roll_error, dt)
#     # pitch_control = pitch_pid.compute(pitch_error, dt)
#     # yaw_control = yaw_pid.compute(yaw_error, dt)
#     #
#     # # Распределение тяги
#     # u1 = z_control - roll_control - pitch_control - yaw_control
#     # u2 = z_control - roll_control + pitch_control + yaw_control
#     # u3 = z_control + roll_control + pitch_control - yaw_control
#     # u4 = z_control + roll_control - pitch_control + yaw_control
#
#     # thrust_values = [u1, u2, u3, u4]
#
#     for i, name in enumerate(['thrust1', 'thrust2', 'thrust3', 'thrust4']):
#         actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
#         if actuator_id != -1:
#             data.ctrl[actuator_id] = thrust_values[i]
#
#
# model = mujoco.MjModel.from_xml_path("scene.xml")  # замените на путь к XML
# data = mujoco.MjData(model)
#
# dt = 0
# mujoco.set_mjcb_control(motion_control)
# # Основной цикл управления
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start_time = time.time()
#
#     while viewer.is_running():
#         step_start = time.time()
#         dt = time.time() - start_time
#         mujoco.mj_step(model, data)
#
#         # Синхронизация с реальным временем
#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)
#
#         # Обновление viewer
#         viewer.sync()



import mujoco
import mujoco.viewer
import numpy as np
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QApplication

from Project.plotter import PlotWidget

if __name__ == '__main__':

    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)

    class PID:
        def __init__(self, kp, ki, kd, i_limit=np.inf):
            self.kp, self.ki, self.kd = kp, ki, kd
            self.i_limit = i_limit
            self.prev_error = 0.0
            self.integral = 0.0

        def reset(self):
            self.prev_error = 0.0
            self.integral = 0.0

        def compute(self, error, dt):
            self.integral += error * dt
            self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)
            derivative = (error - self.prev_error) / dt if dt > 0 else 0
            self.prev_error = error
            return self.kp * error + self.ki * self.integral + self.kd * derivative


    def quaternion_to_euler(q):
        w, x, y, z = q
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw])

    # Цель
    target_z = 3  # заданная высота
    z_pid = PID(kp=1, ki=.5, kd=.5)
    vx_pid = PID(kp=1, ki=1, kd=2)
    # pitch_pid = PID(kp=10, ki=2, kd=1)
    roll_pid = PID(kp=4.0, ki=0.1, kd=2.5)
    pitch_pid = PID(kp=4.0, ki=0.1, kd=2.5)
    yaw_pid = PID(kp=1.0, ki=0.05, kd=0.5)

    target_vx = -1e-6
    # target_vx = 1

    count = 0
    # print(model.body_mass)

    app = QApplication([])
    plotWidget = PlotWidget()
    plotWidget.show()
    app.exec()


    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)

            # --- Контроль высоты ---
            dt = model.opt.timestep
            count += dt
            if count > 1000:
                count = 0
                # target_z = target_z + 1
            current_z = data.qpos[2]  # текущая высота (по оси Z)
            error_z = target_z - current_z  # ошибка по высоте
            # print("target", target_z, "z_current", current_z, "z_error", error_z)
            thrust = z_pid.compute(error_z, dt)


            # print(data.qvel[0])
            current_vx = data.qvel[0]
            e_vx = target_vx - current_vx
            pitch_target = vx_pid.compute(e_vx, dt)
            print("v_target", target_vx, "v_current", current_vx, "v_error", e_vx)

            quat = data.sensor('body_quat').data
            rpy = quaternion_to_euler(quat)

            pitch_target = np.clip(pitch_target, -0.5, 0.5)
            e_pitch = pitch_target - rpy[1]
            # print(rpy)
            pitch_control = pitch_pid.compute(e_pitch, dt)
            print("pitch_target", pitch_target, "pitch", rpy[1], "e_pitch", e_pitch)
            print("pitch_ctrl", pitch_control)

            # --- Управление моторами ---
            # data.ctrl[0] = thrust + pitch_ctrl  # передний левый
            # data.ctrl[1] = thrust + pitch_ctrl  # передний правый
            # data.ctrl[0] = thrust - 1e-15  # передний левый
            # data.ctrl[1] = thrust - 1e-15  # передний правый
            # pitch_ctrl = pitch_ctrl / 2
            ctrl_range = [-13, 13]
            # pitch_ctrl = 0
            # u1 = np.clip(thrust + pitch_control, *ctrl_range)  # передний левый мотор
            # u2 = np.clip(thrust + pitch_control, *ctrl_range)  # передний правый мотор
            # u3 = np.clip(thrust - pitch_control, *ctrl_range)  # задний правый мотор
            # u4 = np.clip(thrust - pitch_control, *ctrl_range)  # задний левый мотор

            roll_control = roll_pid.compute(0 - rpy[0], dt)
            yaw_control = yaw_pid.compute(0 - rpy[2], dt)
            print(roll_control, yaw_control)

            u1 = np.clip(thrust - roll_control + pitch_control + yaw_control, *ctrl_range)
            u2 = np.clip(thrust + roll_control + pitch_control - yaw_control, *ctrl_range)
            u3 = np.clip(thrust - roll_control - pitch_control - yaw_control, *ctrl_range)
            u4 = np.clip(thrust + roll_control - pitch_control + yaw_control, *ctrl_range)


            data.ctrl = [u1, u2, u3, u4]

            # data.ctrl[0] = thrust  # передний левый мотор
            # data.ctrl[1] = thrust  # передний правый мотор
            # data.ctrl[2] = thrust  # задний правый мотор
            # data.ctrl[3] = thrust  # задний левый мотор
            print("\n")
            viewer.sync()  # обновляем состояние экрана
