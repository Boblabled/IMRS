import mujoco
import mujoco.viewer
import time
import numpy as np
import cv2

if __name__ == '__main__':

    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)

    # Параметры управления (пример для квадрокоптера)
    thrust = 3.2495625  # Стартовое значение тяги (из keyframe)
    ctrl_range = [0, 13]  # Диапазон управления из модели


    # Функция для применения управления
    def apply_control(data, thrust_values):
        for i, name in enumerate(['thrust1', 'thrust2', 'thrust3', 'thrust4']):
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator_id != -1:
                data.ctrl[actuator_id] = thrust_values[i]


    # Инициализация управления (значения из keyframe)
    initial_ctrl = [thrust] * 4
    apply_control(data, initial_ctrl)

    # Запуск интерактивного просмотра
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            # Пример простого управления (можно заменить на свою логику)
            elapsed = time.time() - start_time
            sin_wave = np.sin(elapsed * 2)  # Осцилляция для демонстрации

            # Меняем тягу роторов по синусоиде для демонстрации
            coeff = 0.1
            thrust_values = [
                thrust + sin_wave * coeff,
                thrust + sin_wave * coeff,
                thrust + sin_wave * coeff,
                thrust + sin_wave * coeff
            ]

            apply_control(data, thrust_values)

            # Шаг симуляции
            mujoco.mj_step(model, data)

            # Синхронизация с реальным временем
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # Обновление viewer
            viewer.sync()