#!/usr/bin/env python3
"""
Simulação do Robô RRRP para Soldagem - Versão 7.0 (Correção e Análise)

Autor: André MK022 (Cuca) - Refinado com feedback por Gemini
Descrição: Corrige o bug na geração da trajetória "cotovelo para baixo" usando
um perfil de q3 adequado. Adiciona visualização do espaço de trabalho para
melhor compreensão da cinemática do robô.
"""
import matplotlib
matplotlib.use("Qt5Agg")
import numpy as np
import matplotlib.pyplot as plt

try:
    from roboticstoolbox import DHRobot, RevoluteDH
    from spatialmath import SE3
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("Robotics Toolbox não encontrada. Usando fallback para plotagem com Matplotlib.")
    ROBOTICS_TOOLBOX_AVAILABLE = False

# --- Funções do Robô e Cinemática (sem alterações da V6) ---
def create_rrr_robot():
    l1, l2, l3 = 0.2, 0.3, 0.25
    if ROBOTICS_TOOLBOX_AVAILABLE:
        links = [RevoluteDH(d=l1, alpha=np.pi/2), RevoluteDH(a=l2), RevoluteDH(a=l3)]
        return DHRobot(links, name='RRR_Base'), l1, l2, l3
    return None, l1, l2, l3

def get_tcp_position(q, robot, robot_params):
    if ROBOTICS_TOOLBOX_AVAILABLE and robot is not None:
        return robot.fkine(q[:3]) * SE3.Tx(q[3])
    else:
        l1, l2, l3 = robot_params; q1, q2, q3, q4 = q
        x_local = l2*np.cos(q2) + l3*np.cos(q2+q3) + q4
        x, y = x_local*np.cos(q1), x_local*np.sin(q1)
        z = l1 + l2*np.sin(q2) + l3*np.sin(q2+q3)
        return SE3(x, y, z)

def solve_ik_for_vertical_trajectory(l1, l2, l3, horizontal_reach, N, z_range, q3_range, elbow_config='up'):
    z_start, z_end = z_range
    z_vals = np.linspace(z_start, z_end, N)
    q3_start, q3_end = q3_range
    q3_vals = np.linspace(q3_start, q3_end, N)
    elbow_sign = 1.0 if elbow_config == 'up' else -1.0
    Q_partial = []
    for i, z_target in enumerate(z_vals):
        q3 = q3_vals[i]
        M = z_target - l1
        K1, K2 = l2 + l3 * np.cos(q3), l3 * np.sin(q3)
        delta = K1**2 + K2**2 - M**2
        if delta < 0: continue
        den = np.sqrt(delta)
        q2 = np.arctan2(M, elbow_sign * den) - np.arctan2(K2, K1)
        x_arm_projection = l2 * np.cos(q2) + l3 * np.cos(q2 + q3)
        q4 = horizontal_reach - x_arm_projection
        if 0 <= q4 <= 0.2:
            Q_partial.append([q2, q3, q4, z_target])
    return np.array(Q_partial)

def generate_trajectory(wall_type, wall_params, robot_params, elbow_config, N=150, safety_margin=0.01):
    """Gera a trajetória, agora com a configuração do cotovelo como parâmetro."""
    l1, l2, l3 = robot_params
    print(f"\nCalculando trajetória '{elbow_config}'...")
    if wall_type == 'x':
        X_wall = wall_params; q1_const = 0.0; horizontal_reach = X_wall; wall_pos_const = (X_wall, 0.0)
    elif wall_type == 'y':
        x_const, Y_wall = wall_params
        q1_const = np.arctan2(Y_wall, x_const); horizontal_reach = np.sqrt(x_const**2 + Y_wall**2); wall_pos_const = (x_const, Y_wall)

    z_range = (l1, l1 + l2 + l3 - safety_margin)
    
    # *** A CORREÇÃO ESTÁ AQUI ***
    # Define um perfil de q3 diferente para cada configuração do cotovelo
    if elbow_config == 'up':
        q3_range = (np.pi/2, 0) # Dobrado para cima -> esticado
    else: # 'down'
        q3_range = (-np.pi/2, 0) # Dobrado para baixo -> esticado

    Q_partial = solve_ik_for_vertical_trajectory(l1, l2, l3, horizontal_reach, N, z_range, q3_range, elbow_config)

    if len(Q_partial) == 0: return np.array([]), np.array([])
        
    num_valid_points = len(Q_partial)
    Q = np.zeros((num_valid_points, 4)); Q[:, 0] = q1_const; Q[:, 1:4] = Q_partial[:, 0:3]
    z_ref = Q_partial[:, 3]
    pts_reference = np.array([[wall_pos_const[0], wall_pos_const[1], z] for z in z_ref])
    return Q, pts_reference

def plot_workspace_and_tool(env, robot, q, pts_reference):
    """
    Função auxiliar para a animação.
    Desenha o espaço de trabalho do braço RRR e a ferramenta prismática.
    """
    # Desenha o espaço de trabalho (apenas uma vez, se não existir)
    if not hasattr(env, 'workspace_plotted'):
        l1, l2, l3 = robot.links[0].d, robot.links[1].a, robot.links[2].a
        # Círculo externo (braço esticado)
        radius_max = l2 + l3
        # Círculo interno (braço dobrado)
        radius_min = abs(l2 - l3)
        theta = np.linspace(0, 2 * np.pi, 100)
        # Rotaciona o workspace de acordo com a base do robô
        q1 = q[0]
        x_max = radius_max * np.cos(theta) * np.cos(q1)
        y_max = radius_max * np.cos(theta) * np.sin(q1)
        x_min = radius_min * np.cos(theta) * np.cos(q1)
        y_min = radius_min * np.cos(theta) * np.sin(q1)
        env.ax.plot(x_max, y_max, l1, 'k:', alpha=0.5, label='Workspace (z=l1)')
        env.ax.plot(x_min, y_min, l1, 'k:', alpha=0.5)
        env.workspace_plotted = True
        env.ax.legend()

    # Desenha a ferramenta prismática
    wrist_pose = robot.fkine(q[:3])
    tcp_pose = wrist_pose * SE3.Tx(q[3])
    
    # Cria a linha da ferramenta se não existir, senão atualiza
    if not hasattr(env, 'tool_line'):
        env.tool_line, = env.ax.plot([], [], [], 'g-', linewidth=4, label='Ferramenta (q₄)')
        env.ax.legend()
    
    env.tool_line.set_data_3d(
        [wrist_pose.t[0], tcp_pose.t[0]],
        [wrist_pose.t[1], tcp_pose.t[1]],
        [wrist_pose.t[2], tcp_pose.t[2]]
    )

def plot_trajectory_rtb(robot, Q, pts_reference, label):
    """Anima a trajetória e visualiza a ferramenta e o workspace."""
    print(f"\nIniciando animação para: {label}...")
    env = robot.plot(Q[0, :3], block=False, limits=[-0.8, 0.8, -0.8, 0.8, 0, 1.0])
    env.ax.plot(pts_reference[:, 0], pts_reference[:, 1], pts_reference[:, 2], 'r--', linewidth=2, label='Parede Alvo')
    
    for i in range(len(Q)):
        robot.q = Q[i, :3]
        plot_workspace_and_tool(env, robot, Q[i], pts_reference)
        env.step(0.05)
    
    print("Animação concluída. Feche a janela para sair.")
    plt.show(block=True)

def main():
    print("=== Simulação do Robô RRRP - Versão 7.0 (Correção e Análise) ===")
    robot, l1, l2, l3 = create_rrr_robot()
    robot_params = (l1, l2, l3)
    if robot: print(f"Robô RRR base criado! l1={l1}, l2={l2}, l3={l3}")

    wall_params = (0.3, 0.4)
    trajectories_to_run = ['up', 'down']
    generated_trajectories = {}

    for config in trajectories_to_run:
        Q, pts_ref = generate_trajectory('y', wall_params, robot_params, elbow_config=config)
        if len(Q) > 0:
            print(f"✓ Trajetória '{config}' gerada com {len(Q)} pontos.")
            generated_trajectories[config] = (Q, pts_ref)
        else:
            print(f"✗ ERRO: A trajetória '{config}' não pôde ser gerada. A parede pode estar fora do alcance para esta configuração.")

    if not generated_trajectories:
        print("\nNenhuma trajetória foi gerada com sucesso. Encerrando.")
        return

    if ROBOTICS_TOOLBOX_AVAILABLE:
        for config, (Q, pts_ref) in generated_trajectories.items():
            plot_trajectory_rtb(robot, Q, pts_ref, label=f"Cotovelo para {config.capitalize()}")
    else:
        # Fallback para Matplotlib se o RTB não estiver disponível
        print("\nRobotics Toolbox não disponível. Use a V6 para plotagem de comparação no Matplotlib.")

    print("\nSimulação concluída!")

if __name__ == "__main__":
    main()