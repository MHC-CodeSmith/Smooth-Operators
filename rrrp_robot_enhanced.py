#!/usr/bin/env python3
"""
Simulação do Robô RRRP - Versão 10.0 (Cinemática Corrigida)

Autor: André MK022 (Cuca) - Refinado com feedback por Gemini
Descrição: Corrige um bug crítico na cinemática inversa. O novo solver usa
a geometria exata do robô (Lei dos Cossenos) para garantir que a posição
do efetuador seja precisa e consistente com o modelo de simulação,
eliminando o erro que causava a parada de emergência.
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

# --- create_real_robot (sem alterações) ---
def create_real_robot():
    l1, l2, l3 = 0.1, 0.2, 0.1
    if ROBOTICS_TOOLBOX_AVAILABLE:
        q1_lim, q2_lim, q3_lim = [0, 0], [0, np.pi], [-np.pi/2, np.pi/2]
        links = [
            RevoluteDH(d=l1, alpha=np.pi/2, qlim=q1_lim),
            RevoluteDH(a=l2, alpha=0, qlim=q2_lim),
            RevoluteDH(a=l3, alpha=0, qlim=q3_lim),
        ]
        return DHRobot(links, name='Real_RRR_Base'), l1, l2, l3
    return None, l1, l2, l3

# --- NOVO SOLVER DE CINEMÁTICA INVERSA (CORAÇÃO DA CORREÇÃO) ---
def solve_ik_with_constraints_corrected(robot_params, q4_limits, horizontal_reach, N, elbow_config='up'):
    """
    Resolve a cinemática inversa usando a geometria exata do robô (Lei dos Cossenos).
    Este método é matematicamente consistente com a cinemática direta.
    """
    l1, l2, l3 = robot_params
    q4_min, q4_max = q4_limits
    q2_min, q2_max = 0, np.pi
    q3_min, q3_max = -np.pi/2, np.pi/2

    z_range = (l1, l1 + l2 + l3 - 0.001) # Z vai quase do chão ao teto
    z_vals = np.linspace(z_range[0], z_range[1], N)

    if elbow_config == 'up': q3_range = (q3_max, 0)
    else: q3_range = (q3_min, 0)
    q3_vals = np.linspace(q3_range[0], q3_range[1], N)

    elbow_sign = 1.0 if elbow_config == 'up' else -1.0
    Q_partial, rejected_points = [], {'q2': 0, 'q4': 0, 'reach': 0, 'acos':0}

    for i, z_target in enumerate(z_vals):
        q3 = q3_vals[i]
        
        # Ponto alvo no plano do robô
        x_p, z_p = horizontal_reach, z_target - l1
        D_sq = x_p**2 + z_p**2

        # Resolve uma equação quadrática para q4: a*q4^2 + b*q4 + c = 0
        # Derivada da Lei dos Cossenos no triângulo Ombro-Cotovelo-TCP
        a = 1.0
        b = 2 * (l3 + l2 * np.cos(q3))
        c = l2**2 + l3**2 + 2*l2*l3*np.cos(q3) - D_sq
        
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            rejected_points['reach'] += 1
            continue
        
        # Encontra as duas possíveis soluções para q4
        sqrt_discriminant = np.sqrt(discriminant)
        q4_sol1 = (-b + sqrt_discriminant) / (2*a)
        q4_sol2 = (-b - sqrt_discriminant) / (2*a)

        # Escolhe a solução para q4 que está dentro dos limites
        q4 = -1
        if q4_min <= q4_sol1 <= q4_max: q4 = q4_sol1
        elif q4_min <= q4_sol2 <= q4_max: q4 = q4_sol2
        else: rejected_points['q4'] += 1; continue
            
        # Com q4 conhecido, calcula q2
        L_eff = l3 + q4
        gamma = np.arctan2(z_p, x_p)
        
        # Argumento para o acos, da Lei dos Cossenos
        acos_arg = (l2**2 + D_sq - L_eff**2) / (2 * l2 * np.sqrt(D_sq))
        if not (-1.0 <= acos_arg <= 1.0):
             rejected_points['acos'] += 1
             continue

        beta = np.arccos(acos_arg)
        q2 = gamma - elbow_sign * beta
        
        if not (q2_min <= q2 <= q2_max):
            rejected_points['q2'] += 1
            continue
        
        Q_partial.append([q2, q3, q4, z_target])

    print(f" > Análise da Geração ({elbow_config}): Pontos Rejeitados por Limite [Alcance: {rejected_points['reach']}, ACOS: {rejected_points['acos']}, q2: {rejected_points['q2']}, q4: {rejected_points['q4']}]")
    return np.array(Q_partial)

def generate_frontal_trajectory(X_wall, robot_params, elbow_config, N=200):
    """Gera a trajetória para uma parede frontal, usando o novo solver."""
    print(f"\nCalculando trajetória '{elbow_config}' para parede frontal em x={X_wall}m...")
    q4_limits = [0, 0.15]
    
    Q_partial = solve_ik_with_constraints_corrected(robot_params, q4_limits, X_wall, N, elbow_config)

    if len(Q_partial) == 0: return np.array([]), np.array([])
        
    num_valid_points = len(Q_partial)
    Q = np.zeros((num_valid_points, 4)); Q[:, 0] = 0; Q[:, 1:4] = Q_partial[:, 0:3]
    z_ref = Q_partial[:, 3]
    pts_reference = np.array([[X_wall, 0, z] for z in z_ref])
    return Q, pts_reference

# --- Funções de plotagem e main (sem alterações, mas agora receberão dados corretos) ---
def plot_trajectory_rtb(robot, Q, pts_reference, label, X_wall):
    if not ROBOTICS_TOOLBOX_AVAILABLE: return
    print(f"\nIniciando animação para: {label}...")
    error_tolerance, tf_axis_length = 1e-4, 0.05
    env = robot.plot(Q[0, :3], block=False, limits=[-0.5, 0.5, -0.5, 0.5, 0, 0.5])
    env.ax.plot(pts_reference[:, 0], pts_reference[:, 1], pts_reference[:, 2], 'r--', linewidth=2, label='Parede Alvo')
    tool_line, = env.ax.plot([], [], [], 'g-', linewidth=4, label='Ferramenta (q₄)')
    tf_x, = env.ax.plot([], [], [], color='red', linewidth=2); tf_y, = env.ax.plot([], [], [], color='lime', linewidth=2); tf_z, = env.ax.plot([], [], [], color='blue', linewidth=2)
    env.ax.legend()
    for i in range(len(Q)):
        robot.q = Q[i, :3]
        wrist_pose = robot.fkine(robot.q); tcp_pose = wrist_pose * SE3.Tx(Q[i, 3])
        current_x = tcp_pose.t[0]
        if abs(current_x - X_wall) > error_tolerance:
            print(f"\n!!! PARADA DE EMERGÊNCIA !!!\n  > Desvio da parede excedeu a tolerância no passo {i}.\n  > Erro: {abs(current_x - X_wall)*1000:.3f} mm")
            break
        tool_line.set_data_3d([wrist_pose.t[0], tcp_pose.t[0]], [wrist_pose.t[1], tcp_pose.t[1]], [wrist_pose.t[2], tcp_pose.t[2]])
        p = tcp_pose.t; x_axis, y_axis, z_axis = tcp_pose.R[:, 0], tcp_pose.R[:, 1], tcp_pose.R[:, 2]
        tf_x.set_data_3d([p[0], p[0] + tf_axis_length * x_axis[0]], [p[1], p[1] + tf_axis_length * x_axis[1]], [p[2], p[2] + tf_axis_length * x_axis[2]])
        tf_y.set_data_3d([p[0], p[0] + tf_axis_length * y_axis[0]], [p[1], p[1] + tf_axis_length * y_axis[1]], [p[2], p[2] + tf_axis_length * y_axis[2]])
        tf_z.set_data_3d([p[0], p[0] + tf_axis_length * z_axis[0]], [p[1], p[1] + tf_axis_length * z_axis[1]], [p[2], p[2] + tf_axis_length * z_axis[2]])
        env.step(0.05)
    print(f"Animação '{label}' concluída. Feche a janela para sair.")
    plt.show(block=True)

def main():
    print("=== Simulação do Robô RRRP - Versão 10.0 (Cinemática Corrigida) ===")
    robot, l1, l2, l3 = create_real_robot()
    robot_params = (l1, l2, l3)
    if robot: print(f"Robô Real Criado! l1={l1}, l2={l2}, l3={l3}, d4=[0, 0.15]m")
    X_wall = 0.25
    Q_up, pts_ref_up = generate_frontal_trajectory(X_wall, robot_params, elbow_config='up')
    Q_down, pts_ref_down = generate_frontal_trajectory(X_wall, robot_params, elbow_config='down')
    if len(Q_up) > 0:
        if ROBOTICS_TOOLBOX_AVAILABLE: plot_trajectory_rtb(robot, Q_up, pts_ref_up, label=f"Cotovelo para Cima (X_wall={X_wall}m)", X_wall=X_wall)
    else: print(f"✗ ERRO: Trajetória 'up' não pôde ser gerada para X_wall={X_wall}m.")
    if len(Q_down) > 0:
        if ROBOTICS_TOOLBOX_AVAILABLE: plot_trajectory_rtb(robot, Q_down, pts_ref_down, label=f"Cotovelo para Baixo (X_wall={X_wall}m)", X_wall=X_wall)
    else: print(f"✗ ERRO: Trajetória 'down' não pôde ser gerada para X_wall={X_wall}m.")
    if len(Q_up) == 0 and len(Q_down) == 0: print(f"\nNenhuma trajetória foi gerada.")
    print("\nSimulação concluída!")

if __name__ == "__main__":
    main()