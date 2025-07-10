#!/usr/bin/env python3
"""
Simulação do Robô RRRP - kinematics_planner.py

Autor: André MK022 (Cuca) - Refinado com feedback por Gemini
Descrição: Script responsável pelo planejamento cinemático. Ele define o robô,
calcula uma trajetória cinematicamente válida e salva as posições desejadas
das juntas em um arquivo para ser usado pelo simulador do controlador.
"""
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH

def create_real_robot():
    """Cria um robô RRR com as dimensões e limites de junta do hardware real."""
    l1, l2, l3 = 0.1, 0.2, 0.1
    q1_lim, q2_lim, q3_lim = [0, 0], [0, np.pi], [-np.pi/2, np.pi/2]
    links = [
        RevoluteDH(d=l1, alpha=np.pi/2, qlim=q1_lim),
        RevoluteDH(a=l2, alpha=0, qlim=q2_lim),
        RevoluteDH(a=l3, alpha=0, qlim=q3_lim),
    ]
    return DHRobot(links, name='Real_RRR_Base'), l1, l2, l3

def solve_ik_with_constraints_corrected(robot_params, q4_limits, horizontal_reach, N, elbow_config='up'):
    """Resolve a cinemática inversa usando a geometria exata do robô."""
    l1, l2, l3 = robot_params
    q4_min, q4_max = q4_limits
    q2_min, q2_max = 0, np.pi
    q3_min, q3_max = -np.pi/2, np.pi/2
    z_range = (l1, l1 + l2 + l3 - 0.001)
    z_vals = np.linspace(z_range[0], z_range[1], N)
    if elbow_config == 'up': q3_range = (q3_max, 0)
    else: q3_range = (q3_min, 0)
    q3_vals = np.linspace(q3_range[0], q3_range[1], N)
    elbow_sign = 1.0 if elbow_config == 'up' else -1.0
    Q_partial = []
    for i, z_target in enumerate(z_vals):
        q3 = q3_vals[i]
        x_p, z_p = horizontal_reach, z_target - l1
        D_sq = x_p**2 + z_p**2
        a = 1.0
        b = 2 * (l3 + l2 * np.cos(q3))
        c = l2**2 + l3**2 + 2*l2*l3*np.cos(q3) - D_sq
        discriminant = b**2 - 4*a*c
        if discriminant < 0: continue
        sqrt_discriminant = np.sqrt(discriminant)
        q4_sol1 = (-b + sqrt_discriminant) / (2*a)
        q4_sol2 = (-b - sqrt_discriminant) / (2*a)
        q4 = -1
        if q4_min <= q4_sol1 <= q4_max: q4 = q4_sol1
        elif q4_min <= q4_sol2 <= q4_max: q4 = q4_sol2
        else: continue
        L_eff = l3 + q4
        gamma = np.arctan2(z_p, x_p)
        acos_arg = (l2**2 + D_sq - L_eff**2) / (2 * l2 * np.sqrt(D_sq))
        if not (-1.0 <= acos_arg <= 1.0): continue
        beta = np.arccos(acos_arg)
        q2 = gamma - elbow_sign * beta
        if not (q2_min <= q2 <= q2_max): continue
        Q_partial.append([q2, q3, q4, z_target])
    return np.array(Q_partial)

def generate_frontal_trajectory(X_wall, robot_params, elbow_config, N=200):
    """Gera a trajetória para uma parede frontal."""
    print(f"Calculando trajetória '{elbow_config}' para parede frontal em x={X_wall}m...")
    q4_limits = [0, 0.15]
    Q_partial = solve_ik_with_constraints_corrected(robot_params, q4_limits, X_wall, N, elbow_config)
    if len(Q_partial) == 0: return np.array([])
    num_valid_points = len(Q_partial)
    Q = np.zeros((num_valid_points, 4)); Q[:, 0] = 0; Q[:, 1:4] = Q_partial[:, 0:3]
    return Q

def main():
    """Função principal para gerar e salvar a trajetória."""
    print("=== Planejador de Trajetória Cinemática ===")
    _, l1, l2, l3 = create_real_robot()
    robot_params = (l1, l2, l3)
    
    # --- PARÂMETROS DE PLANEJAMENTO ---
    X_wall = 0.25  # Distância da parede em metros
    configuracao = 'up' # Escolha 'up' ou 'down'
    
    # Gera a trajetória de juntas desejada
    q_desejado = generate_frontal_trajectory(X_wall, robot_params, elbow_config=configuracao)

    if q_desejado.size == 0:
        print(f"✗ ERRO: Não foi possível gerar a trajetória para X_wall={X_wall}m.")
        return

    # Salva a trajetória em um arquivo .npy
    np.save('trajetoria_desejada.npy', q_desejado)
    print(f"\n✓ Trajetória '{configuracao}' com {len(q_desejado)} pontos foi gerada e salva em 'trajetoria_desejada.npy'")

if __name__ == "__main__":
    main()