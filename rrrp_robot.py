#!/usr/bin/env python3
"""
Simulação do Robô RRRP para Soldagem
Equivalente Python do código MATLAB original

Autor: André MK022 (Cuca)
Descrição: Robô RRRP com trajetória que compensa o consumo do eletrodo
"""

# Configurar backend do matplotlib para Qt5
import matplotlib
matplotlib.use("Qt5Agg")

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# Verifica se as bibliotecas estão instaladas
try:
    from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
    from spatialmath import SE3
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("Robotics Toolbox não encontrada. Usando simulação básica.")
    print("Para instalar: pip install roboticstoolbox-python spatialmath-python swift-sim")
    ROBOTICS_TOOLBOX_AVAILABLE = False

def create_rrrp_robot():
    """
    Cria o robô RRRP com parâmetros DH
    RRRP = 3 juntas rotacionais + 1 junta prismática
    """
    # Parâmetros do robô (em metros)
    d1 = 0.0    # Offset vertical da base
    l2 = 0.3    # Comprimento do link 2
    l3 = 0.05   # Comprimento do link 3
    
    if ROBOTICS_TOOLBOX_AVAILABLE:
        # Definição dos elos conforme DH
        links = [
            RevoluteDH(d=d1,       alpha=np.pi/2),   # Junta 1: revoluta, base vertical
            RevoluteDH(a=l2,       alpha=0),         # Junta 2: revoluta, plano vertical
            RevoluteDH(a=l3,       alpha=np.pi/2),   # Junta 3: revoluta, continua vertical
            PrismaticDH(alpha=0)                     # Junta 4: prismática, compensação
        ]
        
        # Limite do curso da junta prismática (50 cm)
        links[3].qlim = [0, 0.5]
        
        # Cria o robô
        robot = DHRobot(links, name='RRRP')
        return robot, l2, l3
    else:
        # Retorna parâmetros para simulação básica
        return None, l2, l3

def generate_trajectory(l2, l3, N=100):
    """
    Gera trajetória de soldagem que compensa o consumo do eletrodo
    """
    # Trajetória desejada em X (linear)
    x_desejado = np.linspace(0.2, 0.2 + 0.3, N)
    
    # Configurações das juntas
    q1 = 0.0  # Junta 1 fixa
    q2_vals = np.linspace(np.pi/2, 0.0, N)  # Junta 2: de 90° a 0°
    q3_vals = np.pi/2 - q2_vals             # Junta 3: mantém orientação
    
    # Matriz de configurações
    Q = np.zeros((N, 4))
    
    # Loop para calcular q4 que garante x(t) desejado
    for i in range(N):
        q2 = q2_vals[i]
        q3 = q3_vals[i]
        
        # Compensação com junta prismática para manter X linear
        q4 = x_desejado[i] - l2 * np.cos(q2)
        
        Q[i, :] = [q1, q2, q3, q4]
    
    return Q, x_desejado

def forward_kinematics_basic(q, l2, l3):
    """
    Cinemática direta básica para visualização sem robotics toolbox
    """
    q1, q2, q3, q4 = q
    
    # Matrizes de transformação homogênea
    # Link 1: rotação em Z, depois translação em Y
    T1 = np.array([
        [np.cos(q1), -np.sin(q1), 0, 0],
        [np.sin(q1), np.cos(q1), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Link 2: rotação em Z, depois translação em X
    T2 = np.array([
        [np.cos(q2), -np.sin(q2), 0, l2*np.cos(q2)],
        [np.sin(q2), np.cos(q2), 0, l2*np.sin(q2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Link 3: rotação em Z, depois translação em X
    T3 = np.array([
        [np.cos(q3), -np.sin(q3), 0, l3*np.cos(q3)],
        [np.sin(q3), np.cos(q3), 0, l3*np.sin(q3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Link 4: translação em Z (prismática)
    T4 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, q4],
        [0, 0, 0, 1]
    ])
    
    # Transformação total
    T_total = T1 @ T2 @ T3 @ T4
    
    return T_total[:3, 3]  # Retorna posição (x, y, z)

def plot_trajectory_basic(Q, l2, l3):
    """
    Plota a trajetória usando matplotlib básico
    """
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Calcula posições do efetuador
    positions = []
    for q in Q:
        pos = forward_kinematics_basic(q, l2, l3)
        positions.append(pos)
    
    positions = np.array(positions)
    
    # Plota trajetória
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2, label='Trajetória')
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='g', s=100, label='Início')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='r', s=100, label='Fim')
    
    # Configurações do gráfico
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Trajetória do Robô RRRP - Soldagem')
    ax.legend()
    ax.grid(True)
    
    # Ajusta limites dos eixos
    ax.set_xlim([0, 0.6])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([0, 0.6])
    
    plt.tight_layout()
    plt.show()

def animate_trajectory(robot, Q, V=5):
    """
    Anima a trajetória usando robotics toolbox
    """
    if robot is None:
        print("Robotics Toolbox não disponível para animação")
        return
    
    print("Animando trajetória...")
    print("Pressione 'q' para sair da animação")
    
    try:
        # Usar backend Qt5Agg já configurado
        # Anima o movimento da trajetória
        robot.plot(Q, block=False, movie='rrrp_animation.gif', 
                  backend='pyplot', dt=0.1, limits=[-0.5, 0.8, -0.5, 0.5, 0, 0.8])
        print("Animação salva como 'rrrp_animation.gif'")
        
    except Exception as e:
        print(f"Erro na animação: {e}")
        print("Tentando plot simples...")
        try:
            # Plot simples sem animação
            robot.plot(Q[0], block=True, limits=[-0.5, 0.8, -0.5, 0.5, 0, 0.8])
        except Exception as e2:
            print(f"Erro no plot simples: {e2}")
            print("Usando visualização básica...")
            plot_trajectory_basic(Q, 0.3, 0.05)

def main():
    """
    Função principal
    """
    print("=== Simulação do Robô RRRP para Soldagem ===")
    print("Equivalente Python do código MATLAB")
    print()
    
    # Cria o robô
    robot, l2, l3 = create_rrrp_robot()
    
    if robot:
        print(f"Robô RRRP criado com sucesso!")
        print(f"Parâmetros: l2={l2}m, l3={l3}m")
        print(f"Junta prismática: curso de 0 a 0.5m")
    else:
        print("Usando simulação básica (sem Robotics Toolbox)")
    
    # Gera trajetória
    print("\nGerando trajetória de soldagem...")
    Q, x_desejado = generate_trajectory(l2, l3, N=100)
    
    print(f"Trajetória gerada com {len(Q)} pontos")
    print(f"X inicial: {x_desejado[0]:.3f}m")
    print(f"X final: {x_desejado[-1]:.3f}m")
    
    # Mostra algumas configurações
    print("\nPrimeiras configurações das juntas:")
    for i in range(min(5, len(Q))):
        print(f"  t={i}: q1={Q[i,0]:.3f}, q2={Q[i,1]:.3f}, q3={Q[i,2]:.3f}, q4={Q[i,3]:.3f}")
    
    # Plota trajetória
    if robot and ROBOTICS_TOOLBOX_AVAILABLE:
        print("\nIniciando animação com Robotics Toolbox...")
        animate_trajectory(robot, Q)
    else:
        print("\nPlotando trajetória com matplotlib...")
        plot_trajectory_basic(Q, l2, l3)
    
    print("\nSimulação concluída!")

if __name__ == "__main__":
    main() 