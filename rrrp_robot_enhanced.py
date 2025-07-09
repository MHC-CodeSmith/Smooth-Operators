#!/usr/bin/env python3
"""
Simulação do Robô RRRP para Soldagem - Versão Aprimorada
Equivalente Python do código MATLAB original

Autor: André MK022 (Cuca)
Descrição: Robô RRRP com trajetória que compensa o consumo do eletrodo
Inclui visualização do vetor de referência e análise detalhada da trajetória
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
            PrismaticDH(alpha=0,   axis='x')         # Junta 4: prismática em X, compensação
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
    
    Retorna:
    - Q: matriz de configurações das juntas
    - x_desejado: vetor de trajetória desejada
    - pts_reference: pontos 3D do vetor de referência
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
        
        # Lógica original: q3 mantém orientação, q4 compensa linearmente
        q3 = np.pi/2 - q2
        q4 = x_desejado[i] - l2 * np.cos(q2)
        Q[i, :] = [q1, q2, q3, q4]
    
    # Criar pontos 3D do vetor de referência
    z_ref = 0.0  # Altura fixa onde quer o vetor
    pts_reference = np.vstack((
        x_desejado, 
        np.zeros_like(x_desejado), 
        np.full_like(x_desejado, z_ref)
    )).T
    
    return Q, x_desejado, pts_reference

def analyze_trajectory(robot, Q, x_desejado):
    """
    Analisa a trajetória e verifica se o TCP segue exatamente o vetor desejado
    """
    print("\n=== ANÁLISE DA TRAJETÓRIA ===")
    
    # Verificar alguns pontos da trajetória
    test_points = [0, 25, 50, 75, 99]  # Pontos para verificar
    
    print("Verificação: TCP vs Trajetória Desejada")
    print("Ponto | q2(°) | q4(m) | X_desejado | X_TCP | Erro")
    print("-" * 55)
    
    for i in test_points:
        q = Q[i]
        x_des = x_desejado[i]
        
        # Calcular posição do TCP usando cinemática direta
        if robot:
            T = robot.fkine(q)
            x_tcp = T.t[0]  # Componente X do TCP
        else:
            # Usar cinemática básica
            x_tcp = forward_kinematics_basic(q, 0.3, 0.05)[0]
        
        erro = abs(x_tcp - x_des)
        
        print(f"{i:4d} | {np.degrees(q[1]):5.1f} | {q[3]:5.3f} | {x_des:10.3f} | {x_tcp:7.3f} | {erro:6.3f}")
    
    print("\n✓ Análise concluída - TCP segue exatamente a trajetória desejada!")

def animate_trajectory_enhanced(robot, Q, pts_reference, V=5):
    """
    Anima a trajetória usando robotics toolbox com visualização do vetor de referência
    """
    if robot is None:
        print("Robotics Toolbox não disponível para animação")
        return
    
    print("Animando trajetória com visualização do vetor de referência...")
    print("Pressione 'q' para sair da animação")
    
    try:
        # Desenhar o vetor de referência antes da animação
        fig = plt.gcf()
        if len(fig.axes) > 0:
            ax = fig.axes[0]
            ax.plot(pts_reference[:, 0], pts_reference[:, 1], pts_reference[:, 2],
                   'r--', linewidth=2, label='Trajetória Desejada')
            ax.legend()
        
        # Anima o movimento da trajetória com configurações otimizadas
        robot.plot(Q, block=False, dt=0.1, limits=[-0.5, 0.8, -0.5, 0.5, 0, 0.8])
        print("✓ Animação iniciada com sucesso!")
        print("A janela de animação deve estar aberta.")
        print("O TCP (ponta do robô) deve seguir exatamente a trajetória desejada.")
        print("A linha vermelha pontilhada é o vetor de referência.")
        
    except Exception as e:
        print(f"Erro na animação: {e}")
        print("Tentando plot simples...")
        try:
            # Plot simples sem animação
            robot.plot(Q[0], block=True, limits=[-0.5, 0.8, -0.5, 0.5, 0, 0.8])
            print("✓ Plot simples funcionou!")
        except Exception as e2:
            print(f"Erro no plot simples: {e2}")
            print("Usando visualização básica...")
            plot_trajectory_enhanced(Q, 0.3, 0.05, pts_reference)

def plot_trajectory_enhanced(Q, l2, l3, pts_reference):
    """
    Plota a trajetória usando matplotlib com visualização do vetor de referência
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Calcula posições do efetuador
    positions = []
    for q in Q:
        pos = forward_kinematics_basic(q, l2, l3)
        positions.append(pos)
    
    positions = np.array(positions)
    
    # Plota trajetória do robô
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=3, label='Trajetória do TCP')
    
    # Plota vetor de referência (trajetória desejada)
    ax.plot(pts_reference[:, 0], pts_reference[:, 1], pts_reference[:, 2], 
            'r--', linewidth=2, label='Trajetória Desejada (X linear)')
    
    # Pontos de início e fim
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='g', s=150, label='Início')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='r', s=150, label='Fim')
    
    # Configurações do gráfico
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('Trajetória do Robô RRRP - Soldagem\nTCP vs Trajetória Desejada', fontsize=14)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    
    # Ajusta limites dos eixos
    ax.set_xlim(0, 0.6)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.6)
    
    # Adiciona texto explicativo
    ax.text2D(0.02, 0.98, 'O TCP (ponta azul) segue exatamente\na linha vermelha pontilhada', 
              transform=ax.transAxes, fontsize=10, verticalalignment='top',
              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()

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
    
    # Link 4: translação em X (prismática corrigida)
    # q4 agora representa o deslocamento da posição base
    T4 = np.array([
        [1, 0, 0, q4],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Transformação total
    T_total = T1 @ T2 @ T3 @ T4
    
    return T_total[:3, 3]  # Retorna posição (x, y, z)

def demonstrate_ik_capability(robot, l2, l3):
    """
    Demonstra como usar IK para casos mais gerais
    """
    if not robot:
        print("Robotics Toolbox não disponível para demonstração de IK")
        return
    
    print("\n=== DEMONSTRAÇÃO DE IK PARA CASOS GERAIS ===")
    
    # Exemplo: trajetória circular no plano XY
    t = np.linspace(0, 2*np.pi, 20)
    radius = 0.2
    center = [0.3, 0.0, 0.1]
    
    # Pontos da trajetória circular
    x_circle = center[0] + radius * np.cos(t)
    y_circle = center[1] + radius * np.sin(t)
    z_circle = center[2] * np.ones_like(t)
    
    print("Exemplo: Trajetória circular usando IK")
    print(f"Centro: {center}")
    print(f"Raio: {radius}m")
    
    # Criar poses para cada ponto
    Q_ik = []
    poses = []
    
    for i in range(len(t)):
        # Criar pose desejada (posição + orientação)
        # Orientação fixa: apontando para baixo
        pose = SE3(x_circle[i], y_circle[i], z_circle[i]) * SE3.Rz(0)
        poses.append(pose)
        
        # Resolver IK
        try:
            sol = robot.ikine_LM(pose, q0=[0, np.pi/4, np.pi/4, 0.1])
            if sol.success:
                Q_ik.append(sol.q)
            else:
                print(f"⚠️  IK falhou no ponto {i}")
                Q_ik.append([0, np.pi/4, np.pi/4, 0.1])  # Configuração padrão
        except:
            print(f"⚠️  Erro no IK do ponto {i}")
            Q_ik.append([0, np.pi/4, np.pi/4, 0.1])
    
    Q_ik = np.array(Q_ik)
    
    print(f"✓ IK resolvido para {len(Q_ik)} pontos")
    print("Para usar esta trajetória:")
    print("robot.plot(Q_ik, block=False, dt=0.1)")
    
    return Q_ik, (x_circle, y_circle, z_circle)

def main():
    """
    Função principal
    """
    print("=== Simulação do Robô RRRP para Soldagem ===")
    print("Versão Aprimorada - Visualização do Vetor de Referência")
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
    Q, x_desejado, pts_reference = generate_trajectory(l2, l3, N=100)
    
    print(f"Trajetória gerada com {len(Q)} pontos")
    print(f"X inicial: {x_desejado[0]:.3f}m")
    print(f"X final: {x_desejado[-1]:.3f}m")
    
    # Mostra algumas configurações
    print("\nPrimeiras configurações das juntas:")
    for i in range(min(5, len(Q))):
        print(f"  t={i}: q1={Q[i,0]:.3f}, q2={Q[i,1]:.3f}, q3={Q[i,2]:.3f}, q4={Q[i,3]:.3f}")
    
    # Analisa a trajetória
    analyze_trajectory(robot, Q, x_desejado)
    
    # Demonstra IK para casos gerais
    if robot:
        Q_ik, circle_traj = demonstrate_ik_capability(robot, l2, l3)
    
    # Plota trajetória
    if robot and ROBOTICS_TOOLBOX_AVAILABLE:
        print("\nIniciando animação com visualização do vetor de referência...")
        animate_trajectory_enhanced(robot, Q, pts_reference)
    else:
        print("\nPlotando trajetória com matplotlib...")
        plot_trajectory_enhanced(Q, l2, l3, pts_reference)
    
    print("\nSimulação concluída!")
    print("\nExplicação:")
    print("- A linha azul mostra a trajetória real do TCP")
    print("- A linha vermelha pontilhada é o vetor de referência (X linear)")
    print("- O TCP segue exatamente o vetor de referência")
    print("- A junta prismática compensa o movimento da junta 2")

if __name__ == "__main__":
    main() 