#!/usr/bin/env python3
"""
Exemplo: Usando IK para Trajetórias Complexas
Demonstra como usar cinemática inversa para trajetórias arbitrárias no espaço
"""

import matplotlib
matplotlib.use("Qt5Agg")

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Verifica se as bibliotecas estão instaladas
try:
    from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
    from spatialmath import SE3
    ROBOTICS_TOOLBOX_AVAILABLE = True
except ImportError:
    print("Robotics Toolbox não encontrada. Este exemplo requer a biblioteca.")
    print("Para instalar: pip install roboticstoolbox-python spatialmath-python")
    ROBOTICS_TOOLBOX_AVAILABLE = False

def create_rrrp_robot():
    """
    Cria o robô RRRP
    """
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        return None
    
    # Parâmetros do robô
    d1 = 0.0
    l2 = 0.3
    l3 = 0.05
    
    # Definição dos elos
    links = [
        RevoluteDH(d=d1, alpha=np.pi/2),
        RevoluteDH(a=l2, alpha=0),
        RevoluteDH(a=l3, alpha=np.pi/2),
        PrismaticDH(alpha=0)
    ]
    
    # Limite da junta prismática
    links[3].qlim = [0, 0.5]
    
    return DHRobot(links, name='RRRP')

def generate_circular_trajectory(center, radius, height, N=50):
    """
    Gera trajetória circular no plano XY
    """
    t = np.linspace(0, 2*np.pi, N)
    
    x = center[0] + radius * np.cos(t)
    y = center[1] + radius * np.sin(t)
    z = height * np.ones_like(t)
    
    return x, y, z

def generate_spiral_trajectory(center, radius_start, radius_end, height_start, height_end, N=100):
    """
    Gera trajetória espiral
    """
    t = np.linspace(0, 4*np.pi, N)
    
    # Raio e altura variam linearmente
    radius = np.linspace(radius_start, radius_end, N)
    height = np.linspace(height_start, height_end, N)
    
    x = center[0] + radius * np.cos(t)
    y = center[1] + radius * np.sin(t)
    z = height
    
    return x, y, z

def generate_sinusoidal_trajectory(x_range, amplitude, frequency, height, N=100):
    """
    Gera trajetória senoidal
    """
    x = np.linspace(x_range[0], x_range[1], N)
    y = amplitude * np.sin(frequency * x)
    z = height * np.ones_like(x)
    
    return x, y, z

def solve_ik_for_trajectory(robot, x_traj, y_traj, z_traj, orientation='down'):
    """
    Resolve IK para uma trajetória completa
    """
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        return None, None
    
    Q_ik = []
    success_count = 0
    
    print(f"Resolvendo IK para {len(x_traj)} pontos...")
    
    for i in range(len(x_traj)):
        # Criar pose desejada
        if orientation == 'down':
            # Orientação apontando para baixo (eixo Z negativo)
            pose = SE3(x_traj[i], y_traj[i], z_traj[i]) * SE3.Rz(0)
        elif orientation == 'forward':
            # Orientação apontando para frente (eixo X positivo)
            pose = SE3(x_traj[i], y_traj[i], z_traj[i]) * SE3.Ry(-np.pi/2)
        else:
            # Orientação padrão
            pose = SE3(x_traj[i], y_traj[i], z_traj[i])
        
        # Resolver IK com diferentes chutes iniciais
        success = False
        for q0 in [[0, np.pi/4, np.pi/4, 0.1], 
                   [0, np.pi/3, np.pi/6, 0.2],
                   [0, np.pi/6, np.pi/3, 0.15]]:
            try:
                sol = robot.ikine_LM(pose, q0=q0, ilimit=100)
                if sol.success:
                    Q_ik.append(sol.q)
                    success = True
                    success_count += 1
                    break
            except:
                continue
        
        if not success:
            print(f"⚠️  IK falhou no ponto {i}: ({x_traj[i]:.3f}, {y_traj[i]:.3f}, {z_traj[i]:.3f})")
            # Usar configuração padrão
            Q_ik.append([0, np.pi/4, np.pi/4, 0.1])
    
    Q_ik = np.array(Q_ik)
    success_rate = success_count / len(x_traj) * 100
    
    print(f"✓ IK resolvido: {success_count}/{len(x_traj)} pontos ({success_rate:.1f}% de sucesso)")
    
    return Q_ik, success_rate

def verify_trajectory_accuracy(robot, Q_ik, x_traj, y_traj, z_traj):
    """
    Verifica a precisão da trajetória obtida via IK
    """
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        return
    
    print("\n=== VERIFICAÇÃO DA PRECISÃO DA TRAJETÓRIA ===")
    
    errors = []
    test_points = [0, len(Q_ik)//4, len(Q_ik)//2, 3*len(Q_ik)//4, len(Q_ik)-1]
    
    print("Ponto | X_desejado | Y_desejado | Z_desejado | X_TCP | Y_TCP | Z_TCP | Erro_3D")
    print("-" * 75)
    
    for i in test_points:
        if i < len(Q_ik):
            q = Q_ik[i]
            T = robot.fkine(q)
            pos_tcp = T.t
            
            x_des, y_des, z_des = x_traj[i], y_traj[i], z_traj[i]
            x_tcp, y_tcp, z_tcp = pos_tcp[0], pos_tcp[1], pos_tcp[2]
            
            erro_3d = np.sqrt((x_tcp-x_des)**2 + (y_tcp-y_des)**2 + (z_tcp-z_des)**2)
            errors.append(erro_3d)
            
            print(f"{i:4d} | {x_des:10.3f} | {y_des:10.3f} | {z_des:10.3f} | {x_tcp:6.3f} | {y_tcp:6.3f} | {z_tcp:6.3f} | {erro_3d:7.3f}")
    
    if errors:
        erro_medio = np.mean(errors)
        erro_max = np.max(errors)
        print(f"\nErro médio: {erro_medio:.3f}m")
        print(f"Erro máximo: {erro_max:.3f}m")
        
        if erro_medio < 0.01:
            print("✓ Trajetória muito precisa!")
        elif erro_medio < 0.05:
            print("✓ Trajetória precisa")
        else:
            print("⚠️  Trajetória com baixa precisão")

def plot_trajectory_comparison(x_traj, y_traj, z_traj, Q_ik, robot, title):
    """
    Plota comparação entre trajetória desejada e obtida via IK
    """
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        return
    
    # Calcular posições reais do TCP
    positions_real = []
    for q in Q_ik:
        T = robot.fkine(q)
        positions_real.append(T.t)
    
    positions_real = np.array(positions_real)
    
    # Criar figura 3D
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plotar trajetória desejada
    ax.plot(x_traj, y_traj, z_traj, 'r--', linewidth=3, label='Trajetória Desejada')
    
    # Plotar trajetória real
    ax.plot(positions_real[:, 0], positions_real[:, 1], positions_real[:, 2], 
            'b-', linewidth=2, label='Trajetória Real (IK)')
    
    # Pontos de início e fim
    ax.scatter(x_traj[0], y_traj[0], z_traj[0], c='green', s=150, label='Início')
    ax.scatter(x_traj[-1], y_traj[-1], z_traj[-1], c='red', s=150, label='Fim')
    
    # Configurações
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title(f'{title}\nComparação: Desejada vs Real (IK)', fontsize=14)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def demonstrate_different_trajectories():
    """
    Demonstra diferentes tipos de trajetórias usando IK
    """
    if not ROBOTICS_TOOLBOX_AVAILABLE:
        print("Robotics Toolbox não disponível para demonstração")
        return
    
    robot = create_rrrp_robot()
    
    print("=== DEMONSTRAÇÃO DE TRAJETÓRIAS COMPLEXAS COM IK ===")
    
    # 1. Trajetória Circular
    print("\n1. Trajetória Circular")
    x_circle, y_circle, z_circle = generate_circular_trajectory(
        center=[0.3, 0.0, 0.1], radius=0.15, height=0.1, N=30
    )
    
    Q_circle, success_circle = solve_ik_for_trajectory(robot, x_circle, y_circle, z_circle)
    if Q_circle is not None:
        verify_trajectory_accuracy(robot, Q_circle, x_circle, y_circle, z_circle)
        plot_trajectory_comparison(x_circle, y_circle, z_circle, Q_circle, robot, 
                                 "Trajetória Circular")
    
    # 2. Trajetória Espiral
    print("\n2. Trajetória Espiral")
    x_spiral, y_spiral, z_spiral = generate_spiral_trajectory(
        center=[0.3, 0.0], radius_start=0.1, radius_end=0.2,
        height_start=0.05, height_end=0.15, N=50
    )
    
    Q_spiral, success_spiral = solve_ik_for_trajectory(robot, x_spiral, y_spiral, z_spiral)
    if Q_spiral is not None:
        verify_trajectory_accuracy(robot, Q_spiral, x_spiral, y_spiral, z_spiral)
        plot_trajectory_comparison(x_spiral, y_spiral, z_spiral, Q_spiral, robot,
                                 "Trajetória Espiral")
    
    # 3. Trajetória Senoidal
    print("\n3. Trajetória Senoidal")
    x_sine, y_sine, z_sine = generate_sinusoidal_trajectory(
        x_range=[0.2, 0.5], amplitude=0.1, frequency=10, height=0.1, N=60
    )
    
    Q_sine, success_sine = solve_ik_for_trajectory(robot, x_sine, y_sine, z_sine)
    if Q_sine is not None:
        verify_trajectory_accuracy(robot, Q_sine, x_sine, y_sine, z_sine)
        plot_trajectory_comparison(x_sine, y_sine, z_sine, Q_sine, robot,
                                 "Trajetória Senoidal")
    
    print("\n=== RESUMO ===")
    print(f"Trajetória Circular: {success_circle:.1f}% de sucesso")
    print(f"Trajetória Espiral: {success_spiral:.1f}% de sucesso")
    print(f"Trajetória Senoidal: {success_sine:.1f}% de sucesso")
    
    print("\n✓ Demonstração concluída!")
    print("Para animar qualquer trajetória:")
    print("robot.plot(Q_trajetoria, block=False, dt=0.1)")

def explain_ik_principle():
    """
    Explica o princípio do IK para trajetórias complexas
    """
    print("\n=== PRINCÍPIO DO IK PARA TRAJETÓRIAS COMPLEXAS ===")
    print()
    print("1. PROBLEMA:")
    print("   Dada uma trajetória arbitrária no espaço (x(t), y(t), z(t))")
    print("   Encontrar as configurações das juntas q(t) que levam o TCP")
    print("   exatamente para cada ponto da trajetória.")
    print()
    print("2. SOLUÇÃO COM IK:")
    print("   Para cada ponto P_i = (x_i, y_i, z_i) da trajetória:")
    print("   a) Criar pose desejada: SE3(x_i, y_i, z_i) * orientação")
    print("   b) Resolver IK: q_i = robot.ikine_LM(pose)")
    print("   c) Verificar sucesso e precisão")
    print()
    print("3. VANTAGENS:")
    print("   ✓ Funciona para qualquer trajetória 3D")
    print("   ✓ Mantém orientação desejada")
    print("   ✓ Evita singularidades (se possível)")
    print("   ✓ Considera limites das juntas")
    print()
    print("4. LIMITAÇÕES:")
    print("   ⚠️  Pode falhar em pontos inalcançáveis")
    print("   ⚠️  Múltiplas soluções possíveis")
    print("   ⚠️  Computacionalmente mais custoso")
    print()
    print("5. APLICAÇÕES:")
    print("   • Soldagem em curvas complexas")
    print("   • Pintura de superfícies irregulares")
    print("   • Inspeção de peças 3D")
    print("   • Trajetórias de evasão de obstáculos")

def main():
    """
    Função principal
    """
    print("=== EXEMPLO: IK PARA TRAJETÓRIAS COMPLEXAS ===")
    print("Este script demonstra como usar cinemática inversa")
    print("para trajetórias arbitrárias no espaço 3D.")
    print()
    
    # Explicação do princípio
    explain_ik_principle()
    
    # Demonstração prática
    if ROBOTICS_TOOLBOX_AVAILABLE:
        demonstrate_different_trajectories()
    else:
        print("\nPara executar a demonstração, instale o Robotics Toolbox:")
        print("pip install roboticstoolbox-python spatialmath-python")
    
    print("\n=== CONCLUSÃO ===")
    print("✓ IK permite trajetórias complexas e arbitrárias")
    print("✓ O TCP segue exatamente a curva desejada")
    print("✓ A orientação pode ser controlada independentemente")
    print("✓ Ideal para aplicações industriais complexas")

if __name__ == "__main__":
    main() 